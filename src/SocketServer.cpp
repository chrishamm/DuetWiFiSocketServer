/*
 * SocketServer.cpp
 *
 *  Created on: 25 Mar 2017
 *      Authors: David and Christian
 */

#include <algorithm>
#include <cstring>

#include "ecv.h"

#include <espressif/esp_common.h>
#include <esp/gpio.h>
#include <esp/uart.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <esp8266.h>

#include <dhcpserver.h>

#include <xtensa_ops.h>
#include <malloc.h>
#include <sys/unistd.h>
#include <esp/rtc_regs.h>

#include "SocketServer.h"
#include "Config.h"
#include "Connection.h"
#include "Listener.h"

#include "EEPROM.h"
#include "HSPI.h"

extern "C"
{
	#include <espressif/user_interface.h>

	#include "lwip/api.h"
	#include "lwip/netif.h"
	#include "lwip/stats.h"			// for stats_display()
	//#include "lwip/app/netbios.h"	// for NetBIOS support

	extern uint8_t sdk_rtc_get_reset_reason();
}

#define array _ecv_array

// DC lengthened the timeout at version 1.19beta8 because we now detect most connection failures without timing out
const uint32_t MaxConnectTime = 60 * 1000;		// how long we wait for WiFi to connect in milliseconds
const int DefaultWiFiChannel = 6;

// Global data
char currentSsid[SsidLength + 1] = "";
char webHostName[HostNameLength + 1] = "Duet-WiFi";

const char* lastError = nullptr;
const char* prevLastError = nullptr;

int8_t apRssi = 0;
char lastConnectError[100];

WiFiState currentState = WiFiState::idle;

static HSPIClass hspi;
static QueueHandle_t connectSemaphore;
static uint32_t connectStartTime;

static uint32_t transferBuffer[NumDwords(MaxDataLength + 1)];



// Look up a SSID in our remembered network list, return true if found
bool RetrieveSsidData(const char *ssid, WirelessConfigurationData& ssidData, size_t *index = nullptr)
{
	WirelessConfigurationData dummy;
	for (size_t i = 1; i <= MaxRememberedNetworks; ++i)
	{
		EEPROM.get(i * sizeof(WirelessConfigurationData), dummy);
		if (strncmp(ssid, dummy.ssid, sizeof(dummy.ssid)) == 0)
		{
			memcpy(&ssidData, &dummy, sizeof(WirelessConfigurationData));
			if (index != nullptr)
			{
				*index = i;
			}
			return true;
		}
	}
	return false;
}

// Find an empty entry in the table of known networks
bool FindEmptySsidEntry(size_t *index)
{
	for (size_t i = 1; i <= MaxRememberedNetworks; ++i)
	{
		WirelessConfigurationData tempData;
		EEPROM.get(i * sizeof(WirelessConfigurationData), tempData);
		if (tempData.ssid[0] == 0xFF)
		{
			*index = i;
			return true;
		}
	}
	return false;
}

// Check socket number in range, returning trus if yes. Otherwise, set lastError and return false;
bool ValidSocketNumber(uint8_t num)
{
	if (num < NumTcpSockets)
	{
		return true;
	}
	lastError = "socket number out of range";
	return false;
}

// Reset to default settings
void FactoryReset()
{
	WirelessConfigurationData temp;
	memset(&temp, 0xFF, sizeof(temp));
	for (size_t i = 0; i <= MaxRememberedNetworks; ++i)
	{
		EEPROM.put(i * sizeof(WirelessConfigurationData), temp);
	}
	EEPROM.commit();
}

// Try to connect using the saved SSID and password
void ConnectToAccessPoint(const WirelessConfigurationData& apData)
pre(currentState == NetworkState::disabled)
{
	// initialise some values
	currentState = WiFiState::connecting;
	connectStartTime = xTaskGetTickCount();
	strncpy(currentSsid, apData.ssid, ARRAY_SIZE(currentSsid) - 1);
	currentSsid[ARRAY_SIZE(currentSsid) - 1] = 0;

	// set hostname
	netif_set_hostname(netif_default, webHostName);

	// set IP address details
	if (apData.ip == 0)
	{
		sdk_wifi_station_dhcpc_start();
	}
	else
	{
		sdk_wifi_station_dhcpc_stop();

		struct ip_info info;
		info.ip.addr = apData.ip;
		info.netmask.addr = apData.netmask;
		info.gw.addr = apData.gateway;
		if (!sdk_wifi_set_ip_info(STATION_IF, &info))
		{
			lastError = "Failed to set IP address config";
			currentState = WiFiState::idle;
			return;
		}
	}

	// set AP details
	struct sdk_station_config config;
	config.bssid_set = 0;	// advised by SDK manual
	memcpy(config.ssid, apData.ssid, ARRAY_SIZE(config.ssid));
	memcpy(config.password, apData.password, ARRAY_SIZE(config.password));

	if (!sdk_wifi_station_set_config(&config))
	{
		lastError = "Failed to set WiFi config";
		currentState = WiFiState::idle;
		return;
	}

	// try to connect
	if (!sdk_wifi_station_connect())
	{
		lastError = "Failed to connect to AP";
		currentState = WiFiState::idle;
	}
}

void ConnectPoll()
{
	if (currentState == WiFiState::connecting)
	{
		// The Arduino WiFi.status() call is fairly useless here because it discards too much information, so use the SDK API call instead
		const char *error = nullptr;
		const uint8_t status = sdk_wifi_station_get_connect_status();
		switch (status)
		{
		case STATION_IDLE:
			error = "Unexpected WiFi state 'idle'";
			break;

		case STATION_CONNECTING:
			if ((xTaskGetTickCount() - connectStartTime) * portTICK_PERIOD_MS >= MaxConnectTime)
			{
				error = "Timed out";
			}
			break;

		case STATION_WRONG_PASSWORD:
			error = "Wrong password";
			break;

		case STATION_NO_AP_FOUND:
			error = "Didn't find access point";
			break;

		case STATION_CONNECT_FAIL:
			error = "Failed";
			break;

		case STATION_GOT_IP:
			currentState = WiFiState::connected;
			gpio_write(EspReqTransferPin, false);		// force a status update when complete
			vTaskDelay(2 / portTICK_PERIOD_MS);
			break;

		default:
			error = "Unknown WiFi status";
			break;
		}

		if (error != nullptr)
		{
			sdk_wifi_set_opmode(NULL_MODE);
			currentState = WiFiState::idle;

			strcpy(lastConnectError, error);
			strncat(lastConnectError, " while trying to connect to ", ARRAY_SIZE(lastConnectError) - strlen(lastConnectError) - 1);
			strncat(lastConnectError, currentSsid, ARRAY_SIZE(lastConnectError) - strlen(lastConnectError) - 1);
			lastError = lastConnectError;
			debugPrintf("failed to connect to AP\n");
		}
	}
}

void WiFiScanDone(void *arg, sdk_scan_status_t status)
{
	if (status != SCAN_OK)
	{
		lastError = "WiFi scan failed";
		currentState = WiFiState::idle;
		xSemaphoreGive(connectSemaphore);
		return;
	}

	// First one is invalid
	struct sdk_bss_info *bss = (struct sdk_bss_info *)arg;
	bss = bss->next.stqe_next;

	// Find the strongest network that we know about
	struct sdk_bss_info *strongestBss = nullptr;
	char ssid[33]; // max SSID length + zero byte
	WirelessConfigurationData ssidData;

	while (bss != nullptr)
	{
		size_t len = strlen((const char *)bss->ssid);
		memcpy(ssid, bss->ssid, len);
		ssid[len] = 0;

		if ((strongestBss == nullptr || bss->rssi > strongestBss->rssi) && RetrieveSsidData(ssid, ssidData))
		{
			strongestBss = bss;
		}
		bss = bss->next.stqe_next;
	}

	if (strongestBss == nullptr)
	{
		lastError = "no known networks found";
		currentState = WiFiState::idle;
		xSemaphoreGive(connectSemaphore);
		return;
	}

	apRssi = strongestBss->rssi;
	ConnectToAccessPoint(ssidData);
	xSemaphoreGive(connectSemaphore);
}

void StartClient()
{
	// Switch to station mode, we're connecting to an access point
	if (!sdk_wifi_set_opmode(STATION_MODE))
	{
		lastError = "Could not change to station mode";
		currentState = WiFiState::idle;
		return;
	}

	// Auto scan for strongest known network, then try to connect to it
	struct sdk_scan_config config;
	config.ssid = 0;
	config.bssid = 0;
	config.channel = 0;
	config.show_hidden = false;
	if (!sdk_wifi_station_scan(&config, WiFiScanDone))
	{
		lastError = "WiFi scan failed";
		currentState = WiFiState::idle;
	}

	// Wait for the scan to complete
	xSemaphoreTake(connectSemaphore, portMAX_DELAY);
}

bool CheckValidString(const char * array s, size_t n, bool isSsid)
{
	for (size_t i = 0; i < n; ++i)
	{
		char c = s[i];
		if (c == 0)
		{
			return i != 0 || !isSsid;		// the SSID may not be empty but the password can be
		}
		if (c < 0x20 || c == 0x7F)
		{
			return false;					// bad character
		}
	}
	return false;							// no null terminator
}

// Check that the access point data is valid
bool ValidApData(const WirelessConfigurationData &apData)
{
	// Check the IP address
	if (apData.ip == 0 || apData.ip == 0xFFFFFFFF)
	{
		return false;
	}

	// Check the channel. 0 means auto so it OK.
	if (apData.channel > 13)
	{
		return false;
	}

	return CheckValidString(apData.ssid, SsidLength, true) && CheckValidString(apData.password, PasswordLength, false);
}

void StartAccessPoint()
{
	WirelessConfigurationData apData;
	EEPROM.get(0, apData);

	if (ValidApData(apData))
	{
		// change to AP mode
		if (!sdk_wifi_set_opmode(SOFTAP_MODE))
		{
			sdk_wifi_set_opmode(NULL_MODE);
			lastError = "Failed to change to AP mode";
			currentState = WiFiState::idle;
			return;
		}

		// set IP address
		struct ip_info ipInfo;
		ipInfo.ip.addr = apData.ip;
		if (apData.netmask == 0)
		{
			IP4_ADDR(&ipInfo.netmask, 255, 255, 255, 0);
		}
		else
		{
			ipInfo.netmask.addr = apData.netmask;
		}
		ipInfo.gw.addr = apData.gateway;
		if (!sdk_wifi_set_ip_info(SOFTAP_IF, &ipInfo))
		{
			sdk_wifi_set_opmode(NULL_MODE);
			lastError = "Failed to set IP";
			currentState = WiFiState::idle;
			return;
		}

		// set AP information
		struct sdk_softap_config apConfig;
		memcpy(apConfig.ssid, apData.ssid, ARRAY_SIZE(apConfig.ssid));
		apConfig.authmode = AUTH_WPA2_PSK;
		apConfig.beacon_interval = 100;
		apConfig.channel = (apData.channel == 0) ? DefaultWiFiChannel : apData.channel;
		apConfig.max_connection = MaxConnections;
		memcpy(apConfig.password, apData.password, ARRAY_SIZE(apConfig.password));
		apConfig.ssid_hidden = 0;
		apConfig.ssid_len = strlen(apData.ssid);

		if (!sdk_wifi_softap_set_config(&apConfig))
		{
			sdk_wifi_set_opmode(NULL_MODE);
			lastError = "Could not set AP config";
			currentState = WiFiState::idle;
			return;
		}

		// Set up DHCP server
	    ip_addr_t dhcpFirstClientIp;
	    dhcpFirstClientIp.addr  =    apData.ip & 0x00FFFFFF;
	    dhcpFirstClientIp.addr |= (((apData.ip & 0xFF) > 100) ? 1 : 100) << 24;
	    dhcpserver_start(&dhcpFirstClientIp, MaxConnections);

	    // Set up DNS server
	    /*if (!dns.start(53))
	    {
			sdk_wifi_set_opmode(NULL_MODE);
			lastError = "Failed to start DNS";
			currentState = WiFiState::idle;
	    	return;
	    }*/

	    debugPrintf("AP started\n");
	    strncpy(currentSsid, apData.ssid, ARRAY_SIZE(currentSsid) - 1);
	    currentSsid[ARRAY_SIZE(currentSsid) - 1] = 0;

	    currentState = WiFiState::runningAsAccessPoint;
	    gpio_write(EspReqTransferPin, false);		// force a status update when complete
	    vTaskDelay(2 / portTICK_PERIOD_MS);
	}
	else
	{
		lastError = "invalid access point configuration";
		currentState = WiFiState::idle;
	}
}

static union
{
	MessageHeaderSamToEsp hdr;			// the actual header
	uint32_t asDwords[headerDwords];	// to force alignment
} messageHeaderIn;

static union
{
	MessageHeaderEspToSam hdr;
	uint32_t asDwords[headerDwords];	// to force alignment
} messageHeaderOut;

// Rebuild the mDNS services
void RebuildServices()
{
	/*MDNS.deleteServices();

	// Unfortunately the official ESP8266 mDNS library only reports one service.
	// I (chrishamm) tried to use the old mDNS responder, which is also capable of sending
	// mDNS broadcasts, but the packets it generates are broken and thus not of use.
	const uint16_t httpPort = Listener::GetPortByProtocol(0);
	if (httpPort != 0)
	{
		MDNS.addService("http", "tcp", httpPort);
		MDNS.addServiceTxt("http", "tcp", "product", "DuetWiFi");
		MDNS.addServiceTxt("http", "tcp", "version", firmwareVersion);
	}
	else
	{
		const uint16_t ftpPort = Listener::GetPortByProtocol(1);
		if (ftpPort != 0)
		{
			MDNS.addService("ftp", "tcp", ftpPort);
			MDNS.addServiceTxt("ftp", "tcp", "product", "DuetWiFi");
			MDNS.addServiceTxt("ftp", "tcp", "version", firmwareVersion);
		}
		else
		{
			const uint16_t telnetPort = Listener::GetPortByProtocol(2);
			if (telnetPort != 0)
			{
				MDNS.addService("telnet", "tcp", telnetPort);
				MDNS.addServiceTxt("telnet", "tcp", "product", "DuetWiFi");
				MDNS.addServiceTxt("telnet", "tcp", "version", firmwareVersion);
			}
		}
	}*/
}

// Helper function to get the IP address
uint32_t GetIPAddress()
{
	unsigned char ifIndex;
	switch (currentState)
	{
	case WiFiState::connected:
		ifIndex = STATION_IF;
		break;

	case WiFiState::runningAsAccessPoint:
		ifIndex = SOFTAP_IF;
		break;

	default:
		return 0;
	}

	struct ip_info info;
	sdk_wifi_get_ip_info(ifIndex, &info);
	return info.ip.addr;
}

// Helper function to get the amount of free heap memory
size_t GetFreeHeap()
{
    extern uint32_t xPortSupervisorStackPointer;
    struct mallinfo mi = mallinfo();
    uint32_t brk_val = (uint32_t)sbrk(0);

    uint32_t sp = xPortSupervisorStackPointer;
    if(sp == 0)
    {
        SP(sp);
    }

    /* Total free heap is all memory that could be allocated via
       malloc (assuming fragmentation doesn't become a problem) */
    return sp - brk_val + mi.fordblks;
}

// Get the real flash size
uint32_t GetRealFlashSize(void)
{
    return (1 << ((sdk_spi_flash_get_id() >> 16) & 0xFF));
}

// Send a response.
// 'response' is the number of byes of response if positive, or the error code if negative.
// Use only to respond to commands which don't include a data block, or when we don't want to read the data block.
void SendResponse(int32_t response)
{
	(void)hspi.transfer32(response);
	if (response > 0)
	{
		hspi.transferDwords(transferBuffer, nullptr, NumDwords((size_t)response));
	}
}

// This is called when the SAM is asking to transfer data
void ProcessRequest()
{
	// Set up our own header
	messageHeaderOut.hdr.formatVersion = MyFormatVersion;
	messageHeaderOut.hdr.state = currentState;

	bool deferCommand = false;

	// Begin the transaction
	gpio_write(SamSSPin, false);			// assert CS to SAM
	hspi.beginTransaction();

	// Exchange headers, except for the last dword which will contain our response
	hspi.transferDwords(messageHeaderOut.asDwords, messageHeaderIn.asDwords, headerDwords - 1);
	const size_t dataBufferAvailable = std::min<size_t>(messageHeaderIn.hdr.dataBufferAvailable, MaxDataLength);

	if (messageHeaderIn.hdr.formatVersion != MyFormatVersion)
	{
		SendResponse(ResponseUnknownFormat);
	}
	else if (messageHeaderIn.hdr.dataLength > MaxDataLength)
	{
		SendResponse(ResponseBadDataLength);
	}
	else
	{
		// See what command we have received and take appropriate action
		switch (messageHeaderIn.hdr.command)
		{
		case NetworkCommand::nullCommand:					// no command being sent, SAM just wants the network status
			SendResponse(ResponseEmpty);
			break;

		case NetworkCommand::networkStartClient:			// connect to an access point
		case NetworkCommand::networkStartAccessPoint:		// run as an access point
			if (currentState == WiFiState::idle)
			{
				deferCommand = true;
				messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseEmpty);
			}
			else
			{
				SendResponse(ResponseWrongState);
			}
			break;

		case NetworkCommand::networkFactoryReset:			// clear remembered list, reset factory defaults
			deferCommand = true;
			messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseEmpty);
			break;

		case NetworkCommand::networkStop:					// disconnect from an access point, or close down our own access point
			deferCommand = true;
			messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseEmpty);
			break;

		case NetworkCommand::networkGetStatus:				// get the network connection status
			{
				NetworkStatusResponse * const response = reinterpret_cast<NetworkStatusResponse*>(transferBuffer);
				response->ipAddress = GetIPAddress();
				response->freeHeap = GetFreeHeap();
				response->resetReason = sdk_rtc_get_reset_reason();
				response->flashSize = GetRealFlashSize();
				response->rssi = apRssi;	// sdk_wifi_station_get_rssi() is unsupported by the SDK, use result of the last scan
				response->vcc = 3379;		// cannot change ADC mode in the SDK, report 3.3V
			    sdk_wifi_get_macaddr(STATION_IF, response->macAddress);
				strncpy(response->versionText, firmwareVersion, sizeof(response->versionText));
				strncpy(response->hostName, webHostName, sizeof(response->hostName));
				strncpy(response->ssid, currentSsid, sizeof(response->ssid));
				SendResponse(sizeof(NetworkStatusResponse));
			}
			break;

		case NetworkCommand::networkAddSsid:				// add to our known access point list
		case NetworkCommand::networkConfigureAccessPoint:	// configure our own access point details
			if (messageHeaderIn.hdr.dataLength == sizeof(WirelessConfigurationData))
			{
				messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseEmpty);
				WirelessConfigurationData localClientData;
				hspi.transferDwords(nullptr, transferBuffer, SIZE_IN_DWORDS(WirelessConfigurationData));
				const WirelessConfigurationData * const receivedClientData = reinterpret_cast<const WirelessConfigurationData *>(transferBuffer);
				size_t index;
				bool found;
				if (messageHeaderIn.hdr.command == NetworkCommand::networkConfigureAccessPoint)
				{
					index = 0;
					found = true;
				}
				else
				{
					found = RetrieveSsidData(receivedClientData->ssid, localClientData, &index);
					if (!found)
					{
						found = FindEmptySsidEntry(&index);
					}
				}

				if (found)
				{
					EEPROM.put(index * sizeof(WirelessConfigurationData), *receivedClientData);
					EEPROM.commit();
				}
				else
				{
					lastError = "SSID table full";
				}
			}
			else
			{
				SendResponse(ResponseBadDataLength);
			}
			break;

		case NetworkCommand::networkDeleteSsid:				// delete a network from our access point list
			if (messageHeaderIn.hdr.dataLength == SsidLength)
			{
				messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseEmpty);
				hspi.transferDwords(nullptr, transferBuffer, NumDwords(SsidLength));

				WirelessConfigurationData ssidData;
				size_t index;
				if (RetrieveSsidData(reinterpret_cast<char*>(transferBuffer), ssidData, &index))
				{
					memset(&ssidData, 0xFF, sizeof(ssidData));
					EEPROM.put(index * sizeof(WirelessConfigurationData), ssidData);
					EEPROM.commit();
				}
				else
				{
					lastError = "SSID not found";
				}
			}
			else
			{
				SendResponse(ResponseBadDataLength);
			}
			break;

		case NetworkCommand::networkListSsids:				// list the access points we know about, plus our own access point details
			{
				char *p = reinterpret_cast<char*>(transferBuffer);
				for (size_t i = 0; i <= MaxRememberedNetworks; ++i)
				{
					WirelessConfigurationData tempData;
					EEPROM.get(i * sizeof(WirelessConfigurationData), tempData);
					if (tempData.ssid[0] != 0xFF)
					{
						for (size_t j = 0; j < SsidLength && tempData.ssid[j] != 0; ++j)
						{
							*p++ = tempData.ssid[j];
						}
						*p++ = '\n';
					}
					else if (i == 0)
					{
						// Include an empty entry for our own access point SSID
						*p++ = '\n';
					}
				}
				*p++ = 0;
				const size_t numBytes = p - reinterpret_cast<char*>(transferBuffer);
				if (numBytes <= dataBufferAvailable)
				{
					SendResponse(numBytes);
				}
				else
				{
					SendResponse(ResponseBufferTooSmall);
				}
			}
			break;

		case NetworkCommand::networkSetHostName:			// set the host name
			if (messageHeaderIn.hdr.dataLength == HostNameLength)
			{
				messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseEmpty);
				hspi.transferDwords(nullptr, transferBuffer, NumDwords(HostNameLength));
				memcpy(webHostName, transferBuffer, HostNameLength);
				webHostName[HostNameLength] = 0;			// ensure null terminator

				const bool reconnect = (currentState == WiFiState::connected);
				if (reconnect)
				{
					sdk_wifi_station_disconnect();
				}
				netif_set_hostname(netif_default, webHostName);
				if (reconnect)
				{
					sdk_wifi_station_connect();
				}

				// The following can be called multiple times
				//MDNS.begin(webHostName);
			}
			else
			{
				SendResponse(ResponseBadDataLength);
			}
			break;

		case NetworkCommand::networkGetLastError:
			if (lastError == nullptr)
			{
				SendResponse(0);
			}
			else
			{
				const size_t len = strlen(lastError) + 1;
				if (dataBufferAvailable >= len)
				{
					strcpy(reinterpret_cast<char*>(transferBuffer), lastError);		// copy to 32-bit aligned buffer
					SendResponse(len);
				}
				else
				{
					SendResponse(ResponseBufferTooSmall);
				}
				lastError = nullptr;
			}
			break;

		case NetworkCommand::networkListen:				// listen for incoming connections
			if (messageHeaderIn.hdr.dataLength == sizeof(ListenOrConnectData))
			{
				messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseEmpty);
				ListenOrConnectData lcData;
				hspi.transferDwords(nullptr, reinterpret_cast<uint32_t*>(&lcData), NumDwords(sizeof(lcData)));
				const bool ok = Listener::Listen(lcData.remoteIp, lcData.port, lcData.protocol, lcData.maxConnections);
				if (ok)
				{
					debugPrintf("Listening on port %u\n", lcData.port);
				}
				else
				{
					lastError = "Listen failed";
					debugPrintf("Listen failed\n");
				}
				RebuildServices();
			}
			break;

		case NetworkCommand::connAbort:					// terminate a socket rudely
			if (ValidSocketNumber(messageHeaderIn.hdr.socketNumber))
			{
				messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseEmpty);
				Connection::Get(messageHeaderIn.hdr.socketNumber).Terminate();
			}
			else
			{
				messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseBadParameter);
			}
			break;

		case NetworkCommand::connClose:					// close a socket gracefully
			if (ValidSocketNumber(messageHeaderIn.hdr.socketNumber))
			{
				messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseEmpty);
				Connection::Get(messageHeaderIn.hdr.socketNumber).Close();
			}
			else
			{
				messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseBadParameter);
			}
			break;

		case NetworkCommand::connRead:					// read data from a connection
			if (ValidSocketNumber(messageHeaderIn.hdr.socketNumber))
			{
				Connection& conn = Connection::Get(messageHeaderIn.hdr.socketNumber);
				const size_t amount = conn.Read(reinterpret_cast<uint8_t *>(transferBuffer), std::min<size_t>(messageHeaderIn.hdr.dataBufferAvailable, MaxDataLength));
				messageHeaderIn.hdr.param32 = hspi.transfer32(amount);
				hspi.transferDwords(transferBuffer, nullptr, NumDwords(amount));
			}
			else
			{
				messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseBadParameter);
			}
			break;

		case NetworkCommand::connWrite:					// write data to a connection
			if (ValidSocketNumber(messageHeaderIn.hdr.socketNumber))
			{
				Connection& conn = Connection::Get(messageHeaderIn.hdr.socketNumber);
				const size_t requestedlength = messageHeaderIn.hdr.dataLength;
				const size_t amount = std::min<size_t>(conn.CanWrite(), std::min<size_t>(requestedlength, MaxDataLength));
				const bool closeAfterSending = amount == requestedlength && (messageHeaderIn.hdr.flags & MessageHeaderSamToEsp::FlagCloseAfterWrite) != 0;
				const bool push = amount == requestedlength && (messageHeaderIn.hdr.flags & MessageHeaderSamToEsp::FlagPush) != 0;
				messageHeaderIn.hdr.param32 = hspi.transfer32(amount);
				hspi.transferDwords(nullptr, transferBuffer, NumDwords(amount));
				const size_t written = conn.Write(reinterpret_cast<uint8_t *>(transferBuffer), amount, push, closeAfterSending);
				if (written != amount)
				{
					lastError = "incomplete write";
				}
			}
			else
			{
				messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseBadParameter);
			}
			break;

		case NetworkCommand::connGetStatus:				// get the status of a socket, and summary status for all sockets
			if (ValidSocketNumber(messageHeaderIn.hdr.socketNumber))
			{
				messageHeaderIn.hdr.param32 = hspi.transfer32(sizeof(ConnStatusResponse));
				Connection& conn = Connection::Get(messageHeaderIn.hdr.socketNumber);
				ConnStatusResponse resp;
				conn.GetStatus(resp);
				Connection::GetSummarySocketStatus(resp.connectedSockets, resp.otherEndClosedSockets);
				hspi.transferDwords(reinterpret_cast<const uint32_t *>(&resp), nullptr, sizeof(resp));
			}
			else
			{
				messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseBadParameter);
			}
			break;

		case NetworkCommand::diagnostics:				// print some debug info over the UART line
			stats_display();
			SendResponse(ResponseEmpty);
			break;

		case NetworkCommand::connCreate:				// create a connection
		default:
			SendResponse(ResponseUnknownCommand);
			break;
		}
	}

	gpio_write(SamSSPin, true);   						// de-assert CS to SAM to end the transaction and tell SAM the transfer is complete

	// If we deferred the command until after sending the response (e.g. because it may take some time to execute), complete it now
	if (deferCommand)
	{
		// The following functions must set up lastError if an error occurs.
		lastError = nullptr;								// assume no error
		switch (messageHeaderIn.hdr.command)
		{
		case NetworkCommand::networkStartClient:			// connect to an access point
			StartClient();
			break;

		case NetworkCommand::networkStartAccessPoint:		// run as an access point
			StartAccessPoint();
			break;

		case NetworkCommand::networkStop:					// disconnect from an access point, or close down our own access point
			Connection::TerminateAll();						// terminate all connections
			Listener::StopListening(0);						// stop listening on all ports

			switch (currentState)
			{
			case WiFiState::connected:
			case WiFiState::connecting:
				sdk_wifi_station_disconnect();
				break;

			case WiFiState::runningAsAccessPoint:
				dhcpserver_stop();
				//dns.stop();
				break;

			default:
				break;
			}
			currentState = WiFiState::idle;

			vTaskDelay(100 / portTICK_PERIOD_MS);
			sdk_wifi_set_opmode(NULL_MODE);
			break;

		case NetworkCommand::networkFactoryReset:			// clear remembered list, reset factory defaults
			FactoryReset();
			break;

		default:
			lastError = "bad deferred command";
			break;
		}

		if (lastError == nullptr)
		{
			gpio_write(EspReqTransferPin, false);			// force a status update if no error occurred
			vTaskDelay(2 / portTICK_PERIOD_MS);
		}
	}
}

void xSpiTransmission(void *pvParameters)
{
	for(;;)
	{
		gpio_write(EspReqTransferPin, true);				// tell the SAM we are ready to receive a command

		// See whether there is a request from the SAM
		if (gpio_read(SamTfrReadyPin))
		{
			ProcessRequest();
			if (lastError == nullptr)
			{
				prevLastError = nullptr;
			}
			else
			{
				if (lastError != prevLastError)
				{
					prevLastError = lastError;
					debugPrintf("Signalling error: %s\n", lastError);
				}
				gpio_write(EspReqTransferPin, false);
				vTaskDelay(2 / portTICK_PERIOD_MS);			// force a low to high transition to signal that an error message is available
			}
		}

		// TODO: Move these to separate tasks
		ConnectPoll();
		Connection::PollOne();
		Connection::ReportConnections();

		// Keep the WiFi subsystem running
		portYIELD();
	}
}

extern "C" void user_init()
{
	// Initialise the UART connection
    uart_set_baud(0, 115200);
    debugPrintf("Duet WiFi Server starting...\n");
    debugPrintf("SDK version: %s\n", sdk_system_get_sdk_version());

    // Initialise the WiFi subsystem
    sdk_wifi_set_opmode(NULL_MODE);
    sdk_wifi_station_set_auto_connect(0);
    //netbios_init();
    //mdns_init();

	// Reserve some flash space for use as EEPROM. The maximum EEPROM supported by the core is SPI_FLASH_SEC_SIZE (4Kb).
	const size_t eepromSizeNeeded = (MaxRememberedNetworks + 1) * sizeof(WirelessConfigurationData);
	static_assert(eepromSizeNeeded <= SPI_FLASH_SEC_SIZE, "Insufficient EEPROM");
	EEPROM.begin(eepromSizeNeeded);

	// Set up the SPI subsystem
    gpio_enable(SamTfrReadyPin, GPIO_INPUT);
    gpio_enable(EspReqTransferPin, GPIO_OUTPUT);
    gpio_write(EspReqTransferPin, false);				// not ready to transfer data yet
    gpio_enable(SamSSPin, GPIO_OUTPUT);
    gpio_write(SamSSPin, true);

    // Set up the fast SPI channel
    hspi.begin();
    hspi.setBitOrder(SPI_BIG_ENDIAN);
    hspi.setDataMode(SPI_MODE1);
    hspi.setFrequency(spiFrequency);

    Connection::Init();
    Listener::Init();
    lastError = nullptr;
    debugPrintf("Init complete\n");

    // TODO: This isn't ideal yet, create tasks for each connection and let them run concurrently
    connectSemaphore = xSemaphoreCreateBinary();
    xTaskCreate(xSpiTransmission, "SpiTransmission", 2000, nullptr, 2, nullptr);
    //sdk_wifi_status_led_install(16, PERIPHS_IO_MUX_GPIO16_U, FUNC_GPIO16);
    //xTaskCreate(xLed, "Led", 200, nullptr, 1, nullptr);

    //xPortStartScheduler();							// don't do this on the ESP platform!
}

// End
