/*
 * SocketServer.cpp
 *
 *  Created on: 25 Mar 2017
 *      Author: David
 */

#include "ecv.h"
#undef yield
#undef array

#include <cstdarg>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <EEPROM.h>
#include "SocketServer.h"
#include "Config.h"
#include "PooledStrings.h"
#include "HSPI.h"

#include "include/MessageFormats.h"
#include "Connection.h"
#include "Listener.h"
#include "Misc.h"

extern "C"
{
	#include "user_interface.h"     // for struct rst_info
	#include "lwip/stats.h"			// for stats_display()
	#include "lwip/app/netbios.h"	// for NetBIOS support
}

#define array _ecv_array

const uint32_t MaxConnectTime = 40 * 1000;		// how long we wait for WiFi to connect in milliseconds
const uint32_t StatusReportMillis = 200;

const int DefaultWiFiChannel = 6;

// Global data
char currentSsid[SsidLength + 1];
char webHostName[HostNameLength + 1] = "Duet-WiFi";

DNSServer dns;

static const char* lastError = nullptr;
static const char* prevLastError = nullptr;
static bool connectErrorChanged = false;

static char lastConnectError[100];

static WiFiState currentState = WiFiState::idle,
				prevCurrentState = WiFiState::disabled,
				lastReportedState = WiFiState::disabled;

ADC_MODE(ADC_VCC);          // need this for the ESP.getVcc() call to work

static HSPIClass hspi;
static uint32_t connectStartTime;
static uint32_t lastStatusReportTime;
static uint32_t transferBuffer[NumDwords(MaxDataLength + 1)];

static const WirelessConfigurationData *ssidData = nullptr;

// Look up a SSID in our remembered network list, return pointer to it if found
const WirelessConfigurationData *RetrieveSsidData(const char *ssid, int *index = nullptr)
{
	for (size_t i = 1; i <= MaxRememberedNetworks; ++i)
	{
		const WirelessConfigurationData *wp = EEPROM.getPtr<WirelessConfigurationData>(i * sizeof(WirelessConfigurationData));
		if (wp != nullptr && strncmp(ssid, wp->ssid, sizeof(wp->ssid)) == 0)
		{
			if (index != nullptr)
			{
				*index = i;
			}
			return wp;
		}
	}
	return nullptr;
}

// Find an empty entry in the table of known networks
bool FindEmptySsidEntry(int *index)
{
	for (size_t i = 1; i <= MaxRememberedNetworks; ++i)
	{
		const WirelessConfigurationData *wp = EEPROM.getPtr<WirelessConfigurationData>(i * sizeof(WirelessConfigurationData));
		if (wp != nullptr && wp->ssid[0] == 0xFF)
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

// Try to connect using the specified SSID and password
void ConnectToAccessPoint(const WirelessConfigurationData& apData, bool isRetry)
pre(currentState == NetworkState::idle)
{
	SafeStrncpy(currentSsid, apData.ssid, ARRAY_SIZE(currentSsid));

	WiFi.mode(WIFI_STA);
	wifi_station_set_hostname(webHostName);     				// must do this before calling WiFi.begin()
	WiFi.setAutoConnect(false);
	WiFi.setAutoReconnect(true);
	WiFi.config(IPAddress(apData.ip), IPAddress(apData.gateway), IPAddress(apData.netmask), IPAddress(), IPAddress());
	WiFi.begin(apData.ssid, apData.password);

	if (isRetry)
	{
		currentState = WiFiState::reconnecting;
	}
	else
	{
		currentState = WiFiState::connecting;
		connectStartTime = millis();
	}
}

void ConnectPoll()
{
	// The Arduino WiFi.status() call is fairly useless here because it discards too much information, so use the SDK API call instead
	const station_status_t status = wifi_station_get_connect_status();
	const char *error = nullptr;
	bool retry = false;

	switch (currentState)
	{
	case WiFiState::connecting:
	case WiFiState::reconnecting:
		// We are trying to connect or reconnect, so check for success or failure
		switch (status)
		{
		case STATION_IDLE:
			error = "Unexpected WiFi state 'idle'";
			break;

		case STATION_CONNECTING:
			if (millis() - connectStartTime >= MaxConnectTime)
			{
				error = "Timed out";
			}
			break;

		case STATION_WRONG_PASSWORD:
			error = "Wrong password";
			break;

		case STATION_NO_AP_FOUND:
			error = "Didn't find access point";
			retry = (currentState == WiFiState::reconnecting);
			break;

		case STATION_CONNECT_FAIL:
			error = "Failed";
			retry = (currentState == WiFiState::reconnecting);
			break;

		case STATION_GOT_IP:
			if (currentState == WiFiState::reconnecting)
			{
				lastError = "Reconnect succeeded";
			}
			currentState = WiFiState::connected;
			debugPrintln("Connected to AP");
			break;

		default:
			error = "Unknown WiFi state";
			break;
		}

		if (error != nullptr)
		{
			strcpy(lastConnectError, error);
			SafeStrncat(lastConnectError, " while trying to connect to ", ARRAY_SIZE(lastConnectError));
			SafeStrncat(lastConnectError, currentSsid, ARRAY_SIZE(lastConnectError));
			lastError = lastConnectError;
			connectErrorChanged = true;
			debugPrintln("failed to connect to AP");

			if (!retry)
			{
				WiFi.mode(WIFI_OFF);
				currentState = WiFiState::idle;
			}
		}
		break;

	case WiFiState::connected:
		if (status != STATION_GOT_IP)
		{
			// We have just lost the connection
			connectStartTime = millis();						// start the auto reconnect timer

			switch (status)
			{
			case STATION_CONNECTING:							// auto reconnecting
				error = "auto reconnecting";
				currentState = WiFiState::autoReconnecting;
				break;

			case STATION_IDLE:
				error = "state 'idle'";
				retry = true;
				break;

			case STATION_WRONG_PASSWORD:
				error = "state 'wrong password'";
				currentState = WiFiState::idle;
				break;

			case STATION_NO_AP_FOUND:
				error = "state 'no AP found'";
				retry = true;
				break;

			case STATION_CONNECT_FAIL:
				error = "state 'fail'";
				retry = true;
				break;

			default:
				error = "unknown WiFi state";
				currentState = WiFiState::idle;
				break;
			}

			strcpy(lastConnectError, "Lost connection, ");
			SafeStrncat(lastConnectError, error, ARRAY_SIZE(lastConnectError));
			lastError = lastConnectError;
			connectErrorChanged = true;
			debugPrintln("Lost connection to AP");
			break;
		}
		break;

	case WiFiState::autoReconnecting:
		if (status == STATION_GOT_IP)
		{
			lastError = "Auto reconnect succeeded";
			currentState = WiFiState::connected;
		}
		else if (status != STATION_CONNECTING && lastError == nullptr)
		{
			lastError = "Auto reconnect failed, trying manual reconnect";
			connectStartTime = millis();						// start the manual reconnect timer
			retry = true;
		}
		else if (millis() - connectStartTime >= MaxConnectTime)
		{
			lastError = "Timed out trying to auto-reconnect";
			retry = true;
		}
		break;

	default:
		break;
	}

	if (retry)
	{
		ConnectToAccessPoint(*ssidData, true);
	}
}

void StartClient(const char * array ssid)
pre(currentState == WiFiState::idle)
{
	ssidData = nullptr;

	if (ssid == nullptr || ssid[0] == 0)
	{
		// Auto scan for strongest known network, then try to connect to it
		const int8_t num_ssids = WiFi.scanNetworks(false, true);
		if (num_ssids < 0)
		{
			lastError = "network scan failed";
			currentState = WiFiState::idle;
			return;
		}

		// Find the strongest network that we know about
		int8_t strongestNetwork = -1;
		for (int8_t i = 0; i < num_ssids; ++i)
		{
			if (strongestNetwork < 0 || WiFi.RSSI(i) > WiFi.RSSI(strongestNetwork))
			{
				const WirelessConfigurationData *wp = RetrieveSsidData(WiFi.SSID(i).c_str(), nullptr);
				if (wp != nullptr)
				{
					strongestNetwork = i;
					ssidData = wp;
				}
			}
		}
		if (strongestNetwork < 0)
		{
			lastError = "no known networks found";
			return;
		}
	}
	else
	{
		ssidData = RetrieveSsidData(ssid, nullptr);
		if (ssidData == nullptr)
		{
			lastError = "no data found for requested SSID";
			return;
		}
	}

	// ssidData contains the details of the strongest known access point
	ConnectToAccessPoint(*ssidData, false);
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
		SafeStrncpy(currentSsid, apData.ssid, ARRAY_SIZE(currentSsid));
		bool ok = WiFi.mode(WIFI_AP);
		if (ok)
		{
			IPAddress apIP(apData.ip);
			ok = WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
		}
		if (ok)
		{
			ok = WiFi.softAP(currentSsid, apData.password, (apData.channel == 0) ? DefaultWiFiChannel : apData.channel);
		}
		if (ok)
		{
			dns.setErrorReplyCode(DNSReplyCode::NoError);
			if (!dns.start(53, "*", apData.ip))
			{
				lastError = "Failed to start DNS";
			}
			debugPrintln("AP started");
			SafeStrncpy(currentSsid, apData.ssid, ARRAY_SIZE(currentSsid));
			delay(100);		// trying this to see if it helps
			currentState = WiFiState::runningAsAccessPoint;
		}
		else
		{
			lastError = "Failed to start access point";
			currentState = WiFiState::idle;
		}
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

// Rebuild the MDNS server to advertise a single service
void AdvertiseService(int service, uint16_t port)
{
	static int currentService = -1;
	static const char * const serviceNames[] = { "http", "tcp", "ftp" };

	if (service != currentService)
	{
		currentService = service;
		MDNS.deleteServices();
		if (service >= 0 && service < (int)ARRAY_SIZE(serviceNames))
		{
			const char* serviceName = serviceNames[service];
			MDNS.addService(serviceName, "tcp", port);
			MDNS.addServiceTxt(serviceName, "tcp", "product", "DuetWiFi");
			MDNS.addServiceTxt(serviceName, "tcp", "version", firmwareVersion);
		}
	}
}

// Rebuild the mDNS services
void RebuildServices()
{
	if (currentState == WiFiState::connected)		// MDNS server only works in station mode
	{
		// Unfortunately the official ESP8266 mDNS library only reports one service.
		// I (chrishamm) tried to use the old mDNS responder, which is also capable of sending
		// mDNS broadcasts, but the packets it generates are broken and thus not of use.
		for (int service = 0; service < 3; ++service)
		{
			const uint16_t port = Listener::GetPortByProtocol(service);
			if (port != 0)
			{
				AdvertiseService(service, port);
				return;
			}
		}

		AdvertiseService(-1, 0);		// no services to advertise
	}
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
	digitalWrite(SamSSPin, LOW);            // assert CS to SAM
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
			if (currentState == WiFiState::idle)
			{
				deferCommand = true;
				messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseEmpty);
				if (messageHeaderIn.hdr.dataLength != 0 && messageHeaderIn.hdr.dataLength <= SsidLength + 1)
				{
					hspi.transferDwords(nullptr, transferBuffer, NumDwords(messageHeaderIn.hdr.dataLength));
					reinterpret_cast<char *>(transferBuffer)[messageHeaderIn.hdr.dataLength] = 0;
				}
			}
			else
			{
				SendResponse(ResponseWrongState);
			}
			break;

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
				const bool runningAsAp = (currentState == WiFiState::runningAsAccessPoint);
				const bool runningAsStation = (currentState == WiFiState::connected);
				NetworkStatusResponse * const response = reinterpret_cast<NetworkStatusResponse*>(transferBuffer);
				response->ipAddress = (runningAsAp)
										? static_cast<uint32_t>(WiFi.softAPIP())
										: (runningAsStation)
										  ? static_cast<uint32_t>(WiFi.localIP())
											  : 0;
				response->freeHeap = ESP.getFreeHeap();
				response->resetReason = ESP.getResetInfoPtr()->reason;
				response->flashSize = ESP.getFlashChipRealSize();
				response->rssi = (runningAsStation) ? wifi_station_get_rssi() : 0;
				response->numClients = (runningAsAp) ? wifi_softap_get_station_num() : 0;
				response->spare = 0;
				response->vcc = ESP.getVcc();
			    wifi_get_macaddr((runningAsAp) ? SOFTAP_IF : STATION_IF, response->macAddress);
			    SafeStrncpy(response->versionText, firmwareVersion, sizeof(response->versionText));
			    SafeStrncpy(response->hostName, webHostName, sizeof(response->hostName));
			    SafeStrncpy(response->ssid, currentSsid, sizeof(response->ssid));
				SendResponse(sizeof(NetworkStatusResponse));
			}
			break;

		case NetworkCommand::networkAddSsid:				// add to our known access point list
		case NetworkCommand::networkConfigureAccessPoint:	// configure our own access point details
			if (messageHeaderIn.hdr.dataLength == sizeof(WirelessConfigurationData))
			{
				messageHeaderIn.hdr.param32 = hspi.transfer32(ResponseEmpty);
				hspi.transferDwords(nullptr, transferBuffer, SIZE_IN_DWORDS(WirelessConfigurationData));
				const WirelessConfigurationData * const receivedClientData = reinterpret_cast<const WirelessConfigurationData *>(transferBuffer);
				int index;
				if (messageHeaderIn.hdr.command == NetworkCommand::networkConfigureAccessPoint)
				{
					index = 0;
				}
				else
				{
					index = -1;
					(void)RetrieveSsidData(receivedClientData->ssid, &index);
					if (index < 0)
					{
						(void)FindEmptySsidEntry(&index);
					}
				}

				if (index >= 0)
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

				int index;
				if (RetrieveSsidData(reinterpret_cast<char*>(transferBuffer), &index) != nullptr)
				{
					WirelessConfigurationData ssidData;
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

				// The following can be called multiple times
				MDNS.begin(webHostName);
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
			lastReportedState = currentState;
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
					debugPrint("Listening on port ");
					debugPrintln(lcData.port);
				}
				else
				{
					lastError = "Listen failed";
					debugPrintln("Listen failed");
				}
//				RebuildServices();
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
			debugPrint("SDK Version: ");
			debugPrintln(system_get_sdk_version());
			SendResponse(ResponseEmpty);
			break;

		case NetworkCommand::connCreate:				// create a connection
		default:
			SendResponse(ResponseUnknownCommand);
			break;
		}
	}

	digitalWrite(SamSSPin, HIGH);     						// de-assert CS to SAM to end the transaction and tell SAM the transfer is complete
	hspi.endTransaction();

	// If we deferred the command until after sending the response (e.g. because it may take some time to execute), complete it now
	if (deferCommand)
	{
		// The following functions must set up lastError if an error occurs
		lastError = nullptr;								// assume no error
		switch (messageHeaderIn.hdr.command)
		{
		case NetworkCommand::networkStartClient:			// connect to an access point
			if (messageHeaderIn.hdr.dataLength == 0 || reinterpret_cast<const char*>(transferBuffer)[0] == 0)
			{
				StartClient(nullptr);						// connect to strongest known access point
			}
			else
			{
				StartClient(reinterpret_cast<const char*>(transferBuffer));		// connect to specified access point
			}
			break;

		case NetworkCommand::networkStartAccessPoint:		// run as an access point
			StartAccessPoint();
			break;

		case NetworkCommand::networkStop:					// disconnect from an access point, or close down our own access point
			Connection::TerminateAll();						// terminate all connections
			Listener::StopListening(0);						// stop listening on all ports
			RebuildServices();								// stop the MDNS server (this will be the effect after terminating all listeners)
			switch (currentState)
			{
			case WiFiState::connected:
			case WiFiState::connecting:
				WiFi.disconnect(true);
				break;

			case WiFiState::runningAsAccessPoint:
				WiFi.softAPdisconnect(true);
				break;

			default:
				break;
			}
			delay(100);
			currentState = WiFiState::idle;
			break;

		case NetworkCommand::networkFactoryReset:			// clear remembered list, reset factory defaults
			FactoryReset();
			break;

		default:
			lastError = "bad deferred command";
			break;
		}
	}
}

void setup()
{
	// Enable serial port for debugging
	Serial.begin(115200);
#ifdef DEBUG
	Serial.setDebugOutput(true);
#endif

	WiFi.mode(WIFI_OFF);
	WiFi.persistent(false);

	// Reserve some flash space for use as EEPROM. The maximum EEPROM supported by the core is API_FLASH_SEC_SIZE (4Kb).
	const size_t eepromSizeNeeded = (MaxRememberedNetworks + 1) * sizeof(WirelessConfigurationData);
	static_assert(eepromSizeNeeded <= SPI_FLASH_SEC_SIZE, "Insufficient EEPROM");
	EEPROM.begin(eepromSizeNeeded);

	// Set up the SPI subsystem
    pinMode(SamTfrReadyPin, INPUT);
    pinMode(EspReqTransferPin, OUTPUT);
    digitalWrite(EspReqTransferPin, LOW);				// not ready to transfer data yet
    pinMode(SamSSPin, OUTPUT);
    digitalWrite(SamSSPin, HIGH);

    // Set up the fast SPI channel
    hspi.begin();
    hspi.setBitOrder(MSBFIRST);
    hspi.setDataMode(SPI_MODE1);
    hspi.setFrequency(spiFrequency);

    Connection::Init();
    Listener::Init();
    netbios_init();
    lastError = nullptr;
    debugPrintln("Init completed");
	digitalWrite(EspReqTransferPin, HIGH);				// tell the SAM we are ready to receive a command
	lastStatusReportTime = millis();
}

void loop()
{
	digitalWrite(EspReqTransferPin, HIGH);				// tell the SAM we are ready to receive a command
	if (   (lastError != prevLastError || connectErrorChanged || currentState != prevCurrentState)
		|| ((lastError != nullptr || currentState != lastReportedState) && millis() - lastStatusReportTime > StatusReportMillis)
	   )
	{
		delayMicroseconds(2);							// make sure the pin stays high for long enough for the SAM to see it
		digitalWrite(EspReqTransferPin, LOW);			// force a low to high transition to signal that an error message is available
		delayMicroseconds(2);							// make sure it is low enough to create an interrupt when it goes high
		digitalWrite(EspReqTransferPin, HIGH);			// tell the SAM we are ready to receive a command
		prevLastError = lastError;
		prevCurrentState = currentState;
		connectErrorChanged = false;
		lastStatusReportTime = millis();
	}

	// See whether there is a request from the SAM
	if (digitalRead(SamTfrReadyPin) == HIGH)
	{
		ProcessRequest();
	}

	ConnectPoll();
	Connection::PollOne();
	Connection::ReportConnections();

	// Let the WiFi subsystem get on with its stuff
	//yield();
}

// End
