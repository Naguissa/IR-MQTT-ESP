/**
 * MQTT messages:
 *   C-hexadecimal(<ClientName> + "||" + <client_local_ip>) - ESP Connected to MQTT
 *   R-hexadecimal(<ClientName> + "||" + <2 bytes size><rawddata>) - ESP received IR raw sensor data. <rawdata> is the raw dump as a continous uint16_t hex encoded values (4 characters for each uint16_t value)
 *   S-hexadecimal(<ClientName> + "||" + <2 bytes protocol><2 bytes address><2 bytes command><2 bytes numberOfBits><1 byte flags>) - ESP received IR coded sensor data.
 *
 * Required libraries:
 *  - uMessagesBrokerLib - https://github.com/Naguissa/uMessagesBrokerLib
 *  - uHexLib - https://github.com/Naguissa/uHexLib - Installed as dependency of uMessagesBrokerLib
 *  - IRremote - https://github.com/Arduino-IRremote/Arduino-IRremote
 *
 *
 *
 *
 * @version 2.0.0
 */

/*
V1 messages, not compatible:
<ClientName>-C-<localIP>
<ClientName>-R-<2 bytes signal length><rawdata_in_hexadecimal>
*/
#include <Arduino.h>

#define MQTT_MAX_PACKET_SIZE 1600
#define RAW_BUFFER_LENGTH 1000
#define MARK_EXCESS_MICROS    40    // Adapt it to your IR receiver module. 40 is recommended for the cheap VS1838 modules at high intensity.


#include "libraries/PubSubClient/src/PubSubClient.h"
#include "libraries/PubSubClient/src/PubSubClient.cpp"
#include "uEspConfigLib.h"
#include "uHexLib.h"
#include "uMessagesBrokerLib.h"

#include <IRremote.hpp>




// ==================== start of TUNEABLE PARAMETERS (can be changed using web setup) ====================

// Uncomment these lines in order to activate Serial debug; unable if you are using ESP-01
#define DEBUG 1
#define UDEBUGLIB_BAUDS 115200



// ==================== end of TUNEABLE PARAMETERS ====================

char *fullTopic;
uEspConfigLibFSInterface * configFs;
uEspConfigLib * config;
char * clientName, * ApiKey, * email, *mqtt_server, *topic;
int mqtt_port;
IRData IRReadResults; // Somewhere to store the results
volatile uint8_t doReset = 0;

char currentWifiMode = '-';
byte clientNameLen = 0;
#define MAX_MQTT_MSG_LEN 2500
#define NO_LED_FEEDBACK_CODE true

char mqttData[MAX_MQTT_MSG_LEN];

unsigned long int last_mqtt_connect = 0;

IRData IRDataToEmit;

// =======================================================
// ==================== FUNCTIONALITY ====================
// =======================================================


// ==================== Function definitions ====================
void handleDefault();
void handleConfig();
void handleSaveConfig();


void setup_web(void);
void setup_wifi();
void setup_mqtt();
void setup_ir();
void setup_messageBroker();
void setup_config();
void setup();

void callbackConnect(const char);
void callbackRecieveRaw(const char);
/** In development
void callbackRecieveCoded(const char);
*/

void mqtt_reconnect();
void receiveMQTTMessage(char*, byte*, unsigned int);
bool checkSameClient(char *);
uint16_t separatorPositionPlusOne(const char []);


void sendMQTTMessageConnect();
void sendMQTTMessageIRRaw();
/** In development
void sendMQTTMessageIRStructured();
*/
void loop();


// ==================== Debug functionality; simplified, taken from: https://github.com/Naguissa/uDebugLib ====================
#ifdef DEBUG
  void uDebugLibInitFunction()
  {
    Serial.begin(UDEBUGLIB_BAUDS);
    while (!Serial) ; // wait for serial port to connect. Needed for native USB
  }
  #define uDebugLibInit() uDebugLibInitFunction()
  #define DEBUG_PRINT Serial.print
  #define DEBUG_PRINTLN Serial.println
#else
  #define uDebugLibInit()
  #define DEBUG_PRINT
  #define DEBUG_PRINTLN
#endif

// ==================== ESP web, mqtt and update ====================
#ifdef ARDUINO_ARCH_ESP32
    #include <HTTPUpdateServer.h>
    WebServer server(80);
    HTTPUpdateServer httpUpdater;
#else
    #include <ESP8266HTTPUpdateServer.h>
    ESP8266WebServer server(80);
    ESP8266HTTPUpdateServer httpUpdater;
#endif


WiFiClient espClient;
PubSubClient mqttClient(espClient);

void handleDefault() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");
    yield();
    server.sendContent("<html><head><title>Home - IR-MQTT-ESP</title></head><body><p>IR-MQTT-ESP, IR multiple repeater using MQTT for ESP8266 and ESP32 microcontrollers.</p>");
    if (doReset == 1 && server.hasArg("saved") && server.arg("saved") == "1") {
        server.sendContent("<p><b>Configuration saved, resetting device</b></p>");
        doReset = 2;    
    }
    server.sendContent("<p>Available items:</p>");
    yield();
    server.sendContent("<p><a href=\"/config\">Configuration</a></p>");
    yield();
    server.sendContent("<p><a href=\"/update\">Update</a></p>");
    yield();
    server.sendContent("<p>Check GitHub page for <a href=\"https://github.com/Naguissa/IR-MQTT-ESP\">info</a>, <a href=\"https://github.com/Naguissa/IR-MQTT-ESP/issues?q=is%3Aissue+\">support</a> or <a href=\"https://github.com/Naguissa/IR-MQTT-ESP/releases\">updates</a>.</p></body></html>");
    yield();
}

void handleConfig() {
  config->handleConfigRequestHtml(&server, "/config");
}

void handleSaveConfig() {
    config->handleSaveConfig(&server);
    doReset = 1;
}


// ==================== Setup functions ====================

void setup_web() {
    httpUpdater.setup(&server, "/update");
    server.on("/config", HTTP_POST, handleSaveConfig);
    server.on("/config", HTTP_GET, handleConfig);
    server.onNotFound(handleDefault);
    yield();
    server.begin();
    yield();
}

void setup_config() {
    configFs = new uEspConfigLibFSLittlefs("/ir-mqtt.ini", true);
    if (configFs->status() == uEspConfigLibFS_STATUS_FATAL) {
        DEBUG_PRINTLN("  * Error initializing LittleFS");
    }
    config = new uEspConfigLib(configFs);

    config->addOption("wifi_mode", "WiFi mode (C=Client, other=Access Point)", "");
    config->addOption("wifi_ssid", "WiFi SSID", "IR-MQTT-ESP");
    config->addOption("wifi_password", "WiFi password", "");

    config->addOption("mqtt_server", "Mqtt server address", "foroelectro.net");
    config->addOption("mqtt_port", "Mqtt server port", "1883");
    config->addOption("clientName", "Mqtt client name", "IrMqttEsp");
    config->addOption("ApiKey", "ApiKey from https://www.foroelectro.net/arduino/en/mqtt-doc", "Get one for free on https://www.foroelectro.net/arduino/en/mqtt-doc");
    config->addOption("email", "Email corresponding to your ApiKey", "Get one for free on https://www.foroelectro.net/arduino/en/mqtt-doc");
    config->addOption("topic", "Subtopic where messages are directed; can be empty if you want", "");

    config->addOption("kRecvPin", "Pin number where IR demodulator is connected. Default: 14 (D5 on a NodeMCU board)", "14");
    config->addOption("kIrLed", "Pin number where base of NPN transistor to IR emitter is connected. Default: 4 (D2 on a NodeMCU board)", "4");
    config->addOption("kMinLen", "Signal minimal length. Default: 10", "10");


    config->loadConfigFile();
    // Set global pointers
    clientName = config->getPointer("clientName");
    ApiKey = config->getPointer("ApiKey");
    email = config->getPointer("email");
    mqtt_server = config->getPointer("mqtt_server");
    char *mqtt_port_str = config->getPointer("mqtt_port");
    mqtt_port = atoi(mqtt_port_str);
    topic = config->getPointer("topic");
}


void setup_wifi() {
    char *mode, *ssid, *pass;
    
	server.stop();
	WiFi.disconnect();
    #ifndef ARDUINO_ARCH_ESP32
	    WiFi.setAutoConnect(true);
	#endif
	WiFi.setAutoReconnect(true);
	
    mode = config->getPointer("wifi_mode");
    ssid = config->getPointer("wifi_ssid");
    pass = config->getPointer("wifi_password");

    DEBUG_PRINT("Connecting to WiFi in '");
    DEBUG_PRINT(mode);
    DEBUG_PRINTLN("' mode.");
    DEBUG_PRINT("  - ssid: ");
    DEBUG_PRINTLN(ssid);
    DEBUG_PRINT("  - pass: ");
    DEBUG_PRINTLN(pass);

    if (strcmp(mode, "C") == 0) {
		WiFi.mode(WIFI_STA);
		WiFi.begin(ssid, pass);

		// Wait for connection
		uint8_t i = 0;
		while (WiFi.status() != WL_CONNECTED && i++ < 30) { //wait 30*2 seconds
			Serial.print('.');
			delay(2000);
		}
		if (i == 31) {
	        DEBUG_PRINT("Could not connect to ");
			DEBUG_PRINTLN(ssid);
            return;
		}
        currentWifiMode = 'C';
        DEBUG_PRINT("Connected! IP address: ");
		DEBUG_PRINTLN(WiFi.localIP());
	} else { // Default mode, 'A' (AP)
        WiFi.mode(WIFI_AP);
        WiFi.softAP(ssid, pass);
        currentWifiMode = 'A';
        DEBUG_PRINT("SoftAP created! IP address: ");
        DEBUG_PRINTLN(WiFi.softAPIP());
	}
}

void setup_messageBroker() {
    uMessagesBrokerLib::set('C', callbackConnect);
    uMessagesBrokerLib::set('R', callbackRecieveRaw);
/** In development
    uMessagesBrokerLib::set('S', callbackRecieveCoded);
*/
}

// This section of code runs only once at start-up.
void setup_ir() {
    IrSender.begin(atoi(config->getPointer("kIrLed"))); // Start emiter
    IrReceiver.begin(atoi(config->getPointer("kRecvPin")), false);
    IrReceiver.enableIRIn(); // Start the receiver
}

void setup_mqtt() {
    if (mqtt_port <= 0) {
        mqtt_port = 1883;
    }

	String stopic = ApiKey;

    if (topic[0] != 0) {
        stopic.concat("/");
        stopic.concat(topic);
    }
    if (fullTopic != NULL && fullTopic && strlen(fullTopic)) {
        free(fullTopic);
    }
    fullTopic = (char *) malloc(stopic.length() + 1);
    stopic.toCharArray(fullTopic, stopic.length() + 1);

    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(receiveMQTTMessage);
}



void receiveMQTTMessage(char *topicR, byte *payload, unsigned int length) {
	DEBUG_PRINT("MQTT message arrived [");
	DEBUG_PRINT(topicR);
	DEBUG_PRINTLN("]:");
	DEBUG_PRINTLN((char *) payload);
    uMessagesBrokerLib::process((const char *) payload);
}


bool checkSameClient(const char payload[]) {
    bool sameClient = true;
    uint16_t i = 0, payloadLen = strlen(payload);

    // Always different client name:
    //  - Payload shorter than name length + 2
    //  - Next chars to name are not "||"
    if (
        payloadLen < clientNameLen + 2
        || payload[clientNameLen + 1] != '|'
        || payload[clientNameLen + 2] != '|'
    ) {
        DEBUG_PRINTLN("Same client? No");
        return false;
    }

    yield();
    // Client name
    for (; sameClient && i < clientNameLen; i++) {
        DEBUG_PRINT((char) payload[i]);
        if (clientName[i] != payload[i]) {
            DEBUG_PRINTLN("Same client? No");
            return false;
        }
        yield();
    }

    DEBUG_PRINTLN("Same client? Yes");
    return true;
}

// If return is zero separator is not found
uint16_t separatorPositionPlusOne(const char message[]) {
    uint16_t i, messageLen = strlen(message);
    for(i = 1; i < messageLen; i++) {
        if (message[i-1] == '|' && message[i] == '|') {
            return i;
        }
    }
    return 0;
}


void callbackConnect(const char message[]) {
    uint16_t separatorPos = separatorPositionPlusOne(message);
    if (separatorPos == 0) {
        DEBUG_PRINT("ERROR. Separator not found in message: ");
        DEBUG_PRINTLN(message);
        yield();
        return;
    }
    checkSameClient(message);
    DEBUG_PRINT("New device connected with IP: ");
    DEBUG_PRINTLN((char*) &message[separatorPos+1]);
}


bool checkCallbackRecieveCommon(const char message[], const uint16_t separatorPos, const uint16_t messageLength) {
    if (separatorPos == 0) {
        DEBUG_PRINT("ERROR. callbackRecieve. Separator not found in message: ");
        DEBUG_PRINTLN(message);
        yield();
        return false;
    }

    if (messageLength < separatorPos + 2) {
        DEBUG_PRINT("ERROR. callbackRecieve. Message too short after separator ");
        DEBUG_PRINTLN(message);
        yield();
        return false;
    }

    if (messageLength == separatorPos + 2) {
        DEBUG_PRINT("ERROR. callbackRecieve. Empty IR message");
        DEBUG_PRINTLN(message);
        yield();
        return false;
    }

    bool sameClient = checkSameClient(message);
    if (sameClient) {
        DEBUG_PRINTLN("Same Client -> skipping");
        return false;
    }

    return true;
}

  

void callbackRecieveRaw(const char message[]) {
    uint16_t separatorPos = separatorPositionPlusOne(message);
    uint16_t messageLength = strlen(message);

    if (!checkCallbackRecieveCommon(message, separatorPos, messageLength)) {
        return;
    }

    DEBUG_PRINTLN("New raw message received:");
    DEBUG_PRINTLN((char*) &message[separatorPos+1]);

    uint16_t signalLength = (uint16_t) ((message[separatorPos + 1] << 8) | (message[separatorPos + 2] & 0b11111111));

    DEBUG_PRINT("Signal length: ");
    DEBUG_PRINTLN(signalLength);
    yield();
      
    uint16_t rawSignal[signalLength];
    uint16_t rawSignalIdx = 0;
    uint16_t messagePos = separatorPos + 2;
    // Merge char bytes array into uint16_t array for IR-remote
    for (; messagePos < messageLength && rawSignalIdx < signalLength; messagePos += 2, rawSignalIdx++) {
        rawSignal[rawSignalIdx] = (uint16_t) ((message[messagePos] << 8) | (message[messagePos + 1] & 0b11111111));
        yield();
    }

    #ifdef DEBUG
        DEBUG_PRINT("Restored signal: ");
        for (rawSignalIdx = 0; rawSignalIdx < signalLength; rawSignalIdx++) {
            if (rawSignalIdx) {
                DEBUG_PRINT(", ");
            }
            DEBUG_PRINT(rawSignal[rawSignalIdx]);
            yield();
        }
        DEBUG_PRINTLN(" ");
    #endif

    // Send signal
    IrSender.sendRaw(rawSignal, signalLength, 38); // Send a raw data capture at 38kHz.
}


/** In development
void callbackRecieveCoded(const char message[]) {
    uint16_t separatorPos = separatorPositionPlusOne(message);
    uint16_t messageLength = strlen(message);

    if (!checkCallbackRecieveCommon(message, separatorPos, messageLength)) {
        return;
    }

    DEBUG_PRINTLN("New IR with state message received:");
    DEBUG_PRINTLN((char*) &message[separatorPos+1]);

    if (messageLength - separatorPos < 9) {
        DEBUG_PRINTLN("ERROR: message too short -> skipping");
        return;
    }

    IRDataToEmit.protocol = (decode_type_t) (
        (((decode_type_t) message[separatorPos + 1]) << 8)
        | (((decode_type_t) message[separatorPos + 2]) & 0b11111111)
    );
    IRDataToEmit.address = (uint16_t) (
        (((uint16_t) message[separatorPos + 3]) << 8)
        | (((uint16_t) message[separatorPos + 4]) & 0b11111111)
    );
    IRDataToEmit.command = (uint16_t) (
        (((uint16_t) message[separatorPos + 5]) << 8)
        | (((uint16_t) message[separatorPos + 6]) & 0b11111111)
    );
    IRDataToEmit.numberOfBits = (uint16_t) (
        (((uint16_t) message[separatorPos + 7]) << 8)
        | (((uint16_t) message[separatorPos + 8]) & 0b11111111)
    );
    IRDataToEmit.flags = ((uint8_t) message[separatorPos + 9]) & 0b11111111;

    IrSender.write(&IRDataToEmit);
}
*/


void mqtt_reconnect() {
	if (!mqttClient.connected() && currentWifiMode == 'C' && strcmp(ApiKey, "Get one for free on https://www.foroelectro.net/arduino/en/mqtt-doc") != 0) {
		DEBUG_PRINT("Attempting MQTT connection...");
        DEBUG_PRINTLN(clientName);
        DEBUG_PRINTLN(ApiKey);
        DEBUG_PRINTLN(email);

		// Attempt to connect
		if (mqttClient.connect(clientName, ApiKey, email)) {
            String message = clientName;
            message += "||";
			message.concat(WiFi.localIP().toString());
			DEBUG_PRINTLN("connected");

			mqttClient.subscribe(fullTopic);
			DEBUG_PRINT("Subscribed to topic ");
			DEBUG_PRINTLN(fullTopic);
            sendMQTTMessageConnect();
		} else {
			DEBUG_PRINT("failed, rc=");
			DEBUG_PRINT(mqttClient.state());
			DEBUG_PRINTLN(" try again soon");
		}
	}
}




// This section of code runs only once at start-up.
void setup() {
    uDebugLibInit();
    DEBUG_PRINTLN("");
    delay(2000);
    DEBUG_PRINTLN("SETUP START");
    setup_config();
    DEBUG_PRINTLN("setupConfig OK");
    if (clientName != NULL && clientName) {
        clientNameLen = strlen(clientName);
    } else {
        clientNameLen = 0;
    }
    DEBUG_PRINTLN("clientNameLen OK");
    setup_wifi();
    DEBUG_PRINTLN("setup_wifi OK");
    setup_ir();
    DEBUG_PRINTLN("setup_ir OK");
    setup_mqtt();
    DEBUG_PRINTLN("setup_mqtt OK");
    setup_web();
    DEBUG_PRINTLN("setup_web OK");

    DEBUG_PRINTLN("SETUP END");
}

void sendMQTTMessageConnect() {
	String output = clientName;
	output += "||";
    output.concat(WiFi.localIP().toString());

    char * messageChar = (char *) malloc(sizeof(char) * output.length() * 2 + 1);
    uMessagesBrokerLib::encode('C', output.c_str(), messageChar);
    mqttClient.publish(fullTopic, messageChar);
    free (messageChar);
}


void sendMQTTMessageIRRaw() {
    uint16_t size = IrReceiver.decodedIRData.rawDataPtr->rawlen - 1;
	uint16_t idx = clientNameLen + 2;
    DEBUG_PRINT("Received raw IR signal of size ");
    DEBUG_PRINT(size);
	
	if (size < atoi(config->getPointer("kIrLed"))) {
        DEBUG_PRINTLN(". Skipping.");
        return;
	}
 
    DEBUG_PRINTLN(". sending to MQTT!.");
    strcpy(mqttData, clientName);
    mqttData[clientNameLen] = '|';
    mqttData[clientNameLen + 1] = '|';
    mqttData[idx] = (char) (size >> 8 & 0b11111111);
    idx++;
    mqttData[idx] = (char) (size & 0b11111111);
    idx++;
    yield();
    uint8_t * rawData = (uint8_t *) malloc(sizeof(uint8_t) * IrReceiver.decodedIRData.rawDataPtr->rawlen + 1);
    IrReceiver.compensateAndStoreIRResultInArray(rawData);
    for (uint16_t i = 0; i < IrReceiver.decodedIRData.rawDataPtr->rawlen; i++, idx++) {
        mqttData[idx] = rawData[i];
    }
    free (rawData);

    char * messageChar = (char *) malloc(sizeof(char) * idx * 2 + 1);
    uMessagesBrokerLib::encode('R', mqttData, messageChar, idx);
    mqttClient.publish(fullTopic, messageChar);
    free (messageChar);
}

/** In development
void sendMQTTMessageIRStructured() {
    strcpy(mqttData, clientName);
    mqttData[clientNameLen] = '|';
    mqttData[clientNameLen + 1] = '|';
    mqttData[clientNameLen + 2] = (char) (IrReceiver.decodedIRData.protocol >> 8 & 0b11111111);
    mqttData[clientNameLen + 3] = (char) (IrReceiver.decodedIRData.protocol & 0b11111111);
    mqttData[clientNameLen + 4] = (char) (IrReceiver.decodedIRData.address >> 8 & 0b11111111);
    mqttData[clientNameLen + 5] = (char) (IrReceiver.decodedIRData.address & 0b11111111);
    mqttData[clientNameLen + 6] = (char) (IrReceiver.decodedIRData.command >> 8 & 0b11111111);
    mqttData[clientNameLen + 7] = (char) (IrReceiver.decodedIRData.command & 0b11111111);
    mqttData[clientNameLen + 8] = (char) (IrReceiver.decodedIRData.numberOfBits >> 8 & 0b11111111);
    mqttData[clientNameLen + 9] = (char) (IrReceiver.decodedIRData.numberOfBits & 0b11111111);
    mqttData[clientNameLen + 10] = (char) (IrReceiver.decodedIRData.flags & 0b11111111);
    mqttData[clientNameLen + 11] = 0;
    yield();

    uint16_t length = (clientNameLen + 10) * 2;
    char * messageChar = (char *) malloc(sizeof(char) * (length * 2 + 1));
    uMessagesBrokerLib::encode('S', mqttData, messageChar, length);
    mqttClient.publish(fullTopic, messageChar);
    free (messageChar);
}
*/

// The repeating section of the code
void loop() {
    if (last_mqtt_connect < (millis() - 1000) && !mqttClient.connected()) {
        last_mqtt_connect = millis();
        mqtt_reconnect();
        yield();
    }
    mqttClient.loop();
    yield();
    if(doReset == 2) {
        // Little delay to avoid cuttin current connection
        for (int i = 0; i < 10; i++) {
            yield();
            mqttClient.loop();
            yield();
            delay(100);
        }
        ESP.reset();
    }
        

    if (IrReceiver.decode()) {
        if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {
            yield();
            DEBUG_PRINTLN("ERROR. Overflow.");
        } else {
            yield();

/** In development
      // If protocol is unknown or MAGIQUEST (needed extra data not implemented) we send it raw
      if (IrReceiver.decodedIRData.protocol == decode_type_t::UNKNOWN || IrReceiver.decodedIRData.protocol == decode_type_t::MAGIQUEST) {
        sendMQTTMessageIRRaw();
      } else  {  // Coded signal
        sendMQTTMessageIRStructured();
      }
*/
// In development, alternative path:
sendMQTTMessageIRRaw();
      
            // Resume capturing IR messages.
            IrReceiver.resume();
        }
        yield();
    }
    server.handleClient();
    yield();
}
