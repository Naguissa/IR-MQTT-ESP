/**
 * MQTT messages:
 *   IRMQTT-C - ESP Connected to MQTT
 *   IRMQTT-R-<rawdata> - ESP received IR sensor data, <rawdata> is the raw dump as a continous uint16_t hex encoded values (4 characters for each uint16_t value)
 */

#include <Arduino.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRrecv.h>
#include <IRac.h>
#include <IRtext.h>
#include <IRutils.h>
#include <ESP8266WiFi.h>
#define FS_NO_GLOBALS  // Avoid File compile error on Eclipse
#include "FS.h"
extern fs::FS SPIFFS;

#define MQTT_MAX_PACKET_SIZE 1600
#include "libraries/PubSubClient/src/PubSubClient.h"
#include "libraries/PubSubClient/src/PubSubClient.cpp"

#include <WiFiClient.h>

// Uncomment these lines in order to activate Serial debug; unable if you are using ESP-01
// #define DEBUG 1
// #define UDEBUGLIB_BAUDS 57600


// ==================== start of TUNEABLE PARAMETERS ====================


const uint16_t kRecvPin = 14; // IR demodulator  connected to GPIO pin 14 - D5 on a NodeMCU board.
const uint16_t kIrLed = 4; // Base of NPN transistor to IR emitter connected to GPIO pin 4 - D2 on a NodeMCU board.

// As this program is a special purpose capture/decoder, let us use a larger than normal buffer so we can handle Air Conditioner remote codes.
const uint16_t kCaptureBufferSize = 1024;

// Some A/C units have gaps in their protocols of ~40ms. e.g. Kelvinator. A value this large may swallow repeats of some protocols
const uint8_t kTimeout = 50;

// Set the smallest sized "UNKNOWN" message packets we actually care about.
// This value helps reduce the false-positive detection rate of IR background
// noise as real messages. The chances of background IR noise getting detected
// as a message increases with the length of the kTimeout value. (See above)
// The downside of setting this message too large is you can miss some valid
// short messages for protocols that this library doesn't yet decode.
//
// Set higher if you get lots of random short UNKNOWN messages when nothing
// should be sending a message.
// Set lower if you are sure your setup is working, but it doesn't see messages
// from your device. (e.g. Other IR remotes work.)
// NOTE: Set this value very high to effectively turn off UNKNOWN detection.
const uint16_t kMinUnknownSize = 12;

// ==================== end of TUNEABLE PARAMETERS ====================
const char *mqtt_server = "foroelectro.net";
char *fullTopic;

char* ApiKey; // "ApiKey from https://www.foroelectro.net/arduino/en/mqtt-doc";
char* email; // "email corresponding to previous ApiKey";
char *clientName; 
char *topic;

char* ssid;
char *password;



byte clientNameLen = 0;
#define MAX_MQTT_MSG_LEN 1600

char mqttData[MAX_MQTT_MSG_LEN];
//volatile uint16_t rawSendData[200];
// volatile uint8_t rawSendDataLength = 0;
IRsend irsend(kIrLed); // Set the GPIO to be used to sending the message.

unsigned long int last_mqtt_connect = 0;
unsigned long int doResetTime = 0;
// =======================================================
// ==================== FUNCTIONALITY ====================
// =======================================================


// ==================== Function definitions ====================
void hexEncode(const byte);
byte hexDecode(const byte, const byte);

void handleNotFound();
void handleSaveConfig();
void setupUpdater(void);


void setup_wifi();
void setup_mqtt();
void mqtt_reconnect();
void receiveMQTTMessage(char*, byte*, unsigned int);
void getMQTTMessage();

void setup_ir();
void emitIR();
void setup();
void loop();

void parseConfigString(char **, String *);
void parseConfigLine(String);
void loadConfig();
bool saveConfig();


// ==================== coding-decoding  ====================
char hexEncodeRet[2];

void hexEncode(const byte in) {
  switch (in >> 4) {
    case 0: hexEncodeRet[0] = '0'; break;
    case 1: hexEncodeRet[0] = '1'; break;
    case 2: hexEncodeRet[0] = '2'; break;
    case 3: hexEncodeRet[0] = '3'; break;
    case 4: hexEncodeRet[0] = '4'; break;
    case 5: hexEncodeRet[0] = '5'; break;
    case 6: hexEncodeRet[0] = '6'; break;
    case 7: hexEncodeRet[0] = '7'; break;
    case 8: hexEncodeRet[0] = '8'; break;
    case 9: hexEncodeRet[0] = '9'; break;
    case 10: hexEncodeRet[0] = 'A'; break;
    case 11: hexEncodeRet[0] = 'B'; break;
    case 12: hexEncodeRet[0] = 'C'; break;
    case 13: hexEncodeRet[0] = 'D'; break;
    case 14: hexEncodeRet[0] = 'E'; break;
    case 15: default: hexEncodeRet[0] = 'F'; break;
  }
  
  switch (in & 0b00001111) {
    case 0: hexEncodeRet[1] = '0'; break;
    case 1: hexEncodeRet[1] = '1'; break;
    case 2: hexEncodeRet[1] = '2'; break;
    case 3: hexEncodeRet[1] = '3'; break;
    case 4: hexEncodeRet[1] = '4'; break;
    case 5: hexEncodeRet[1] = '5'; break;
    case 6: hexEncodeRet[1] = '6'; break;
    case 7: hexEncodeRet[1] = '7'; break;
    case 8: hexEncodeRet[1] = '8'; break;
    case 9: hexEncodeRet[1] = '9'; break;
    case 10: hexEncodeRet[1] = 'A'; break;
    case 11: hexEncodeRet[1] = 'B'; break;
    case 12: hexEncodeRet[1] = 'C'; break;
    case 13: hexEncodeRet[1] = 'D'; break;
    case 14: hexEncodeRet[1] = 'E'; break;
    case 15: default: hexEncodeRet[1] = 'F'; break;
  }  
}


byte hexDecode(const byte inA, const byte inB) {
  byte ret = 0;
  switch (inA) {
    case '0': break;
    case '1': ret = 0b00010000; break;
    case '2': ret = 0b00100000; break;
    case '3': ret = 0b00110000; break;
    case '4': ret = 0b01000000; break;
    case '5': ret = 0b01010000; break;
    case '6': ret = 0b01100000; break;
    case '7': ret = 0b01110000; break;
    case '8': ret = 0b10000000; break;
    case '9': ret = 0b10010000; break;
    case 'A': ret = 0b10100000; break;
    case 'B': ret = 0b10110000; break;
    case 'C': ret = 0b11000000; break;
    case 'D': ret = 0b11010000; break;
    case 'E': ret = 0b11100000; break;
    case 'F': default: ret = 0b11110000; break;
  }
  
  switch (inB) {
    case '0': break;
    case '1': ret |= 0b00000001; break;
    case '2': ret |= 0b00000010; break;
    case '3': ret |= 0b00000011; break;
    case '4': ret |= 0b00000100; break;
    case '5': ret |= 0b00000101; break;
    case '6': ret |= 0b00000110; break;
    case '7': ret |= 0b00000111; break;
    case '8': ret |= 0b00001000; break;
    case '9': ret |= 0b00001001; break;
    case 'A': ret |= 0b00001010; break;
    case 'B': ret |= 0b00001011; break;
    case 'C': ret |= 0b00001100; break;
    case 'D': ret |= 0b00001101; break;
    case 'E': ret |= 0b00001110; break;
    case 'F': default: ret |= 0b00001111; break;
  }
  return ret;
}



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

// ==================== ESP web and update ====================
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>

ESP8266WebServer server(80);
ESP8266HTTPUpdateServer httpUpdater;
WiFiClient espClient;

void handleNotFound() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", F("<HTML>"));
server.sendContent(F("<HEAD><TITLE>ESP IR MQTT - github.com/Naguissa/ESP-IR-MQTT</TITLE></HEAD>"));
server.sendContent(F("<BODY>"));
server.sendContent(F("<H1>Configuration</H1>"));
server.sendContent(F("<FORM METHOD=\"POST\" ACTION=\"/saveConfig\">"));
server.sendContent(F("<P>WiFi SSID: <INPUT TYPE=\"text\" name=\"ssid\" value=\""));
server.sendContent(ssid);
server.sendContent(F("\"></P>"));

server.sendContent(F("<P>WiFi password: <INPUT TYPE=\"text\" name=\"password\" value=\""));
if (password != NULL && password) {
  server.sendContent(password);
}
server.sendContent(F("\"></P>"));
server.sendContent(F("<P>MQTT ApiKey: <INPUT TYPE=\"text\" name=\"ApiKey\" value=\""));
if (ApiKey != NULL && ApiKey) {
  server.sendContent(ApiKey);
}
server.sendContent(F("\"></P>"));
server.sendContent(F("<P>MQTT email: <INPUT TYPE=\"text\" name=\"email\" value=\""));
if (email != NULL && email) {
  server.sendContent(email);
}
server.sendContent(F("\"></P>"));
server.sendContent(F("<P>MQTT client name: <INPUT TYPE=\"text\" name=\"clientName\" value=\""));
if (clientName != NULL && clientName) {
  server.sendContent(clientName);
}
server.sendContent(F("\"></P>"));
server.sendContent(F("<P>MQTT topic: <INPUT TYPE=\"text\" name=\"topic\" value=\""));
if (topic != NULL && topic) {
  server.sendContent(topic);
}
server.sendContent(F("\"></P>"));
server.sendContent(F("<P><INPUT TYPE=\"submit\" value=\" Save config and reset \"></P>"));
server.sendContent(F("</FORM>"));
server.sendContent(F("</BODY>"));
server.sendContent(F("</HTML>"));
}

void setupUpdater() {
  httpUpdater.setup(&server, "/update");
  server.on("/saveConfig", HTTP_POST, handleSaveConfig);
  server.onNotFound(handleNotFound);
  server.begin();
}


void handleSaveConfig() {
  String value;

  DEBUG_PRINTLN("SAVE CONFIG");

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  yield();

  value = server.arg("ssid");
  value.trim();
  if (value != "") {
    parseConfigString(&ssid, &value);
    yield();
  }

  value = server.arg("password");
  value.trim();
  if (value != "") {
    parseConfigString(&password, &value);
    yield();
  }

  value = server.arg("ApiKey");
  value.trim();
  if (value != "") {
    parseConfigString(&ApiKey, &value);
    yield();
  }

  value = server.arg("email");
  value.trim();
  if (value != "") {
    parseConfigString(&email, &value);
    yield();
  }

  value = server.arg("clientName");
  value.trim();
  if (value != "") {
    parseConfigString(&clientName, &value);
    yield();
  }

  value = server.arg("topic");
  value.trim();
  if (value != "") {
    parseConfigString(&topic, &value);
    yield();
  }

   yield();

  saveConfig();
  yield();

  server.sendHeader("Location", "/", true);
  server.send(302);
  doResetTime = millis();
  yield();
 
}

// ==================== Setup functions ====================
PubSubClient client(espClient);





// Use turn on the save buffer feature for more complete capture coverage.
IRrecv irrecv(kRecvPin, kCaptureBufferSize, kTimeout, true);
decode_results results; // Somewhere to store the results



void setupMandatoryInitialValues() {
  // Initial values of SSID and pass
  ssid = (char *) malloc(11);
  strcpy(ssid, "IR-MQTT-AP");
}



void setup_wifi() {
	delay(10);
	// We start by connecting to a WiFi network
	DEBUG_PRINTLN(' ');


  DEBUG_PRINT("Connecting to ");
  DEBUG_PRINTLN(ssid);
  DEBUG_PRINT(" in ");
 if (password != NULL && password[0] != '\0') {
    DEBUG_PRINT("STA mode");
    WiFi.mode(WIFI_STA);
  	WiFi.begin(ssid, password);
    for (byte i = 0; i < 30 && WiFi.status() != WL_CONNECTED; i++) {
      delay(500);
      DEBUG_PRINT(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
      DEBUG_PRINTLN(' ');
      DEBUG_PRINTLN("CONNECTION FAILED! Entering in AP mode");
  }
 }
if (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINT("AP mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid);
 }
	randomSeed (micros());
	DEBUG_PRINTLN("");
  if (password) {
	  DEBUG_PRINTLN("WiFi connected");
	  DEBUG_PRINTLN("IP address: ");
	  DEBUG_PRINTLN(WiFi.localIP());
  } else {
    DEBUG_PRINTLN("WiFi created");
    DEBUG_PRINTLN("AP IP address: ");
    DEBUG_PRINTLN(WiFi.softAPIP());
 }
}





void receiveMQTTMessage(char *topicR, byte *payload, unsigned int length) {
	DEBUG_PRINT("Message arrived [");
	DEBUG_PRINT(topicR);
	DEBUG_PRINTLN("]");

  bool sameClient = true;
  uint16_t i = 0;

  yield();
  // Client name
  DEBUG_PRINT("Client name (debug only): ");
	for (; i < length && payload[i] != '-'; i++) {
		DEBUG_PRINT((char) payload[i]);
   if (clientNameLen > i && clientName[i] != payload[i]) {
      sameClient = false;
   }
	}

	DEBUG_PRINTLN(' ');
  DEBUG_PRINT("Same client? ");
  DEBUG_PRINTLN(sameClient ? "Y -> return" : "N -> continue");
  yield();

  if (sameClient) {
    return;
  }


  // Bad-formed message
  if (i + 2 > length || payload[i+2] != '-') {
    return;
  }

  // Operation:
  i++;
  switch(payload[i]) {

#ifdef DEBUG
    case 'C': // New device connected
      payload[length] = '\0';
      DEBUG_PRINT("New device connected with IP: ");
      DEBUG_PRINTLN((char*) &payload[i+2]);
      break;
#endif
    
    case 'R': // New IR signal to repeat
      DEBUG_PRINTLN("New IR signal received, repeating...");

      uint16_t signalLen;
      byte byteTmp = 0;
      
      byteTmp = hexDecode(payload[i+2], payload[i+3]);
      signalLen = (byteTmp << 8);
      byteTmp = hexDecode(payload[i+4], payload[i+5]);
      signalLen |= (byteTmp & 0b11111111);
      
      if (i+8 >= length) { // Bad-formed message
        return;
      }

      DEBUG_PRINT("Signal length: ");
      DEBUG_PRINTLN((unsigned long int) signalLen);

      i += 7;

      yield();
      
      uint16_t rawSignal[signalLen];
      uint16_t rawSignalIdx = 0;
      for (; i < length && rawSignalIdx < signalLen; i += 4, rawSignalIdx++) {
        byteTmp = hexDecode(payload[i], payload[i+1]);
        rawSignal[rawSignalIdx] = (byteTmp << 8);
        byteTmp = hexDecode(payload[i+2], payload[i+3]);
        rawSignal[rawSignalIdx] |= (byteTmp & 0b11111111);
      }

      #ifdef DEBUG
        yield();
        DEBUG_PRINT("Restored signal: ");
        for (int16_t debugTmp = 0; debugTmp < signalLen; debugTmp++) {
          if (debugTmp) {
            DEBUG_PRINT(", ");
          }
          DEBUG_PRINT(rawSignal[debugTmp]);
        }
        DEBUG_PRINTLN(' ');
      #endif

      yield();
      irsend.sendRaw(rawSignal, signalLen, 38); // Send a raw data capture at 38kHz.

     break;

    // default: // Unknown, ignore
  }
 
}






void mqtt_reconnect() {
	if (!client.connected() && clientName != NULL && ApiKey != NULL && email != NULL) {
		DEBUG_PRINT("Attempting MQTT connection...");

    DEBUG_PRINTLN(clientName);
    DEBUG_PRINTLN(ApiKey);
    DEBUG_PRINTLN(email);

   
		// Attempt to connect

		if (client.connect(clientName, ApiKey, email)) {
      String message = clientName;
      message += "-C-";
      char messageChar[120];
			message.concat(WiFi.localIP().toString());
			DEBUG_PRINTLN("connected");
			client.subscribe(fullTopic);
			DEBUG_PRINT("Subscribed to topic ");
			DEBUG_PRINTLN(fullTopic);
			message.toCharArray(messageChar, 120);
			client.publish(fullTopic, messageChar);
			DEBUG_PRINT("Sent to MQTT: ");
			DEBUG_PRINTLN(messageChar);
		} else {
			DEBUG_PRINT("failed, rc=");
			DEBUG_PRINT(client.state());
			DEBUG_PRINTLN(" try again soon");
		}
	}
}

// This section of code runs only once at start-up.
void setup_ir() {
	irsend.begin();
	irrecv.setUnknownThreshold(kMinUnknownSize);
	irrecv.enableIRIn(); // Start the receiver
}

void setup_mqtt() {
	String stopic = ApiKey;
	stopic.concat("/");
  stopic.concat(topic);
  if (fullTopic != NULL && fullTopic && strlen(fullTopic)) {
    free(fullTopic);
  }
  fullTopic = (char *) malloc(stopic.length() + 1);
	stopic.toCharArray(fullTopic, stopic.length() + 1);

	client.setServer(mqtt_server, 1883);
	client.setCallback(receiveMQTTMessage);
}



// This section of code runs only once at start-up.
void setup() {
	uDebugLibInit();
  delay(2000);
  setupMandatoryInitialValues();
  DEBUG_PRINTLN("setupMandatoryInitialValues OK");
  loadConfig();
  DEBUG_PRINTLN("loadConfig OK");
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
  setupUpdater();
  DEBUG_PRINTLN("setupUpdater OK");
}

void getMQTTMessage() {
	String output = clientName;

  uint16_t len = getCorrectedRawLength(&results);
	output += "-R-";
  hexEncode((byte) (len >> 8));
  output += hexEncodeRet[0];
  output += hexEncodeRet[1];
  hexEncode((byte) (len & 0b11111111));
  output += hexEncodeRet[0];
  output += hexEncodeRet[1];
	output += "-";
	yield();
	for (uint16_t i = 1; i < results.rawlen; i++) {
		uint32_t usecs;
		for (usecs = results.rawbuf[i] * kRawTick; usecs > UINT16_MAX; usecs -= UINT16_MAX) {
			output += F("FFFF"); // FFFF = UINT16_MAX hex encoding
			output += F("0000"); // 0
		}
    // Max remaining is UINT16_MAX - 1, so encode 2 bytes:
    hexEncode((byte) (usecs >> 8));
    output += hexEncodeRet[0];
    output += hexEncodeRet[1];
    hexEncode((byte) (usecs & 0b11111111));
    output += hexEncodeRet[0];
    output += hexEncodeRet[1];
		yield();
	}
	output.toCharArray(mqttData, MAX_MQTT_MSG_LEN);
}





// The repeating section of the code
void loop() {
    if (doResetTime > 0) {
      if (doResetTime < (millis() - 1000)) {
        DEBUG_PRINTLN("RESET!");
        ESP.reset();
      }
    } else {
  	if (last_mqtt_connect < (millis() - 1000) && !client.connected()) {
      last_mqtt_connect = millis();
  		mqtt_reconnect();
  		yield();
  	}
  	client.loop();
  	yield();
  
  	if (irrecv.decode(&results)) {
  		if (results.overflow) {
  			yield();
  #ifdef DEBUG
        DEBUG_PRINTLN("Overflow");
        char message[MAX_MQTT_MSG_LEN];
        sprintf(message, D_WARN_BUFFERFULL, kCaptureBufferSize);
        DEBUG_PRINTLN(message);
  #endif
  		}
  		yield();
  		DEBUG_PRINT(resultToHumanReadableBasic(&results));
  		String description = IRAcUtils::resultAcToString(&results);
  		if (description.length()) {
  			yield();
  		DEBUG_PRINTLN(D_STR_MESGDESC ": " + description);
  	}
  	yield();
  	DEBUG_PRINTLN(resultToSourceCode(&results));
  	yield();
  
  	getMQTTMessage();
  	yield();
    
    uint16_t len = strlen(mqttData);
    uint16_t idx = 0;
    client.beginPublish (fullTopic, len, false);
    yield();
    for (;idx < len; idx++) {
      client.write(mqttData[idx]);
      yield();
    }
    client.endPublish();
  	yield();
  
  	DEBUG_PRINTLN(' ');
  	DEBUG_PRINT("Sent MQTT data: ");
  	DEBUG_PRINTLN(mqttData);
  	DEBUG_PRINTLN(' ');
  }
  server.handleClient();

  }

}


// 
// ==================== CONFIG FUNCTIONS ====================
//

void parseConfigString(char ** dest, String * value) {
  if (*dest != NULL && strlen(*dest)) {
    free(*dest);
  }
  *dest = (char *) malloc(value->length() + 1);
  char tmpbuffer[200];
  value->toCharArray(tmpbuffer, 200);
  strcpy(*dest, tmpbuffer);
  *(*dest + value->length()) = '\0';
}


void parseConfigLine(String line) {
  int pos;
  String variable, value;
  line.trim();
  DEBUG_PRINT(F("CONFIG LINE START: "));
  DEBUG_PRINTLN(line);
  if (line.startsWith(";") || line.startsWith("#") || line.startsWith("//")) { // comment line
    DEBUG_PRINT(F("CONFIG LINE COMMENT: "));
    DEBUG_PRINTLN(line);
    return;
  }
  pos = line.indexOf('=');
  if (pos < 2) { // Not found or too short for sure, skip
    DEBUG_PRINT(F("CONFIG LINE TOO SHORT: "));
    DEBUG_PRINTLN(line);
    return;
  }
  variable = line.substring(0, pos - 1);
  value = line.substring(pos + 1);
  variable.trim();
  value.trim();

  DEBUG_PRINT(F("CONFIG LINE: "));
  DEBUG_PRINT(variable);
  DEBUG_PRINT(F(" = "));
  DEBUG_PRINTLN(value);

  if (variable.equals("ssid")) {
    parseConfigString(&ssid, &value);
  } else if (variable.equals("password")) {
    parseConfigString(&password, &value);
  } else if (variable.equals("ApiKey")) {
    parseConfigString(&ApiKey, &value);
  } else if (variable.equals("email")) {
    parseConfigString(&email, &value);
  } else if (variable.equals("clientName")) {
    parseConfigString(&clientName, &value);
  } else if (variable.equals("topic")) {
    parseConfigString(&topic, &value);
  } else {
    DEBUG_PRINT(F("CONFIG LINE UNKNOWN: "));
    DEBUG_PRINTLN(line);
  }
}




void loadConfig() {
  bool hasFS = SPIFFS.begin();
  if (!hasFS) {
    DEBUG_PRINTLN(F("Formatting SPIFFS..."));
    SPIFFS.format();
    hasFS = SPIFFS.begin();
  }
  if (hasFS) {
    DEBUG_PRINTLN(F("SPIFFS initialized."));
  }

  
  fs::File dataFile = SPIFFS.open("/CONFIG.TXT", "r");
  // check for open error
  if (!dataFile) {
    DEBUG_PRINTLN(F("Error opening SPIFFS:/CONFIG.TXT in ReadOnly mode - Not available."));
    return;
  }

  String line;
  // read lines from the file
  while (dataFile.available()) {
    line = dataFile.readStringUntil('\n');
    parseConfigLine(line);
  }

}

bool saveConfig() {
  fs::File configFile = SPIFFS.open("/CONFIG.TXT", "w");
  if (configFile) {
    configFile.println(F("# syntax:"));
    yield();
    configFile.println(F("# variable = value"));
    yield();
    configFile.println(F("# (spaces doesn't care)"));
    yield();
    configFile.println(F("# Comments: lines starting with #, ; or //"));
    yield();

    if (ssid != NULL && ssid && strlen(ssid)) {
      configFile.print(F("ssid = "));
      configFile.println(ssid);
    } else {
      configFile.println(F(";ssid = <Your WiFi SSID>"));
    }
    yield();

    if (password != NULL && password && strlen(password)) {
      configFile.print(F("password = "));
      configFile.println(password);
    } else {
      configFile.println(F(";password = <Your WiFi password>"));
    }
    yield();


    if (ApiKey != NULL && ApiKey && strlen(ApiKey)) {
      configFile.print(F("ApiKey = "));
      configFile.println(ApiKey);
    } else {
      configFile.println(F(";ApiKey = Get one at https://www.foroelectro.net/arduino/"));
    }
    yield();

    if (email != NULL && email && strlen(email)) {
      configFile.print(F("email = "));
      configFile.println(email);
    } else {
      configFile.println(F(";email = Corresponding to your ApiKey. Get one at https://www.foroelectro.net/arduino/"));
    }
    yield();

    if (clientName != NULL && clientName && strlen(clientName)) {
      configFile.print(F("clientName = "));
      configFile.println(clientName);
    } else {
      configFile.println(F(";clientName = Choose one simple and descriptive name, as LivingRoom. Use only numbers and letters for compatibility. NEVER use dash."));
    }
    yield();
    
    if (topic != NULL && topic && strlen(topic)) {
      configFile.print(F("topic = "));
      configFile.println(topic);
    } else {
      configFile.println(F(";topic = Choose one simple and descriptive name, as IR_BRIDGE. Used in MQTT server."));
    }
    yield();

    configFile.close();
    return true;
  } else {
    // if the file didn't open, print an error:
    DEBUG_PRINTLN("ERROR saving CONFIG.TXT");
    return false;
  }
}
