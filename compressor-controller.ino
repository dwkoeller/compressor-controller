//This can be used to output the date the code was compiled
const char compile_date[] = __DATE__ " " __TIME__;

/************ WIFI, OTA and MQTT INFORMATION (CHANGE THESE FOR YOUR SETUP) ******************/
//#define WIFI_SSID "" //enter your WIFI SSID
//#define WIFI_PASSWORD "" //enter your WIFI Password
//#define MQTT_SERVER "" // Enter your MQTT server address or IP.
//#define MQTT_USER "" //enter your MQTT username
//#define MQTT_PASSWORD "" //enter your password
#define MQTT_DEVICE "compressor_controller" // Enter your MQTT device
#define MQTT_DEVICE_NAME "Air Compressor Controller"
#define MQTT_SSL_PORT 8883 // Enter your MQTT server port.
#define MQTT_SOCKET_TIMEOUT 120
#define FW_UPDATE_INTERVAL_SEC 24*3600
#define STATUS_UPDATE_INTERVAL_SEC 120
#define PRESSURE_UPDATE_INTERVAL_MS 250
#define PUSH_BUTTON_UPDATE_INTERVAL_MS 333
#define PURGE_DELAY_MS 3000
#define UPDATE_SERVER "http://192.168.100.15/firmware/"
#define FIRMWARE_VERSION "-1.03"

/****************************** MQTT TOPICS (change these topics as you wish)  ***************************************/

#define MQTT_HEARTBEAT_SUB "heartbeat/#"
#define MQTT_HEARTBEAT_TOPIC "heartbeat"
#define MQTT_UPDATE_REQUEST "update"
#define MQTT_DISCOVERY_SWITCH_PREFIX  "homeassistant/switch/"
#define MQTT_DISCOVERY_SENSOR_PREFIX  "homeassistant/sensor/"
#define HA_TELEMETRY "ha"

#define RELAY_ON 0
#define RELAY_OFF 1
#define SWITCH_ON "ON"
#define SWITCH_OFF "OFF"
#define RELAY_COMPRESSOR    5  //  D1
#define RELAY_PURGE         4  //  D2
#define PRESSURE_PIN        13 //  D7
#define WATCHDOG_PIN        14 //  D5
#define PUSH_BUTTON_PIN    12 //  D6

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include "credentials.h" // Place credentials for wifi and mqtt in this file
#include "certificates.h" // Place certificates for mqtt in this file

Ticker ticker_fw, ticker_status, tickerPressureState, tickerPushButton;

bool readyForFwUpdate = false;
bool readyForPressureUpdate = false;
bool readyForPushButtonUpdate = false;
int relayStatus = 0;
bool registered = false;

String switchStateTopic = String(MQTT_DISCOVERY_SWITCH_PREFIX) + MQTT_DEVICE + "/state";
String switchCommandTopic = String(MQTT_DISCOVERY_SWITCH_PREFIX) + MQTT_DEVICE + "/command";

String state = "";
String lastPressureState = "";
String lastPushButtonState = "";

WiFiClientSecure espClient;
PubSubClient client(espClient);

#include "common.h"

void setup() {
  Serial.begin(115200);
  setup_wifi();

  IPAddress result;
  int err = WiFi.hostByName(MQTT_SERVER, result) ;
  if(err == 1){
        Serial.print("MQTT Server IP address: ");
        Serial.println(result);
        MQTTServerIP = result.toString();
  } else {
        Serial.print("Error code: ");
        Serial.println(err);
  }  

  client.setBufferSize(512);  
  client.setServer(MQTT_SERVER, MQTT_SSL_PORT); //CHANGE PORT HERE IF NEEDED
  client.setCallback(callback);

  digitalWrite(RELAY_COMPRESSOR, RELAY_OFF);
  digitalWrite(RELAY_PURGE, RELAY_OFF);
  digitalWrite(WATCHDOG_PIN, LOW);

  pinMode(RELAY_COMPRESSOR, OUTPUT);
  pinMode(RELAY_PURGE, OUTPUT);
  pinMode(WATCHDOG_PIN, OUTPUT);
  pinMode(PRESSURE_PIN, INPUT_PULLUP);
  pinMode(PUSH_BUTTON_PIN, INPUT_PULLUP);

  ticker_status.attach_ms(STATUS_UPDATE_INTERVAL_SEC * 1000, statusTicker);
  tickerPressureState.attach_ms(PRESSURE_UPDATE_INTERVAL_MS, pressureStateTickerFunc);
  ticker_fw.attach_ms(FW_UPDATE_INTERVAL_SEC * 1000, fwTicker);
  tickerPushButton.attach_ms(PUSH_BUTTON_UPDATE_INTERVAL_MS, pushButtonStateTickerFunc);

  checkForUpdates();
  resetWatchdog();

}

void loop() {

  client.loop();

  if(readyForFwUpdate) {
    readyForFwUpdate = false;
    checkForUpdates();
  }

  if(readyForPressureUpdate) {
    readyForPressureUpdate = false;
    checkPresureState();
  }

  if(readyForPushButtonUpdate) {
    readyForPushButtonUpdate = false;
    checkPushButtonState();
  }

  if (!client.connected()) {
      reconnect();
      client.subscribe(switchCommandTopic.c_str());

  }
  if (! registered) {
    registerTelemetry();
    updateTelemetry("Unknown");
    createSwitch(MQTT_DEVICE, MQTT_DEVICE_NAME);
    registered = true;
  }

}

void callback(char* p_topic, byte* p_payload, unsigned int p_length) {

  //convert topic to string to make it easier to work with
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }   
  if (String(MQTT_HEARTBEAT_TOPIC).equals(p_topic)) {
    resetWatchdog();
    updateTelemetry(payload);
    if (payload.equals(String(MQTT_UPDATE_REQUEST))) {
      checkForUpdates();
    }    
    
    return;
  }
  if (payload.equals(String(SWITCH_ON))) {
    digitalWrite(RELAY_COMPRESSOR, RELAY_ON);
    updateSwitch(MQTT_DEVICE, SWITCH_ON);
    relayStatus = 1;
  }
  else if (payload.equals(String(SWITCH_OFF))) {
    digitalWrite(RELAY_COMPRESSOR, RELAY_OFF);
    updateSwitch(MQTT_DEVICE, SWITCH_OFF);
    relayStatus = 0;
  }
}

void statusTicker() {
  String status;
  if (relayStatus > 0) {
    status = "ON";
  }
  else {
    status = "OFF";
  }
  updateSwitch(MQTT_DEVICE, status.c_str());
}

void createSwitch(String switch_device, String switch_name) {
  String topic = String(MQTT_DISCOVERY_SWITCH_PREFIX) + switch_device + "/config";
  String message = String("{\"name\": \"") + switch_name +
                   String("\", \"retain\": \"true") +
                   String("\", \"unique_id\": \"") + switch_device + getUUID() +
                   String("\", \"optimistic\": \"false") +
                   String("\", \"command_topic\": \"") + String(MQTT_DISCOVERY_SWITCH_PREFIX) + switch_device +
                   String("/command\", \"state_topic\": \"") + String(MQTT_DISCOVERY_SWITCH_PREFIX) + switch_device +
                   String("/state\"}");
  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(message.c_str());

  client.publish(topic.c_str(), message.c_str(), true);

}

void updateSwitch(String switch_device, String state) {
  String topic = String(MQTT_DISCOVERY_SWITCH_PREFIX) + switch_device + "/state";

  Serial.print(F("MQTT - "));
  Serial.print(topic);
  Serial.print(F(" : "));
  Serial.println(state);
  client.publish(topic.c_str(), state.c_str(), true);

}

void checkPresureState() {
 
  state = getCurrentState(PRESSURE_PIN);
  if(state != lastPressureState) {
    lastPressureState = state;
    if(state == "ON") {
      purgeCompressor();
    }
  }
}

void checkPushButtonState() {
 
  state = getCurrentState(PUSH_BUTTON_PIN);
  if(state != lastPushButtonState) {
    lastPushButtonState = state;
    if(state == "ON") {
      if(relayStatus == 1) {
        digitalWrite(RELAY_COMPRESSOR, RELAY_OFF);
        updateSwitch(MQTT_DEVICE, SWITCH_OFF);
        relayStatus = 0;
      }
      else {
        digitalWrite(RELAY_COMPRESSOR, RELAY_ON);
        updateSwitch(MQTT_DEVICE, SWITCH_ON);
        relayStatus = 1;        
      }
      delay(1500);
    }
  }
}

String getCurrentState(int pin) {
  String state;
  int val;
  val = digitalRead(pin);
  if(val == LOW) {
    state = "ON";
  }
  else {
    state = "OFF";    
  }
  return state;
}

void pressureStateTickerFunc() {
  readyForPressureUpdate = true;
}

void pushButtonStateTickerFunc() {
  readyForPushButtonUpdate = true;
}

void purgeCompressor() {
  digitalWrite(RELAY_PURGE, RELAY_ON);
  delay(PURGE_DELAY_MS);
  digitalWrite(RELAY_PURGE, RELAY_OFF);
}
