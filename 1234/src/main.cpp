#include <esp_now.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebSocketsClient.h>
#include <WiFiMulti.h>

// Replace with your network credentials (STATION)
const char* ssid = "Beeline_61";
const char* password = "87024670827";

#define WS_HOST "d80ktxonv3.execute-api.eu-central-1.amazonaws.com"
#define WS_PORT 443
#define WS_URL "/dev"

#define JSON_DOC_SIZE 2048
#define MSG_SIZE 256

WiFiMulti wifiMulti;
WebSocketsClient wsClient;

esp_now_peer_info_t slave;
int chan; 

enum MessageType {PAIRING, DATA,};
MessageType messageType;

int counter = 0;


int incomingisGasLeak;
bool incomingisMotionDetected, rele1IncomingValue, rele2IncomingValue, rele3IncomingValue;
float temperature, humidity, incomingTemp, incomingHum, incomingDistanceSensorValue;
String success;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  uint8_t msgType;
  uint8_t id;
  bool rele1, rele2, rele3, rele1OutgoingValue, rele2OutgoingValue, rele3OutgoingValue, isMotionDetected;
  float temp, hum, distanceSensorValue;
  int GasLeak;
  unsigned int readingId;
} struct_message;

typedef struct struct_pairing {       // new structure for pairing
    uint8_t msgType;
    uint8_t id;
    uint8_t macAddr[6];
    uint8_t channel;
} struct_pairing;

struct_message incomingReadings;
struct_message outgoingSetpoints;
struct_pairing pairingData;

// ---------------------------- esp_ now -------------------------
void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

bool addPeer(const uint8_t *peer_addr) {      // add pairing
  memset(&slave, 0, sizeof(slave));
  const esp_now_peer_info_t *peer = &slave;
  memcpy(slave.peer_addr, peer_addr, 6);
  
  slave.channel = chan; // pick a channel
  slave.encrypt = 0; // no encryption
  // check if the peer exists
  bool exists = esp_now_is_peer_exist(slave.peer_addr);
  if (exists) {
    // Slave already paired.
    Serial.println("Already Paired");
    return true;
  }
  else {
    esp_err_t addStatus = esp_now_add_peer(peer);
    if (addStatus == ESP_OK) {
      // Pair success
      Serial.println("Pair success");
      return true;
    }
    else 
    {
      Serial.println("Pair failed");
      return false;
    }
  }
} 

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success to " : "Delivery Fail to ");
  printMAC(mac_addr);
  Serial.println();
}

void sendSensorDataToClient(bool isMotionDetected, float temp, float hum, int gasLeak, float distanceSensorValue){
    char msg[MSG_SIZE];
    sprintf(msg,
          "{\"action\":\"msg\",\"type\":\"dataReceived\",\"body\":{\"isMotionDetected\":%d,"
          "\"temp\":%.1f,""\"hum\":%.1f,""\"gasLeak\":%d, ""\"distanceSensorValue\":%.1f}}",
          isMotionDetected, temp, hum, gasLeak, distanceSensorValue);  
    wsClient.sendTXT(msg);
}
void sendErrorMessage(const char *error) {
  char msg[MSG_SIZE];

  sprintf(msg, "{\"action\":\"msg\",\"type\":\"error\",\"body\":\"%s\"}",
          error);

  wsClient.sendTXT(msg);
}

void sendOkMessage() {
  wsClient.sendTXT("{\"action\":\"msg\",\"type\":\"status\",\"body\":\"ok\"}");
}

uint8_t toMode(const char *val) {
  if (strcmp(val, "output") == 0) {
    return OUTPUT;
  }

  if (strcmp(val, "input_pullup") == 0) {
    return INPUT_PULLUP;
  }

  return INPUT;
}

void handleMessage(uint8_t *payload) {
  StaticJsonDocument<JSON_DOC_SIZE> doc;

  DeserializationError error = deserializeJson(doc, payload);

  // Test if parsing succeeds.
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    sendErrorMessage(error.c_str());
    return;
  }

  if (!doc["type"].is<const char *>()) {
    sendErrorMessage("invalid message type format");
    return;
  }

  if (strcmp(doc["type"], "cmd") == 0) {
    if (!doc["body"].is<JsonObject>()) {
      sendErrorMessage("invalid command body");
      return;
    }

    if (strcmp(doc["body"]["type"], "pinMode") == 0) {

      // Uncomment this code for better validation of pinMode command

      if (!doc["body"]["mode"].is<const char *>()) {
        sendErrorMessage("invalid pinMode mode type");
        return;
      }

      if (strcmp(doc["body"]["mode"], "input") != 0 &&
          strcmp(doc["body"]["mode"], "input_pullup") != 0 &&
          strcmp(doc["body"]["mode"], "output") != 0) {
        sendErrorMessage("invalid pinMode mode value");
        return;
      }

      pinMode(doc["body"]["pin"], toMode(doc["body"]["mode"]));
      sendOkMessage();
      return;
    }

    if (strcmp(doc["body"]["type"], "digitalWrite") == 0) {
      //digitalWrite(doc["body"]["pin"], doc["body"]["value"]);
      if(doc["body"]["pin"] == 18) outgoingSetpoints.rele1 = doc["body"]["value"];
      if(doc["body"]["pin"] == 19) outgoingSetpoints.rele2 = doc["body"]["value"];
      if(doc["body"]["pin"] == 22) outgoingSetpoints.rele3 = doc["body"]["value"];

      // Send message via ESP-NOW
      outgoingSetpoints.msgType = DATA;
    esp_now_send(NULL, (uint8_t *) &outgoingSetpoints, sizeof(outgoingSetpoints));
      sendOkMessage();
      return;
    }

    if (strcmp(doc["body"]["type"], "digitalRead") == 0) {
      bool value;
      int pin = int(doc["body"]["pin"]);
      switch (pin) {
        case 18: value = rele1IncomingValue; break;
        case 19: value = rele2IncomingValue; break;
        case 22: value = rele3IncomingValue; break;
      }
      char msg[MSG_SIZE];

      outgoingSetpoints.msgType = DATA;
      esp_now_send(NULL, (uint8_t *) &outgoingSetpoints, sizeof(outgoingSetpoints));


      sprintf(msg, "{\"action\":\"msg\",\"type\":\"output\",\"body\":{\"pin\": %d, \"value\": %d}}",
              pin, value);

      wsClient.sendTXT(msg);
      return;
    }
    sendErrorMessage("unsupported command type");
    return;
  }

  sendErrorMessage("unsupported message type");
  return;
}

void onWSEvent(WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
  case WStype_CONNECTED:
    Serial.println("WS Connected");
    break;
  case WStype_DISCONNECTED:
    Serial.println("WS Disconnected");
    break;
  case WStype_TEXT:
    Serial.printf("WS Message: %s\n", payload);

    handleMessage(payload);

    break;
  }
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  Serial.print(len);
  Serial.print(" bytes of data received from : ");
  printMAC(mac_addr);
  Serial.println();
  StaticJsonDocument<1000> root;
  String payload;
  uint8_t type = incomingData[0];       // first message byte is the type of message 
  switch (type) {
  case DATA :                           // the message is data type
    memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
    // create a JSON document with received data and send it by event to the web page
Serial.print("Bytes received: ");
  Serial.println(len);
  incomingTemp = incomingReadings.temp;
  incomingHum = incomingReadings.hum;
  incomingisGasLeak = incomingReadings.GasLeak;
  incomingisMotionDetected = incomingReadings.isMotionDetected;
  incomingDistanceSensorValue = incomingReadings.distanceSensorValue;
  rele1IncomingValue = incomingReadings.rele1OutgoingValue;
  rele2IncomingValue = incomingReadings.rele2OutgoingValue;
  rele3IncomingValue = incomingReadings.rele3OutgoingValue;

  Serial.println("temp = " + String(incomingTemp));
  Serial.println("hum = " + String(incomingHum));
  Serial.println("incomingisGasLeak = " + String(incomingisGasLeak));
  Serial.println("incomingisMotionDetected = " + String(incomingisMotionDetected));
  Serial.println("incomingDistanceSensorValue = " + String(incomingDistanceSensorValue));
  Serial.println("incomingDistanceSensorValue = " + String(incomingDistanceSensorValue));

  Serial.println(" rele1IncomingValue = " + String(rele1IncomingValue));
  Serial.println(" rele2IncomingValue = " + String(rele2IncomingValue));
  Serial.println(" rele3IncomingValue = " + String(rele3IncomingValue));

  sendSensorDataToClient(incomingisMotionDetected, incomingTemp, incomingHum, incomingisGasLeak, incomingDistanceSensorValue);

    break;
  
  case PAIRING:                            // the message is a pairing request 
    memcpy(&pairingData, incomingData, sizeof(pairingData));
    Serial.println(pairingData.msgType);
    Serial.println(pairingData.id);
    Serial.print("Pairing request from: ");
    printMAC(mac_addr);
    Serial.println();
    Serial.println(pairingData.channel);
    if (pairingData.id > 0) {     // do not replay to server itself
      if (pairingData.msgType == PAIRING) { 
        pairingData.id = 0;       // 0 is server
        // Server is in AP_STA mode: peers need to send data to server soft AP MAC address 
        WiFi.softAPmacAddress(pairingData.macAddr);   
        pairingData.channel = chan;
        Serial.println("send response");
        esp_err_t result = esp_now_send(mac_addr, (uint8_t *) &pairingData, sizeof(pairingData));
        addPeer(mac_addr);
      }  
    }  
    break; 
  }
}

void initESP_NOW(){
    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
} 


void setup() {
  // Initialize Serial Monitor
  Serial.begin(921600);
  Serial.print("Server MAC Address:  ");
  Serial.println(WiFi.macAddress());

  // Set the device as a Station and Soft Access Point simultaneously
  WiFi.mode(WIFI_AP_STA);
  // Set device as a Wi-Fi Station
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Setting as a Wi-Fi Station..");
  }

  Serial.print("Server SOFT AP MAC Address:  ");
  Serial.println(WiFi.softAPmacAddress());

  chan = WiFi.channel();
  Serial.print("Station IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Wi-Fi Channel: ");
  Serial.println(WiFi.channel());

  initESP_NOW();
  
    Serial.println("Connected");
  wsClient.beginSSL(WS_HOST, WS_PORT, WS_URL, "", "wss"); 
  wsClient.onEvent(onWSEvent); 

}

void loop() {

  wsClient.loop();

}