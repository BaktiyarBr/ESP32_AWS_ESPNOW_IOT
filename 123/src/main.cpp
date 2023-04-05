#define MOTION_SENSOR_PIN 34
#define GAS_SENSOR_PIN 35
#define DHT22_PIN 32
#define RELE1_PIN 27
#define RELE2_PIN 23
#define RELE3_PIN 25

#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <EEPROM.h>

#include <esp_wifi.h>
#include "DHTesp.h"
                DHTesp dht;

#include <NewPing.h>
NewPing sonar(14, 12, 400);
float dist_3[3] = {0.0, 0.0, 0.0};
float middle, dist, dist_filtered;
float k;
unsigned long sensTimer;

byte i, delta;

int incomingisGasLeak;
bool incomingisMotionDetected, Rele1, Rele2, Rele3;
float temperature, humidity, incomingTemp, incomingHum;
String success;

// Set your Board and Server ID
#define BOARD_ID 1
#define MAX_CHANNEL 13 // for North America // 13 in Europe

uint8_t serverAddress[] = {0xC8, 0xF0, 0x9E, 0x52, 0xB3, 0xB8};

// Structure example to receive data
// Must match the receiver structure
typedef struct struct_message
{
  uint8_t msgType;
  uint8_t id;
  bool rele1, rele2, rele3, rele1OutgoingValue, rele2OutgoingValue, rele3OutgoingValue, isMotionDetected;
  float temp, hum, distanceSensorValue;
  int GasLeak;
  unsigned int readingId;
} struct_message;

typedef struct struct_pairing
{ // new structure for pairing
  uint8_t msgType;
  uint8_t id;
  uint8_t macAddr[6];
  uint8_t channel;
} struct_pairing;

// Create 2 struct_message
struct_message myData; // data to send
struct_message inData; // data received
struct_pairing pairingData;

enum PairingStatus
{
  NOT_PAIRED,
  PAIR_REQUEST,
  PAIR_REQUESTED,
  PAIR_PAIRED,
};
PairingStatus pairingStatus = NOT_PAIRED;

enum MessageType
{
  PAIRING,
  DATA,
};
MessageType messageType;

#ifdef SAVE_CHANNEL
int lastChannel;
#endif
int channel = 1;

unsigned long currentMillis = millis();
unsigned long previousMillis = 0; // Stores last time temperature was published
const long interval = 10000;      // Interval at which to publish sensor readings
unsigned long start;              // used to measure Pairing time
unsigned int readingId = 0;

void addPeer(const uint8_t *mac_addr, uint8_t chan)
{
  esp_now_peer_info_t peer;
  // ESP_ERROR_CHECK(esp_wifi_set_channel(chan ,WIFI_SECOND_CHAN_NONE));
  esp_now_del_peer(mac_addr);
  memset(&peer, 0, sizeof(esp_now_peer_info_t));
  peer.channel = chan;
  peer.encrypt = false;
  memcpy(peer.peer_addr, mac_addr, sizeof(uint8_t[6]));
  if (esp_now_add_peer(&peer) != ESP_OK)
  {
    Serial.println("Failed to add peer");
    return;
  }
  memcpy(serverAddress, mac_addr, sizeof(uint8_t[6]));
}

void printMAC(const uint8_t *mac_addr)
{
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  Serial.print("Packet received from: ");
  printMAC(mac_addr);
  Serial.println();
  Serial.print("data size = ");
  Serial.println(sizeof(incomingData));
  uint8_t type = incomingData[0];
  switch (type)
  {
  case DATA: // we received data from server
    memcpy(&inData, incomingData, sizeof(inData));
    digitalWrite(RELE1_PIN, inData.rele1);
    digitalWrite(RELE2_PIN, inData.rele2);
    digitalWrite(RELE3_PIN, inData.rele3);
    Serial.println(inData.rele1);
    Serial.println(inData.rele2);
    Serial.println(inData.rele3);

    break;

  case PAIRING: // we received pairing data from server
    memcpy(&pairingData, incomingData, sizeof(pairingData));
    if (pairingData.id == 0)
    { // the message comes from server
      printMAC(mac_addr);
      Serial.print("Pairing done for ");
      printMAC(pairingData.macAddr);
      Serial.print(" on channel ");
      Serial.print(pairingData.channel); // channel used by the server
      Serial.print(" in ");
      Serial.print(millis() - start);
      Serial.println("ms");
      addPeer(pairingData.macAddr, pairingData.channel); // add the server  to the peer list
#ifdef SAVE_CHANNEL
      lastChannel = pairingData.channel;
      EEPROM.write(0, pairingData.channel);
      EEPROM.commit();
#endif
      pairingStatus = PAIR_PAIRED; // set the pairing status
    }
    break;
  }
}

PairingStatus autoPairing()
{
  switch (pairingStatus)
  {
  case PAIR_REQUEST:
    Serial.print("Pairing request on channel ");
    Serial.println(channel);

    // set WiFi channel
    ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
    if (esp_now_init() != ESP_OK)
    {
      Serial.println("Error initializing ESP-NOW");
    }

    // set callback routines
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // set pairing data to send to the server
    pairingData.msgType = PAIRING;
    pairingData.id = BOARD_ID;
    pairingData.channel = channel;

    // add peer and send request
    addPeer(serverAddress, channel);
    esp_now_send(serverAddress, (uint8_t *)&pairingData, sizeof(pairingData));
    previousMillis = millis();
    pairingStatus = PAIR_REQUESTED;
    break;

  case PAIR_REQUESTED:
    // time out to allow receiving response from server
    currentMillis = millis();
    if (currentMillis - previousMillis > 250)
    {
      previousMillis = currentMillis;
      // time out expired,  try next channel
      channel++;
      if (channel > MAX_CHANNEL)
      {
        channel = 1;
      }
      pairingStatus = PAIR_REQUEST;
    }
    break;

  case PAIR_PAIRED:
    // nothing to do here
    break;
  }
  return pairingStatus;
}

// медианный фильтр из 3ёх значений
float middle_of_3(float a, float b, float c)
{
  if ((a <= b) && (a <= c))
  {
    middle = (b <= c) ? b : c;
  }
  else
  {
    if ((b <= a) && (b <= c))
    {
      middle = (a <= c) ? a : c;
    }
    else
    {
      middle = (a <= b) ? a : b;
    }
  }
  return middle;
}

void setup()
{
  Serial.begin(921600);
  pinMode(MOTION_SENSOR_PIN, INPUT);
  pinMode(RELE1_PIN, OUTPUT);
  pinMode(RELE2_PIN, OUTPUT);
  pinMode(RELE3_PIN, OUTPUT);
  dht.setup(DHT22_PIN, DHTesp::DHT22); // Connect DHT sensor to GPIO 17

  Serial.print("Client Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  start = millis();

#ifdef SAVE_CHANNEL
  EEPROM.begin(10);
  lastChannel = EEPROM.read(0);
  Serial.println(lastChannel);
  if (lastChannel >= 1 && lastChannel <= MAX_CHANNEL)
  {
    channel = lastChannel;
  }
  Serial.println(channel);
#endif
  pairingStatus = PAIR_REQUEST;
  Serial.print("sizeof struct_message = ");
  Serial.println(sizeof(struct_message));
}

void loop()
{
  if (autoPairing() == PAIR_PAIRED)
  {
    unsigned long currentMillis = millis();
    if (millis() - sensTimer > 500)
    { // измерение и вывод каждые 50 мс
      // счётчик от 0 до 2
      // каждую итерацию таймера i последовательно принимает значения 0, 1, 2, и так по кругу
      if (i > 1)
        i = 0;
      else
        i++;
      dist_3[i] = (float)sonar.ping() / 57.5;              // получить расстояние в текущую ячейку массива
      dist = middle_of_3(dist_3[0], dist_3[1], dist_3[2]); // фильтровать медианным фильтром из 3ёх последних измерений
      delta = abs(dist_filtered - dist);                   // расчёт изменения с предыдущим
      if (delta > 1)
        k = 0.7; // если большое - резкий коэффициент
      else
        k = 0.1;                                          // если маленькое - плавный коэффициент
      dist_filtered = dist * k + dist_filtered * (1 - k); // фильтр "бегущее среднее"
      sensTimer = millis();                               // сбросить таймер
    }

    if (currentMillis - previousMillis >= interval)
    {
      // Save the last time a new reading was published
      previousMillis = currentMillis;
      // Set values to send
      myData.isMotionDetected = digitalRead(MOTION_SENSOR_PIN);
      myData.temp = dht.getTemperature();
      myData.hum = dht.getHumidity();
      myData.GasLeak = analogRead(GAS_SENSOR_PIN);
      myData.distanceSensorValue = dist_filtered;
      // Set values to send
      myData.msgType = DATA;
      myData.id = 2;
      myData.readingId = readingId++;
      myData.rele1OutgoingValue = digitalRead(RELE1_PIN);
      myData.rele2OutgoingValue = digitalRead(RELE2_PIN);
      myData.rele3OutgoingValue = digitalRead(RELE3_PIN);

      esp_err_t result = esp_now_send(serverAddress, (uint8_t*) &myData, sizeof(myData));
    }
  }
}