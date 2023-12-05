#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

const char* ssid = "ALL LUMI";
const char* password = "lumivn274";

esp_now_peer_info_t slave;
int chan;

enum MessageType {
  PAIRING,
  DATA,
};
MessageType messageType;

int counter = 0;

typedef struct struct_message{
  uint8_t msgtType;
  uint8_t id;
  unsigned int readingId;
}struct_message;

typedef struct struct_pairing{
  uint8_t msgType;
  uint8_t id;
  uint8_t macAddr[6];
  uint8_t channel;
}struct_pairing;

struct_message incomingReadings;
struct_message outgoingSetpoints;
struct_pairing pairingData;

void readDataToSend();
void printMAC(const uint8_t * mac_addr);
bool addPeer(const uint8_t * peer_addr);
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const uint8_t * mac_addr, const uint8_t * incomingData, int len);
void initESP_NOW();
void initWiFi();

void setup()
{
  Serial.begin(115200);
  initWiFi();
  initESP_NOW();
}

void loop()
{
  static unsigned long lastEventTime = millis();
  static const unsigned long EVENT_INTERVAL_MS = 500;
  if((millis() - lastEventTime) > EVENT_INTERVAL_MS){
    lastEventTime = millis();
    readDataToSend();
    esp_now_send(NULL, (uint8_t *) &outgoingSetpoints, sizeof(outgoingSetpoints));
  }
}

void readDataToSend(){
  outgoingSetpoints.msgtType = DATA;
  outgoingSetpoints.id = 0;
  outgoingSetpoints.readingId = counter++;
}

void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

bool addPeer(const uint8_t * peer_addr){ //add pairing
  memset(&slave, 0, sizeof(slave));
  const esp_now_peer_info_t *peer = &slave;
  memcpy(slave.peer_addr, peer_addr, 6);

  slave.channel = chan; //pick a channel
  slave.encrypt = 0; // no encryption
  //check if the peer exists
  bool exists = esp_now_is_peer_exist(slave.peer_addr);
  if(exists){
    //slave already paired
    Serial.println("Already paired");
    return true;
  }
  else{
    esp_err_t addStatus = esp_now_add_peer(peer);
    if(addStatus == ESP_OK){
      //Pair success
      Serial.println("Pair success");
      return true;
    }
    else{
      Serial.println("Pair failed");
      return false;
    }
  }
}

//callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  Serial.print("Last Packet send status: ");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success to " : " Delevery Fail to ");
  printMAC(mac_addr);
  Serial.println();
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t * incomingData, int len){
  Serial.print(len);
  Serial.print(" bytes of data received from : ");
  printMAC(mac_addr);
  Serial.println();
  String payload;
  uint8_t type = incomingData[0];   //first message byte is the type of message
  switch(type){
    case DATA:
      memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
    break;

    case PAIRING:
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
  //init esp_now
  if(esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}

void initWiFi(){
  Serial.print("Server MAC Address: ");
  Serial.println(WiFi.macAddress());

  //Set the device as a stattion and soft access point simultanesouly
  WiFi.mode(WIFI_AP_STA);
  //set device as a WiFi station
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED){
    delay(1000);
    Serial.println("Setting as a Wi-Fi station ...");
  }

  Serial1.print("server SOFT AP MAC Address: ");
  Serial.println(WiFi.softAPmacAddress());

  chan = WiFi.channel();
  Serial.print("Station IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("WiFi channel: ");
  Serial.println(WiFi.channel());
}