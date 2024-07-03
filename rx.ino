#include <esp_now.h>
#include <WiFi.h>
#include "HardwareSerial.h"
#include "esp_wifi.h"

// Structure example to receive data
// Must match the sender structure
typedef struct data {
    int valueX1 = 0; // to store the X-axis value
    int valueY1 = 0; // to store the Y-axis value
    int valueX2 = 0; // to store the X-axis value
    int valueY2 = 0; // to store the Y-axis value
    int valueSPEED = 0; 
    int Bt1 = 0;
    int Bt2 = 0;
    int Bt3 = 0;
    int Bt4 = 0;
    int Bt5 = 0;
    int Bt6 = 0;
} data;

// Create a struct_message called myData
data DS;

struct senddata {
  int valueX1;
  int valueY1;
  int valueX2;
  int valueY2;
  int valueSPEED;
  int Bt1;
  int Bt2;
  int Bt3;
  int Bt4;
  int Bt5;
  int Bt6;
};
senddata SD;
// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&DS, incomingData, sizeof(DS));
  
  SD.valueX1 = DS.valueX1;
  SD.valueY1 = DS.valueY1;
  SD.valueX2 = DS.valueX2;
  SD.valueY2 = DS.valueY2;
  SD.valueSPEED = DS.valueSPEED;
  SD.Bt1 = DS.Bt1;
  SD.Bt2 = DS.Bt2;
  SD.Bt3 = DS.Bt3;
  SD.Bt4 = DS.Bt4;
  SD.Bt5 = DS.Bt5;
  SD.Bt6 = DS.Bt6;
  Serial2.write((uint8_t*)&SD, sizeof(SD));
  Serial.print("valueX1: "); Serial.println(SD.valueX1);
    Serial.print("valueY1: "); Serial.println(SD.valueY1);
    Serial.print("valueX2: "); Serial.println(SD.valueX2);
    Serial.print("valueY2: "); Serial.println(SD.valueY2);
    Serial.print("valueSPEED: "); Serial.println(SD.valueSPEED);
    Serial.print("Bt1: "); Serial.println(SD.Bt1);
    Serial.print("Bt2: "); Serial.println(SD.Bt2);
    Serial.print("Bt3: "); Serial.println(SD.Bt3);
    Serial.print("Bt4: "); Serial.println(SD.Bt4);
    Serial.print("Bt5: "); Serial.println(SD.Bt5);
    Serial.print("Bt6: "); Serial.println(SD.Bt6);
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.STA.begin();
  uint8_t newMACAddress[] = {0x02, 0x3E, 0x5D, 0x1B, 0x39, 0xC2}; // Địa chỉ MAC tùy chỉnh
  esp_err_t result = esp_wifi_set_mac(WIFI_IF_STA, newMACAddress);
  Serial.println(result);

  if (result == ESP_OK) {
    Serial.println("Địa chỉ MAC đã được thiết lập thành công.");
  } else {
    Serial.println("Thiết lập địa chỉ MAC thất bại.");
  }  
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {
  
}