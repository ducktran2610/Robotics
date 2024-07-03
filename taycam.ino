/*
  Rui Santos & Sara Santos - Random Nerd Tutorials
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*/
#include <esp_now.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>

#define VRX1_PIN  39
#define VRY1_PIN  36
#define VRX2_PIN  32
#define VRY2_PIN  35
#define SPEED_PIN  34
#define B1_PIN 27
#define B2_PIN 26
#define B3_PIN 14
#define B4_PIN 5
#define B5_PIN 19
#define B6_PIN 18

int lcdColumns = 16;
int lcdRows = 2;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows); 

int valueX1 = 0; // to store the X-axis value
int valueY1 = 0; // to store the Y-axis value
int valueX2 = 0; // to store the X-axis value
int valueY2 = 0; // to store the Y-axis value
int valueSPEED = 0; 
// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0x02, 0x3E, 0x5D, 0x1B, 0x39, 0xC2};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
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
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
    if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Sent with success");
    lcd.setCursor(4, 0);
    lcd.print("   ");
    lcd.setCursor(4, 0);
    lcd.print("ON");
  }
  else {
    Serial.println("Error sending the data");
    lcd.setCursor(4, 0);
    lcd.print("   ");
    lcd.setCursor(4, 0);
    lcd.print("OFF");
  }
}
 
void setup() {
  Serial.begin(115200) ;
  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();
  lcd.setCursor(0, 0);
  // print message
  lcd.print("Peroctopiece RC");
  lcd.setCursor(0, 1);
  lcd.print("FPT HIGHSCHOOL");
  delay(1500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Loading...");
  pinMode(B1_PIN, INPUT_PULLUP);
  pinMode(B2_PIN, INPUT_PULLUP);
  pinMode(B3_PIN, INPUT_PULLUP);
  pinMode(B4_PIN, INPUT_PULLUP);
  pinMode(B5_PIN, INPUT_PULLUP);
  pinMode(B6_PIN, INPUT_PULLUP);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Error ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CONECTING EROR");
    return;
  };
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Starting successfully");
  delay(1000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Stt:");
  lcd.setCursor(8, 0);
  lcd.print("Sp:");
}
 
void loop() {
  // Set values to send
  myData.Bt1 = digitalRead(B1_PIN);
  myData.Bt2 = digitalRead(B2_PIN);
  myData.Bt3 = digitalRead(B3_PIN);
  myData.Bt4 = digitalRead(B4_PIN);
  myData.Bt5 = digitalRead(B5_PIN);
  myData.Bt6 = digitalRead(B6_PIN);

  valueX1 = (analogRead(VRX1_PIN)*2000)/4095-1000;
  valueY1 = (analogRead(VRY1_PIN)*2000)/4095-1000;
  if ((-200 < valueX1) and (valueX1 < 200)) valueX1 = 0;
  if ((-200 < valueY1) and (valueY1 < 200)) valueY1 = 0;
  myData.valueX1 = valueX1/-1000;
  myData.valueY1 = valueY1/-1000;

  valueX2 = (analogRead(VRX2_PIN)*2000)/4095-1000;
  valueY2 = (analogRead(VRY2_PIN)*2000)/4095-1000;
  if ((-200 < valueX2) and (valueX2 < 200)) valueX2 = 0;
  if ((-200 < valueY2) and (valueY2 < 200)) valueY2 = 0;
  myData.valueX2 = valueX2/-1000;
  myData.valueY2 = valueY2/-1000;

  myData.valueSPEED = (analogRead(SPEED_PIN)*100)/4095;

  Serial.print(myData.valueX1);
  Serial.print(myData.valueY1);
  Serial.print(myData.valueX2);
  Serial.print(myData.valueY2);
  Serial.println(myData.valueSPEED);
  lcd.setCursor(11, 0);
  lcd.print("   ");
  lcd.setCursor(11, 0);
  lcd.print(myData.valueSPEED);
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

  delay(50);
}