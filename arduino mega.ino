struct data {
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

void setup() {
  Serial.begin(115200);   // Mở Serial để giao tiếp với máy tính
  Serial2.begin(115200);  // Mở Serial1 để nhận dữ liệu từ ESP32
  
}
data SD;
void loop() {
  if (Serial2.available() >= 0) { // Kiểm tra xem có đủ dữ liệu để đọc cấu trúc Data không
    
    Serial2.readBytes((uint8_t*)&SD, sizeof(SD)); // Đọc dữ liệu vào cấu trúc

    // Hiển thị dữ liệu nhận được
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
}
