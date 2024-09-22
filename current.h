int currentPin = A6; // 電流傳感器連接到模擬輸入引腳
float current_mA;

void current() {

  int current = analogRead(currentPin);
  float voltage = current * (5.0 / 1023.0);
  current_mA = (voltage - 2.5) * 1000; // 轉換為毫安

  Serial.print("Current: ");
  Serial.print(current_mA);
  Serial.println(" mA");


}
