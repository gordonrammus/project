#include <Ultrasonic.h>

Ultrasonic ultrasonic1(54, 55);
Ultrasonic ultrasonic2(56, 57);
Ultrasonic ultrasonic3(58, 59);

const int numReadings = 20;
long readings1[numReadings];
long readings2[numReadings];
long readings3[numReadings];

int readIndex1 = 0;
int readIndex2 = 0;
int readIndex3 = 0;
long total1 = 0;
long total2 = 0;
long total3 = 0;
long average1 = 0;
long average2 = 0;
long average3 = 0;

long distance1;
long distance2;
long distance3;

void distance() {

  distance1 = ultrasonic1.read();  //左
  distance2 = ultrasonic2.read();  //中
  distance3 = ultrasonic3.read();  //右

  
  total1 = total1 - readings1[readIndex1];
  total2 = total2 - readings2[readIndex2];
  total3 = total3 - readings3[readIndex3];
  
  readings1[readIndex1] = distance1;
  readings2[readIndex2] = distance2;
  readings3[readIndex3] = distance3;
  
  total1 = total1 + readings1[readIndex1];
  total2 = total2 + readings2[readIndex2];
  total3 = total3 + readings3[readIndex3];
  
  readIndex1 = readIndex1 + 1;
  readIndex2 = readIndex2 + 1;
  readIndex3 = readIndex3 + 1;
  
  if (readIndex1 >= numReadings) readIndex1 = 0;
  if (readIndex2 >= numReadings) readIndex2 = 0;
  if (readIndex3 >= numReadings) readIndex3 = 0;

  average1 = total1 / numReadings;
  average2 = total2 / numReadings;
  average3 = total3 / numReadings;
/*
  Serial.print(" LEFT Distance : ");
  Serial.print(average1);
  Serial.print("       MIDDLE Distance : ");
  Serial.print(average2);
  Serial.print("       RIGHT Distance : ");
  Serial.println(average3);
  
  delay(100);  
*/

}
