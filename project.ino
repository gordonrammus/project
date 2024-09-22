#include <Wire.h>
#include <Servo.h>
#include "imu.h"
#include "distance.h"
#include "current.h"

Servo thigh_left;
Servo calf_left;
Servo thigh_right;
Servo calf_right;
Servo kuan_left;
Servo kuan_right;
Servo shoulder_left;
Servo arm_left;
Servo shoulder_right;
Servo arm_right;
Servo forearm_right;
Servo forearm_left;
Servo core;

bool standbit = 0;
int state = 0;
int step = 1;
int steptime = 100;
unsigned long currentTime;
unsigned long startTime;  
unsigned long elapsedTime;  

void Balance();
void Balance_left();
void Balance_right();

void Step1();
void Step2();
void Step3();
void Step4();
void Step5();
void Step6();
void Step7();
void Step8();
void Step9();
void Step10();
void Fixed();
void default_state();
void ToDefault1();
void ToDefault2();

void TurnLeft();
void turnleft1();
void turnleft2();
void turnleft3();
void To_left1();
void To_left2();

void TurnRight();
void turnright1();
void turnright2();
void turnright3();
void To_right1();
void To_right2();
void stand();
void StandToBalance();
void StandToBalanceLeft();
void StandToBalanceRight();

void Display();
void Adjust_high();
void Adjust_low();

void setup() {
  Wire.begin();
  Serial.begin(115200);


  arm_right.attach(12);   
  arm_left.attach(10);
  forearm_right.attach(11);
  forearm_left.attach(9);
  shoulder_right.attach(8);
  shoulder_left.attach(7);
  core.attach(6);
  thigh_right.attach(5); 
  calf_right.attach(4);
  thigh_left.attach(3);
  calf_left.attach(2);
  kuan_right.attach(44); 
  kuan_left.attach(46);  

  default_state();

  setup_mpu_6050_registers();
  calibrate_gyro();

  Serial.println("Calibration complete. Starting main loop.");
  loopTimer = micros();
  loopTimer2 = micros();
  
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings1[thisReading] = 0;
    readings2[thisReading] = 0;
    readings3[thisReading] = 0;
  }

  for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      distance();
    }
  }
  
}

void loop() {

  Display();
  ///////////////////////////////////////////////////////////////////forward//////////////////////////////////////
  if(distance1 >= 50 && distance2 >= 40 && distance3 >= 50 ){
    IMUdata();
    ///////////////////////////////////////balance
    if (roll < -4){
      Display();
      step = 0;
      if (standbit){
        StandToBalance();
      }
      standbit = 0;
      Serial.println("balance");
      while( roll <= -5 || filteredRoll >= 5 || filteredRoll < -5 ){
      Balance();
      IMUdata();
      //distance();
      } 
    }
    ///////////////////////////////////////ride
    else{
      standbit = 0;
      if (step == -1){
        TurnLeft();
        stand();
        step++;
      }
      else if (step == 0){
        ToDefault1();
        ToDefault2();
        distance();
        step++;
      }
      else if (step == 1){
        if (roll < 3.5){
          Adjust_high();
          Serial.println("Adjust High");
          Step1();
          step++;
        }
        else if (roll > 5.5){
          Adjust_low();
          Serial.println("Adjust Low");
          Step1();
          step++;
        }
        else{
          Step1();
          step++;
        }
      }
      else if (step == 2){
        Step2();
        step++;
      }
      else if (step == 3){
        Step3();
        step++;
      }
      else if (step == 4){
        Step4();
        step++;
      }
      else if (step == 5){
        Step5();
        step++;
      }
      else if (step == 6){
        Step6();
        step++;
      }
      else if (step == 7){
        Step7();
        step++;
      }
      else if (step == 8){
        Step8();
        step++;
      }
      else if (step == 9){
        Step9();
        step++;
      }
      else if (step == 10){
        Step10();
        if (roll < 0){
          step = 1;
        }
        else{
          step++;
        }
        
      }
      else if (step == 11){
        step = 1;
        stand();

        
      }
    }
  }
    ///////////////////////////////////////////////////////////////turn/////////////////////////////////////
  else{
    if (distance1 < 50 && distance3 > 50){
      if (step == -1){
        stand();
      }
      step = 0;
      TurnRight();
    }
    else{
      step = -1;//左轉結束
      TurnLeft();
    }
    distance();
  }
}

   
//------------------------------------------------------------MOVEMENT-------------------------------------------------------------------------------------
void default_state(){
  kuan_left.write(62);
  kuan_right.write(100);

  shoulder_left.write(140);
  shoulder_right.write(80);

  arm_left.write(90);
  arm_right.write(90);

  forearm_left.write(140);
  forearm_right.write(60);

  thigh_right.write(135);//右腿蹲
  calf_right.write(69);

  thigh_left.write(51);
  calf_left.write(80);

  core.write(80);
}

void Fixed(){
  kuan_left.write(62);
  kuan_right.write(100);
  shoulder_left.write(140);
  shoulder_right.write(80);
  forearm_left.write(140);
  forearm_right.write(60);
  arm_left.write(90);
  arm_right.write(90);
  thigh_right.write(135);//右腿蹲
  core.write(80);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Step1(){
  Fixed();
  calf_left.write(80);
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      //distance();
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int thigh_left_current = map(elapsedTime, 0, steptime, 51, 63); //53->133    +8
    //int calf_left_current = map(elapsedTime, 0, steptime, 82, 77); //82->42     -4
    int calf_right_current = map(elapsedTime, 0, steptime, 69, 73);  // 60->80  +2
    thigh_left.write(thigh_left_current);
    //calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
  }
}
void Step2(){
  Fixed();
  calf_left.write(80);
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      //distance();
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int thigh_left_current = map(elapsedTime, 0, steptime, 63, 73); //53->133    +8
    //int calf_left_current = map(elapsedTime, 0, steptime, 77, 72); //82->42     -5
    int calf_right_current = map(elapsedTime, 0, steptime, 73, 77);  // 60->80  +2
    thigh_left.write(thigh_left_current);
    //calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
  }
}
void Step3(){
  Fixed();
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      //distance();
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int thigh_left_current = map(elapsedTime, 0, steptime, 73, 83); //53->133    +8
    int calf_left_current = map(elapsedTime, 0, steptime, 80, 80); //82->42     -4
    int calf_right_current = map(elapsedTime, 0, steptime, 77, 81);  // 60->80  +2
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
  }
}
void Step4(){ 
  Fixed();
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      //distance();
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int thigh_left_current = map(elapsedTime, 0, steptime, 83, 93); //53->133    +8
    int calf_left_current = map(elapsedTime, 0, steptime, 80, 72); //82->42     -5
    int calf_right_current = map(elapsedTime, 0, steptime, 81, 83);  // 60->80  +2
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
  }
}
void Step5(){
  Fixed();
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      //distance();
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int thigh_left_current = map(elapsedTime, 0, steptime, 93, 103); //53->133    +8
    int calf_left_current = map(elapsedTime, 0, steptime, 72, 67); //82->42     -4
    int calf_right_current = map(elapsedTime, 0, steptime, 83, 85);  // 60->80  +2
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
  }
}
void Step6(){ 
  Fixed();
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      //distance();
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int thigh_left_current = map(elapsedTime, 0, steptime, 103, 113); //53->133    +8
    int calf_left_current = map(elapsedTime, 0, steptime, 67, 62); //82->42     -4
    int calf_right_current = map(elapsedTime, 0, steptime, 85, 87);  // 60->80  +2
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
  }
}
void Step7(){
  Fixed();
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      //distance();
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int thigh_left_current = map(elapsedTime, 0, steptime, 113, 123); //53->133    +8
    int calf_left_current = map(elapsedTime, 0, steptime, 62, 57); //82->42     -4
    int calf_right_current = map(elapsedTime, 0, steptime, 87, 89);  // 60->80  +2
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
  }
}
void Step8(){
  Fixed();
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      //distance();
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int thigh_left_current = map(elapsedTime, 0, steptime, 123, 133); //53->133    +8
    int calf_left_current = map(elapsedTime, 0, steptime, 57, 52); //82->42     -4
    int calf_right_current = map(elapsedTime, 0, steptime, 89, 91);  // 60->80  +2
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
  }
}
void Step9(){
  Fixed();
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      //distance();
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int thigh_left_current = map(elapsedTime, 0, steptime, 133, 143); //53->133    +8
    int calf_left_current = map(elapsedTime, 0, steptime, 52, 47); //82->42     -4
    int calf_right_current = map(elapsedTime, 0, steptime, 91, 93);  // 60->80  +2
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
  }
}
void Step10(){ 
  Fixed();
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      //distance();
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int thigh_left_current = map(elapsedTime, 0, steptime, 143, 153); //53->133    +8
    int calf_left_current = map(elapsedTime, 0, steptime, 47, 42); //82->42     -4
    int calf_right_current = map(elapsedTime, 0, steptime, 93, 95);  // 60->80  +2
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////balance
void Balance(){  
  
  kuan_left.write(55);
  kuan_right.write(107);

  shoulder_left.write(140);
  shoulder_right.write(80);

  arm_left.write(90);
  arm_right.write(90);

  forearm_left.write(130);
  forearm_right.write(70);

  thigh_right.write(135);//右腿蹲
  calf_right.write(86);

  thigh_left.write(55);
  calf_left.write(78);

  core.write(80);
}
void Balance_left(){
  
  kuan_left.write(55);
  kuan_right.write(107);

  shoulder_left.write(140);
  shoulder_right.write(80);

  arm_left.write(90);
  arm_right.write(90);

  forearm_left.write(130);
  forearm_right.write(70);

  thigh_right.write(135);//右腿蹲
  calf_right.write(87);

  thigh_left.write(56);
  calf_left.write(80);

  core.write(80);
}
void Balance_right(){
  
  kuan_left.write(60);
  kuan_right.write(107);
  shoulder_left.write(140);
  shoulder_right.write(80);

  arm_left.write(90);
  arm_right.write(90);

  forearm_left.write(130);
  forearm_right.write(70);

  thigh_right.write(135);//右腿蹲
  calf_right.write(87);

  thigh_left.write(56);
  calf_left.write(80);

  core.write(80);
}

void ToDefault1(){
  kuan_left.write(55);
  kuan_right.write(107);

  shoulder_left.write(140);
  shoulder_right.write(80);

  arm_left.write(90);
  arm_right.write(90);

  thigh_right.write(135);


  core.write(80);
 //用力往下踩//////////////////////////////////////////////////////
  int balancetime = 800; 
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= balancetime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    //int kuan_left_current = map(elapsedTime, 0, balancetime, 57, 57); // 80->67  (20/5 = -4)

    int forearm_left_current = map(elapsedTime, 0, balancetime, 130, 140); // 50->70  (20/5 = -4)
    int forearm_right_current = map(elapsedTime, 0, balancetime, 70, 60); // 50->70  (20/5 = -4)
    int thigh_left_current = map(elapsedTime, 0, balancetime, 55, 55); //60->58    +20
    int calf_left_current = map(elapsedTime, 0, balancetime, 78, 72); //65->80     -10
    int calf_right_current = map(elapsedTime, 0, balancetime, 86, 69); //90->70     -10
    // Write the current angles to the servos
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
    forearm_left.write(forearm_left_current);
    forearm_right.write(forearm_right_current);
    //kuan_left.write(kuan_left_current);

  }
}
void ToDefault2(){
  //kuan_left.write(62);
  kuan_right.write(100);//90 數字- 往內 right kuan
  shoulder_left.write(140);//120 數字- 往上 left shoulder
  shoulder_right.write(80);//60 數字- 往下 right shoulder
  arm_left.write(90);
  arm_right.write(90);
  forearm_left.write(140);//90 數字+ 往上
  forearm_right.write(60);//90 數字+ 往下
  thigh_right.write(135);//60 數字- 後勾  right thigh
  core.write(80);//95 數字+ 往左傾
 //用力往下踩//////////////////////////////////////////////////////
  int partytime = 400; 
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= partytime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;
    int kuan_left_current = map(elapsedTime, 0, partytime, 55, 62); // 80->67  (20/5 = -4)
    int kuan_right_current = map(elapsedTime, 0, partytime, 107, 100); // 80->67  (20/5 = -4)
    int thigh_left_current = map(elapsedTime, 0, partytime, 55, 51); //90->70     -10
    int calf_left_current = map(elapsedTime, 0, partytime, 72, 80); //90->70     -10
    int calf_right_current = map(elapsedTime, 0, partytime, 69, 69); //90->70     -10
    // Write the current angles to the servos
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
    kuan_left.write(kuan_left_current);
    kuan_right.write(kuan_right_current);

  }

}
/////////////////////////////////////////////////////////////////////////////////////////////////turnleft
void TurnLeft(){
  /////////////////////////////////balance
  if (roll<=-5){
    
    if (standbit){
      StandToBalanceLeft();
    }
    Serial.println("");
    Serial.print("balance_left");
    Serial.println("");
    standbit = 0;
    while( roll <= -5 || filteredRoll >= 10 || filteredRoll < -10 ){
      Balance_left();
      IMUdata();
    }
    To_left1();
    To_left2();
  }
  ////////////////////////////////////ride
  else{
    standbit = 0;
    if (roll < 1){
      Serial.println("\nAdjust high");
      Adjust_high();
    }
    turnleft1();
    if (roll > -4){
      delay(300);
      turnleft2();
      if (roll > -4){
        delay(300);
        turnleft3();
      }
    }
  }
  
}
void turnleft1(){
  arm_left.write(90);
  arm_right.write(90);
  thigh_right.write(135);
  int steptime = 800;
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int kuan_left_current = map(elapsedTime, 0, steptime, 62, 62);
    int kuan_right_current = map(elapsedTime, 0, steptime, 100, 100);
    int shoulder_left_current = map(elapsedTime, 0, steptime, 140, 160);
    int shoulder_right_current = map(elapsedTime, 0, steptime, 80, 85);
    int forearm_left_current = map(elapsedTime, 0, steptime, 140, 160);
    int forearm_right_current = map(elapsedTime, 0, steptime, 60, 80);
    int thigh_left_current = map(elapsedTime, 0, steptime, 53, 80);
    int calf_left_current = map(elapsedTime, 0, steptime, 82, 60);
    int calf_right_current = map(elapsedTime, 0, steptime, 69, 75);
    int core_current = map(elapsedTime, 0, steptime, 80, 110);

    kuan_left.write(kuan_left_current);
    kuan_right.write(kuan_right_current);
    shoulder_left.write(shoulder_left_current);
    shoulder_right.write(shoulder_right_current);
    forearm_left.write(forearm_left_current);
    forearm_right.write(forearm_right_current);
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
    core.write(core_current);
  }
  Serial.print("left_1");
}
////default->balance
void turnleft2(){
  arm_left.write(90);
  arm_right.write(90);

  thigh_right.write(135);
  int steptime = 800;
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int kuan_left_current = map(elapsedTime, 0, steptime, 62, 55);
    int kuan_right_current = map(elapsedTime, 0, steptime, 100, 90);
    int shoulder_left_current = map(elapsedTime, 0, steptime, 160, 100);
    int shoulder_right_current = map(elapsedTime, 0, steptime, 85, 35);
    int forearm_left_current = map(elapsedTime, 0, steptime, 160, 95);
    int forearm_right_current = map(elapsedTime, 0, steptime, 80, 35);
    int thigh_left_current = map(elapsedTime, 0, steptime, 80, 53);
    int calf_left_current = map(elapsedTime, 0, steptime, 60, 65);
    int calf_right_current = map(elapsedTime, 0, steptime, 75, 80);
    int core_current = map(elapsedTime, 0, steptime, 110, 60);

    kuan_left.write(kuan_left_current);
    kuan_right.write(kuan_right_current);
    shoulder_left.write(shoulder_left_current);
    shoulder_right.write(shoulder_right_current);
    forearm_left.write(forearm_left_current);
    forearm_right.write(forearm_right_current);
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
    core.write(core_current);
  }
  Serial.print("left_2");
}
///balance->squat
void turnleft3(){
  
  //shoulder_left.write(120);//120 數字- 往上 left shoulder
  //shoulder_right.write(55);//60 數字- 往下 right shoulder
  arm_left.write(90);
  arm_right.write(90);
  //forearm_left.write(115);//90 數字+ 往上
  //forearm_right.write(55);//90 數字+ 往下
  //thigh_left.write(58);
  thigh_right.write(135);
  int steptime = 800;
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int kuan_left_current = map(elapsedTime, 0, steptime, 55, 62);
    int kuan_right_current = map(elapsedTime, 0, steptime, 90, 100);

    int shoulder_left_current = map(elapsedTime, 0, steptime, 100, 140);
    int shoulder_right_current = map(elapsedTime, 0, steptime, 35, 80);
    int forearm_left_current = map(elapsedTime, 0, steptime, 95, 140);
    int forearm_right_current = map(elapsedTime, 0, steptime, 35, 60);
    int thigh_left_current = map(elapsedTime, 0, steptime, 53, 53);
    int calf_left_current = map(elapsedTime, 0, steptime, 65, 82);
    int calf_right_current = map(elapsedTime, 0, steptime, 80, 69);
    int core_current = map(elapsedTime, 0, steptime, 60, 80);

    kuan_left.write(kuan_left_current);
    kuan_right.write(kuan_right_current);
    shoulder_left.write(shoulder_left_current);
    shoulder_right.write(shoulder_right_current);
    forearm_left.write(forearm_left_current);
    forearm_right.write(forearm_right_current);
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
    core.write(core_current);
  }
  Serial.println("left_3");
}
void To_left1(){
  Serial.println("To left 1");
  kuan_left.write(55);
  kuan_right.write(107);//90 數字- 往內 right kuan
  arm_left.write(90);
  arm_right.write(90);

  thigh_right.write(135);
  core.write(80);

 //用力往下踩//////////////////////////////////////////////////////
  int balancetime = 400; 
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= balancetime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    //int kuan_left_current = map(elapsedTime, 0, balancetime, 55, 62); // 80->67  (20/5 = -4)
    //int kuan_right_current = map(elapsedTime, 0, balancetime, 107, 100); // 80->67  (20/5 = -4)
    int shoulder_left_current = map(elapsedTime, 0, balancetime, 140, 140); // 50->70  (20/5 = -4)
    int shoulder_right_current = map(elapsedTime, 0, balancetime, 80, 80);
    int forearm_left_current = map(elapsedTime, 0, balancetime, 130, 140); // 50->70  (20/5 = -4)
    int forearm_right_current = map(elapsedTime, 0, balancetime, 70, 60); // 50->70  (20/5 = -4)
    int thigh_left_current = map(elapsedTime, 0, balancetime, 56, 55); //60->58    +20
    int calf_left_current = map(elapsedTime, 0, balancetime, 80, 72); //65->80     -10
    int calf_right_current = map(elapsedTime, 0, balancetime, 87, 77); //90->70  
    //int core_current = map(elapsedTime, 0, balancetime, 80, 70); //90->70  
    // Write the cu  -10rrent angles to the servos
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
    shoulder_left.write(shoulder_left_current);
    shoulder_right.write(shoulder_right_current);
    forearm_left.write(forearm_left_current);
    forearm_right.write(forearm_right_current);
    //kuan_left.write(kuan_left_current);
    //kuan_right.write(kuan_right_current);
    //core.write(core_current);

  }
}
void To_left2(){
  Serial.println("To left 2");

  shoulder_left.write(140);//120 數字- 往上 left shoulder
  shoulder_right.write(80);//60 數字- 往下 right shoulder
  arm_left.write(90);
  arm_right.write(90);
  forearm_left.write(140);//90 數字+ 往上
  forearm_right.write(60);//90 數字+ 往下
  thigh_right.write(135);//60 數字- 後勾  right thigh
  core.write(80);//95 數字+ 往左傾
 //用力往下踩//////////////////////////////////////////////////////
  int partytime = 400; 
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= partytime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;
    int kuan_left_current = map(elapsedTime, 0, partytime, 55, 62); // 80->67  (20/5 = -4)
    int kuan_right_current = map(elapsedTime, 0, partytime, 107, 100); // 80->67  (20/5 = -4)
    int thigh_left_current = map(elapsedTime, 0, partytime, 55, 53); //90->70     -10
    int calf_left_current = map(elapsedTime, 0, partytime, 72, 82); //90->70     -10
    int calf_right_current = map(elapsedTime, 0, partytime, 77, 69); //90->70     -10
    // Write the current angles to the servos
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
    kuan_left.write(kuan_left_current);
    kuan_right.write(kuan_right_current);

  }

}
/////////////////////////////////////////////////////////////////////////////////////////////////turnright
void TurnRight(){
  //////////////////////////////////balance
  if (roll<=-2){
    
    if (standbit){
      StandToBalanceRight();
    }
    Serial.println("");
    Serial.print("balance_right");
    Serial.println("");
    standbit = 0;
    while( roll <= -4 || filteredRoll >= 10 || filteredRoll < -10 ){
      Balance_right();
      IMUdata();
    }
    To_right1();
    To_right2();
  }
  ////////////////////////////////////ride
  else{
    standbit = 0;
    if (roll < 2){
      Serial.println("\nAdjust high");
      Adjust_high();
      delay(500);
    }
    turnright1();
    //delay(500);
    if (roll > -4){
      turnright2();
      delay(1000);
      if (roll > -4){
        turnright3();
        delay(500);
        if (roll > -4){
          stand();
        }
      }
    }
  }
}
void turnright1(){
  arm_left.write(90);
  arm_right.write(90);
  thigh_right.write(135);
  int steptime = 1200;
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int kuan_left_current = map(elapsedTime, 0, steptime, 62, 62);//50
    int kuan_right_current = map(elapsedTime, 0, steptime, 100, 100); //75
    int shoulder_left_current = map(elapsedTime, 0, steptime, 140, 100);
    int shoulder_right_current = map(elapsedTime, 0, steptime, 80, 40);
    int forearm_left_current = map(elapsedTime, 0, steptime, 140, 95);
    int forearm_right_current = map(elapsedTime, 0, steptime, 60, 45);
    int thigh_left_current = map(elapsedTime, 0, steptime, 51, 67);
    int calf_left_current = map(elapsedTime, 0, steptime, 80, 65);
    int calf_right_current = map(elapsedTime, 0, steptime, 69, 86);
    int core_current = map(elapsedTime, 0, steptime, 80, 70);

    kuan_left.write(kuan_left_current);
    kuan_right.write(kuan_right_current);
    shoulder_left.write(shoulder_left_current);
    shoulder_right.write(shoulder_right_current);
    forearm_left.write(forearm_left_current);
    forearm_right.write(forearm_right_current);
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
    core.write(core_current);
  }
  Serial.print("right_1");
}
//////////////////////////////////////拉回
void turnright2(){
  arm_left.write(90);
  arm_right.write(90);

  thigh_right.write(135);
  int steptime = 1500;
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int kuan_left_current = map(elapsedTime, 0, steptime, 62, 55); //65
    int kuan_right_current = map(elapsedTime, 0, steptime, 100, 72); //100
    int shoulder_left_current = map(elapsedTime, 0, steptime, 100, 160);
    int shoulder_right_current = map(elapsedTime, 0, steptime, 40, 95);
    int forearm_left_current = map(elapsedTime, 0, steptime , 95, 170);
    int forearm_right_current = map(elapsedTime, 0, steptime, 45, 90);
    int thigh_left_current = map(elapsedTime, 0, steptime, 67 , 53);
    int calf_left_current = map(elapsedTime, 0, steptime, 65, 70);
    int calf_right_current = map(elapsedTime, 0, steptime, 86, 60);
    int core_current = map(elapsedTime, 0, steptime, 70, 105);


    kuan_left.write(kuan_left_current);
    kuan_right.write(kuan_right_current);
    shoulder_left.write(shoulder_left_current);
    shoulder_right.write(shoulder_right_current);
    forearm_left.write(forearm_left_current);
    forearm_right.write(forearm_right_current);
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
    core.write(core_current);
  }
  Serial.print("right_2");
}
////////////////////////////////////調整
void turnright3(){
  
  //shoulder_left.write(160);//120 數字- 往上 left shoulder
  //shoulder_right.write(85);//60 數字- 往下 right shoulder
  arm_left.write(90);
  arm_right.write(90);
  //forearm_left.write(160);//90 數字+ 往上
  //forearm_right.write(80);//90 數字+ 往下
  //thigh_left.write(50);
  thigh_right.write(135);
  int steptime = 400;
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= steptime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int kuan_left_current = map(elapsedTime, 0, steptime, 55, 62);
    int kuan_right_current = map(elapsedTime, 0, steptime, 72, 100);
    int shoulder_left_current = map(elapsedTime, 0, steptime, 150, 140);
    int shoulder_right_current = map(elapsedTime, 0, steptime, 85, 80);
    int forearm_left_current = map(elapsedTime, 0, steptime, 160, 140);
    int forearm_right_current = map(elapsedTime, 0, steptime, 80, 60);
    int thigh_left_current = map(elapsedTime, 0, steptime, 53, 51); //55 50
    int calf_left_current = map(elapsedTime, 0, steptime, 70, 80); //65 80
    int calf_right_current = map(elapsedTime, 0, steptime, 60, 69);
    int core_current = map(elapsedTime, 0, steptime, 95, 80);

    kuan_left.write(kuan_left_current);
    kuan_right.write(kuan_right_current);
    shoulder_left.write(shoulder_left_current);
    shoulder_right.write(shoulder_right_current);
    forearm_left.write(forearm_left_current);
    forearm_right.write(forearm_right_current);
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
    core.write(core_current);
  }
  Serial.println("right_3");
}
void To_right1(){
  Serial.println("To right 1");
  //kuan_right.write(110);

  shoulder_left.write(140);
  shoulder_right.write(80);

  arm_left.write(90);
  arm_right.write(90);

  thigh_right.write(135);


  core.write(80);
 //用力往下踩//////////////////////////////////////////////////////
  int balancetime = 400; 
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= balancetime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int kuan_left_current = map(elapsedTime, 0, balancetime, 60, 62); // 80->67  (20/5 = -4)
    int kuan_right_current = map(elapsedTime, 0, balancetime, 107, 100); // 80->67  (20/5 = -4)
    int forearm_left_current = map(elapsedTime, 0, balancetime, 130, 140); // 50->70  (20/5 = -4)
    int forearm_right_current = map(elapsedTime, 0, balancetime, 70, 60); // 50->70  (20/5 = -4)
    int thigh_left_current = map(elapsedTime, 0, balancetime, 56, 55); //60->58    +20
    int calf_left_current = map(elapsedTime, 0, balancetime, 80, 72); //65->80     -10
    int calf_right_current = map(elapsedTime, 0, balancetime, 87, 77); //90->70     -10
    // Write the current angles to the servos
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
    forearm_left.write(forearm_left_current);
    forearm_right.write(forearm_right_current);
    kuan_left.write(kuan_left_current);
    kuan_right.write(kuan_right_current);

  }
}
void To_right2(){
  Serial.println("To right 2");
  kuan_left.write(62);
  kuan_right.write(100);//90 數字- 往內 right kuan
  shoulder_left.write(140);//120 數字- 往上 left shoulder
  shoulder_right.write(80);//60 數字- 往下 right shoulder
  arm_left.write(90);
  arm_right.write(90);
  forearm_left.write(140);//90 數字+ 往上
  forearm_right.write(60);//90 數字+ 往下
  thigh_right.write(135);//60 數字- 後勾  right thigh
  core.write(80);//95 數字+ 往左傾
 //用力往下踩//////////////////////////////////////////////////////
  int partytime = 400; 
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= partytime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;
    int thigh_left_current = map(elapsedTime, 0, partytime, 55, 51); //90->70     -10
    int calf_left_current = map(elapsedTime, 0, partytime, 72, 80); //90->70     -10
    int calf_right_current = map(elapsedTime, 0, partytime, 77, 69); //90->70     -10
    // Write the current angles to the servos
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);

  }

}


void stand(){
  Serial.println("stand");
  standbit = 1;
  //kuan_left.write(50);
  //kuan_right.write(90);//90 數字- 往內 right kuan
  shoulder_left.write(140);//120 數字- 往上 left shoulder
  shoulder_right.write(80);//60 數字- 往下 right shoulder
  arm_left.write(90);
  arm_right.write(90);
  forearm_left.write(140);//90 數字+ 往上
  forearm_right.write(60);//90 數字+ 往下
  thigh_right.write(135);//60 數字- 後勾  right thigh
  //calf_right.write(69);
  core.write(80);//95 數字+ 往左傾
 //用力往下踩//////////////////////////////////////////////////////
  int partytime = 1500; 
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= partytime && roll > -4) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;
    int kuan_left_current = map(elapsedTime, 0, partytime, 62, 45);
    int kuan_right_current = map(elapsedTime, 0, partytime, 100, 80);
    int thigh_left_current = map(elapsedTime, 0, partytime, 51, 70); //90->70     -10
    int calf_left_current = map(elapsedTime, 0, partytime, 80, 40); //90->70     -10
    int calf_right_current = map(elapsedTime, 0, partytime, 69, 60); //90->70     -10
    // Write the current angles to the servos
    kuan_left.write(kuan_left_current);
    kuan_right.write(kuan_right_current);
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);

  }

}

void StandToBalance(){
  Serial.println("stand to balance");
  shoulder_left.write(140);//120 數字- 往上 left shoulder
  shoulder_right.write(80);//60 數字- 往下 right shoulder
  arm_left.write(90);
  arm_right.write(90);
  thigh_right.write(135);//60 數字- 後勾  right thigh
  core.write(80);//95 數字+ 往左傾
 //用力往下踩//////////////////////////////////////////////////////
  int partytime = 500; 
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= partytime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;
    int kuan_left_current = map(elapsedTime, 0, partytime, 45, 55);
    int kuan_right_current = map(elapsedTime, 0, partytime, 80, 107);
    int forearm_left_current = map(elapsedTime, 0, partytime, 140, 130);
    int forearm_right_current = map(elapsedTime, 0, partytime, 60, 70);
    int thigh_left_current = map(elapsedTime, 0, partytime, 70, 55); //90->70     -10
    int calf_left_current = map(elapsedTime, 0, partytime, 40, 78); //90->70     -10
    int calf_right_current = map(elapsedTime, 0, partytime, 60, 86); //90->70     -10
    // Write the current angles to the servos
    kuan_left.write(kuan_left_current);
    kuan_right.write(kuan_right_current);
    forearm_left.write(forearm_left_current);
    forearm_right.write(forearm_right_current);
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);

  }
}

void StandToBalanceLeft(){
  Serial.println("stand to balance left");
  shoulder_left.write(140);//120 數字- 往上 left shoulder
  shoulder_right.write(80);//60 數字- 往下 right shoulder
  arm_left.write(90);
  arm_right.write(90);
  thigh_right.write(135);//60 數字- 後勾  right thigh
  core.write(80);//95 數字+ 往左傾
 //用力往下踩//////////////////////////////////////////////////////
  int partytime = 500; 
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= partytime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;
    int kuan_left_current = map(elapsedTime, 0, partytime, 45, 55);
    int kuan_right_current = map(elapsedTime, 0, partytime, 80, 107);
    int forearm_left_current = map(elapsedTime, 0, partytime, 140, 130);
    int forearm_right_current = map(elapsedTime, 0, partytime, 60, 70);
    int thigh_left_current = map(elapsedTime, 0, partytime, 70, 56); //90->70     -10
    int calf_left_current = map(elapsedTime, 0, partytime, 40, 80); //90->70     -10
    int calf_right_current = map(elapsedTime, 0, partytime, 60, 87); //90->70     -10
    // Write the current angles to the servos
    kuan_left.write(kuan_left_current);
    kuan_right.write(kuan_right_current);
    forearm_left.write(forearm_left_current);
    forearm_right.write(forearm_right_current);
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);

  }
}

void StandToBalanceRight(){
  Serial.println("stand to balance right");
  shoulder_left.write(140);//120 數字- 往上 left shoulder
  shoulder_right.write(80);//60 數字- 往下 right shoulder
  arm_left.write(90);
  arm_right.write(90);
  thigh_right.write(135);//60 數字- 後勾  right thigh
  core.write(80);//95 數字+ 往左傾
 //用力往下踩//////////////////////////////////////////////////////
  int partytime = 500; 
  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= partytime) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;
    int kuan_left_current = map(elapsedTime, 0, partytime, 45, 60);
    int kuan_right_current = map(elapsedTime, 0, partytime, 80, 107);
    int forearm_left_current = map(elapsedTime, 0, partytime, 140, 130);
    int forearm_right_current = map(elapsedTime, 0, partytime, 60, 70);
    int thigh_left_current = map(elapsedTime, 0, partytime, 70, 56); //90->70     -10
    int calf_left_current = map(elapsedTime, 0, partytime, 40, 80); //90->70     -10
    int calf_right_current = map(elapsedTime, 0, partytime, 60, 87); //90->70     -10
    // Write the current angles to the servos
    kuan_left.write(kuan_left_current);
    kuan_right.write(kuan_right_current);
    forearm_left.write(forearm_left_current);
    forearm_right.write(forearm_right_current);
    thigh_left.write(thigh_left_current);
    calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);

  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Display() {
  char buffer[120];
  
  // 格式化輸出，每個數字占固定寬度（例如 6 個字符）
  sprintf(buffer, " STEP : %2d  LEFT Distance : %6ld  MIDDLE Distance : %6ld  RIGHT Distance : %6ld ", step, distance1, distance2, distance3);
  Serial.print(buffer);
  //Serial.print(distance3); 
  Serial.print("       Roll : ");
  Serial.println(roll,1); 

}
void Adjust_high(){

  shoulder_left.write(140);
  shoulder_right.write(80);
  forearm_left.write(140);
  forearm_right.write(60);
  arm_left.write(90);
  arm_right.write(90);
  thigh_right.write(135);//右腿蹲
  core.write(80);
  calf_left.write(82);
  thigh_left.write(51);
  calf_right.write(80);

  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= 500) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      //distance();
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    int kuan_left_current = map(elapsedTime, 0, steptime, 62, 65); //53->133    +8
    int kuan_right_current = map(elapsedTime, 0, steptime, 100, 105); //82->42     -4
    //int thigh_left_current = map(elapsedTime, 0, steptime, 53, 63); //53->133    +8
    //int calf_left_current = map(elapsedTime, 0, steptime, 82, 77); //82->42     -4
    //int calf_right_current = map(elapsedTime, 0, steptime, 69, 71);  // 60->80  +2

    kuan_left.write(kuan_left_current);
    kuan_left.write(kuan_left_current);
    //thigh_left.write(thigh_left_current);
    //calf_left.write(calf_left_current);
    //calf_right.write(calf_right_current);
  }
}
void Adjust_low(){
  Fixed();
  calf_left.write(82);

  startTime = millis();  // Record the start time
  elapsedTime = 0;
  while (elapsedTime <= 500) {
    for (int i = 0 ; i < 50; i++ ){
    if (i%5 == 0){
      IMUdata(); 
      //distance();
      }
    }
    currentTime = millis();
    elapsedTime = currentTime - startTime;

    //int thigh_left_current = map(elapsedTime, 0, steptime, 53, 63); //53->133    +8
    //int calf_left_current = map(elapsedTime, 0, steptime, 82, 77); //82->42     -4
    int calf_right_current = map(elapsedTime, 0, steptime, 69, 67);  // 60->80  +2
    //thigh_left.write(thigh_left_current);
    //calf_left.write(calf_left_current);
    calf_right.write(calf_right_current);
  }
}
