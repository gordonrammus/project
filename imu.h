#include <Wire.h>
#include <Servo.h>


long loopTimer, loopTimer2;
int temperature;
double accelPitch;
double accelRoll;
long acc_x, acc_y, acc_z;
double accel_x, accel_y, accel_z;
double gyroRoll, gyroPitch, gyroYaw;
int gyro_x, gyro_y, gyro_z;
long gyro_x_cal = -29, gyro_y_cal= 98, gyro_z_cal = -66;
double rotation_x, rotation_y, rotation_z;
double freq, dt;
double tau = 0.98;
double roll = 0;
double pitch = 0;
double prevRoll = 0;  // 上一個pitch值
double rollSpeed = 0; // pitch速度值

// 移動平均濾波器參數
const int filterSize = 10;
double rollHistory[filterSize] = {0};
int filterIndex = 0;
double filteredRoll = 0;

// 250 deg/s --> 131.0, 500 deg/s --> 65.5, 1000 deg/s --> 32.8, 2000 deg/s --> 16.4
long scaleFactorGyro = 65.5;

// 2g --> 16384 , 4g --> 8192 , 8g --> 4096, 16g --> 2048
long scaleFactorAccel = 8192;

void read_mpu_6050_data() {
  // Subroutine for reading the raw data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 14);

  // Read data --> Temperature falls between acc and gyro registers
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  temperature = Wire.read() <<8 | Wire.read();
  gyro_x = Wire.read()<<8 | Wire.read();
  gyro_y = Wire.read()<<8 | Wire.read();
  gyro_z = Wire.read()<<8 | Wire.read();
}

void calibrate_gyro() {
  Serial.println("Calibrating gyro, place on level surface and do not move.");

  for (int cal_int = 0; cal_int < 3000; cal_int++) {
    if(cal_int % 200 == 0)Serial.print(".");
    read_mpu_6050_data();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    delay(3);
  }

  gyro_x_cal /= 3000;
  gyro_y_cal /= 3000;
  gyro_z_cal /= 3000;
  
}



void IMUdata(){
  freq = 1/((micros() - loopTimer2) * 1e-6);
  loopTimer2 = micros();
  dt = 1/freq;

  // Read the raw acc data from MPU-6050
  read_mpu_6050_data();

  // Subtract the offset calibration value
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  // Convert to instantaneous degrees per second
  rotation_x = (double)gyro_x / (double)scaleFactorGyro;
  rotation_y = (double)gyro_y / (double)scaleFactorGyro;
  rotation_z = (double)gyro_z / (double)scaleFactorGyro;

  // Convert to g force
  accel_x = (double)acc_x / (double)scaleFactorAccel;
  accel_y = (double)acc_y / (double)scaleFactorAccel;
  accel_z = (double)acc_z / (double)scaleFactorAccel;

  // Complementary filter
  accelPitch = atan2(accel_y, accel_z) * RAD_TO_DEG;
  accelRoll = atan2(accel_x, accel_z) * RAD_TO_DEG;

  pitch = (tau)*(pitch + rotation_x*dt) + (1-tau)*(accelPitch);
  roll = (tau)*(roll - rotation_y*dt) + (1-tau)*(accelRoll);

  gyroPitch += rotation_x*dt;
  gyroRoll -= rotation_y*dt;
  gyroYaw += rotation_z*dt;
  // 計算pitch速度
  rollSpeed = (roll - prevRoll) / dt;
  prevRoll = roll;

  // 移動平均濾波器
  rollHistory[filterIndex] = rollSpeed;
  filterIndex = (filterIndex + 1) % filterSize;
  filteredRoll = 0;
  for (int i = 0; i < filterSize; i++) {
    filteredRoll += rollHistory[i];
  }
  filteredRoll /= filterSize;

  // Data out serial monitor
  /*
  Serial.print(freq,0);   Serial.print(",");
  Serial.print(roll,1);   Serial.print(",");
  Serial.print(pitch,1);  Serial.print(","); */
 // Serial.println(filteredRoll, 2);  
  
  //Serial.println(rollSpeed, 1);  

  // Wait until the loopTimer reaches 4000us (250Hz) before next loop
  while (micros() - loopTimer <= 4000);
  loopTimer = micros();
}





void setup_mpu_6050_registers() {
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Configure the accelerometer
  // Wire.write(0x__);
  // Wire.write; 2g --> 0x00, 4g --> 0x08, 8g --> 0x10, 16g --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x08);
  Wire.endTransmission();

  // Configure the gyro
  // Wire.write(0x__);
  // 250 deg/s --> 0x00, 500 deg/s --> 0x08, 1000 deg/s --> 0x10, 2000 deg/s --> 0x18
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
}