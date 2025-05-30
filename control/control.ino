/*
  ============================================
  UFMG BIA-Kit is placed under the MIT License
  Copyright (c) 2024 by GTI (UFMG)
  ============================================

  Mechanical design: https://cad.onshape.com/documents/a3f5df55d0d81678d39d592b/w/adcc6d84828ac41bc0f0c0d9/e/42ce4c70408afcd2abf03366?renderMode=0&uiState=6830ca4d92858f230c27091c

*/

// I2C libray communication
#include <Wire.h>

// Bluetooth communication
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

// ENCODER library based on the built in counter hardware
#include <ESP32Encoder.h>

// ESP32 BLUE LED pin
#define INTERNAL_LED 2

// IMU I2C address
#define MPU   0x68

// SERVO pin
// #define SERVO_PWM 32
// #define STEERING_CENTER 78
// #define ANGULO_MAX 31
// int steering = 0;

// SERVO PWM config
// #define SERVO_TIMER_BIT 16
// #define SERVO_BASE_FREQ 50
// #define SERVO_PWM_CH 3

// NIDEC PWM config
#define NIDEC_TIMER_BIT   8
#define NIDEC_BASE_FREQ   20000

// NIDEC pins: Reaction Wheel Motor
#define BRAKE      14 //Yellow wire
#define NIDEC_PWM  27 //White wire 
#define DIR        16 //Green wire
#define ENCA       25 //Brown wire
#define ENCB       26 //Orange
#define NIDEC_PWM_CH 1

// Encoder vars
ESP32Encoder NIDEC_ENC;

// Kalman Filter vars
float Q_angle = 0.001; // Angular data confidence
float Q_bias  = 0.005; // Angular velocity data confidence
float R_meas  = 1.0;
float roll_angle = 0.0;
float bias = 0.0;
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float K[2] = {0, 0};

// Control vars
//     theta         dthehta         omega           integral 
float K1 = -192,    K2 = -21.30,    K3 = -0.462,    K4 = 0.251;
//float K1 = -188,    K2 = -19.8,    K3 = -0.112,    K4 = 0.011;
float U = 0;
int pwm = 0;
float theta = 0.0, theta_dot = 0.0;            // System states
float omega = 0, integral = 0, integral_past = 0;
float Ts = 0.01, currentT = 0.0, previousT = 0.0;        // Elapsed time in loop() function

// MAIN SETUP
void setup() { // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);                   // make sure your Serial Monitor is also set at this baud rate.
  SerialBT.begin("Wheel"); // Bluetooth device name

  NIDECsetup();
  IMUsetup();
  // SERVOsetup(); // Remove servo setup

  pinMode(INTERNAL_LED,OUTPUT);
  digitalWrite(INTERNAL_LED,HIGH);  // Turn on blue led
  delay(1500);                      // Wait for the system to stabilize
  for (int i=1; i<= 400; i++){      // Wait for the Kalman filter stabilize
    IMUread();
    delay(5);
  }
  currentT = millis();
  digitalWrite(INTERNAL_LED,LOW);  
}

// MAIN LOOP
void loop() {// put your main code here, to run repeatedly:

  currentT = millis();
  if ((currentT - previousT)/1000.0 >= Ts) {
    previousT = currentT;
    Tuning();                   // tuning control parameters

    IMUread();
    Serial.print(roll_angle);
    Serial.print(" ");
    Serial.println(theta_dot);


    if (abs(roll_angle) < 20){ 
      digitalWrite(BRAKE, HIGH);

      theta += theta_dot * Ts;
      integral = -NIDEC_ENC.getCount()/63.7;
      omega = -(integral - integral_past)/Ts;
      integral_past = integral;
      // Serial.print(integral);
      // Serial.print(" ");
      // Serial.println(omega);

      U = -(K1*theta + K2*theta_dot + K3*omega + K4*integral);
      pwm = U*21.3;

      MOTORcmd(pwm);
      
    } else {
      digitalWrite(BRAKE, LOW);
      digitalWrite(INTERNAL_LED,HIGH); 
      delay(5000);
      digitalWrite(INTERNAL_LED,LOW);
      for (int i=1; i<= 400; i++){
        IMUread();
        delay(5);
      }
      previousT = millis();
      theta = 0.0;
      integral = 0.0;
      NIDEC_ENC.clearCount();
    }
  }   

}

// SETUP functions
void IMUsetup(){
  // Initialize the MPU6050
  Wire.beginTransmission(MPU);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                      //make the reset (place a 0 into the 6B register)
  Wire.write(0);
  Wire.endTransmission(true);            //end the transmission
  //Gyro config
  Wire.beginTransmission(MPU);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                      //We want to write to the GYRO_CONFIG register (1B hex)
  // Wire.write(0x00000000);             //Set the register bits as 00000000 (250dps full scale), 00010000 (1000dps full scale)
  Wire.write(1 << 3);
  Wire.endTransmission();                //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(MPU);           //Start communication with the address found during search.
  Wire.write(0x1C);                      //We want to write to the ACCEL_CONFIG register
  Wire.write(0b00000000);                //Set the register bits as 00000000 (+/- 2g full scale range), 00010000 (+/- 8g full scale range)
  Wire.endTransmission(); 
}

// void SERVOsetup(){
//   pinMode(SERVO_PWM, OUTPUT);
//   ledcSetup(SERVO_PWM_CH, SERVO_BASE_FREQ, SERVO_TIMER_BIT);
//   ledcAttachPin(SERVO_PWM, SERVO_PWM_CH);
//   SERVOangle(STEERING_CENTER);                   // SERVO initial position
// }

void NIDECsetup(){
  pinMode(BRAKE, OUTPUT);
  digitalWrite(BRAKE, HIGH);

  pinMode(DIR, OUTPUT);
  ledcAttach(NIDEC_PWM, NIDEC_BASE_FREQ, NIDEC_TIMER_BIT);
  MOTORcmd(0);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  NIDEC_ENC.attachFullQuad(ENCB, ENCA);
  NIDEC_ENC.clearCount();
}

// IMU function: Kalman Filter
void IMUread(){
  // read IMU
  int16_t ax,ay,az,temp,gx,gy,gz;
  int16_t rax,raz,rgx,rgz;
  Wire.beginTransmission(MPU);    // IMU address: 0x68
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14);        // IMU address: 0x68
  rax=Wire.read()<<8|Wire.read();   // X-axis value: 16384.0; 
  ay=Wire.read()<<8|Wire.read();   // Y-axis value: 16384.0;     
  raz=Wire.read()<<8|Wire.read();   // Z-axis value: 16384.0;  
  temp=Wire.read()<<8|Wire.read();      
  rgx=Wire.read()<<8|Wire.read();  
  gy=Wire.read()<<8|Wire.read();  
  rgz=Wire.read()<<8|Wire.read();  
  //
  az = rax;
  gz = rgx;
  ax = -raz;
  gx = -rgz;
  //
  //accelerometer angles in degrees (or rads)
  float ax_angle = atan2(ay, sqrt(ax*ax + az*az)) * 57.3; // roll
  //float ay_angle = atan2(ax, sqrt(ay*ay + az*az)) * 57.3; // pitch
  // float az_angle = atan2(sqrt(ax*ax + az*az), az) * 57.3; // yaw (useless)
  // gyro measurements in degress (or rads)
  gx =  gx / 65.5; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  // gy =  gy / 65.5; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  // gz =  gz / 65.5; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  
  // begin: Kalman filter - Roll Axis (X)
  roll_angle += (gx - bias) * Ts;
  
  P[0][0] += (Q_angle - P[0][1] - P[1][0]) * Ts;
  P[0][1] += -P[1][1] * Ts;
  P[1][0] += -P[1][1] * Ts;
  P[1][1] += Q_bias * Ts;
  //
  K[0] = P[0][0] / (P[0][0] + R_meas);
  K[1] = P[1][0] / (P[0][0] + R_meas);  
  //
  roll_angle += K[0] * (ax_angle - roll_angle); 
  bias       += K[1] * (ax_angle - roll_angle);
  //
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
  // end: Kalman filter 

  theta_dot = (gx - bias)/57.3; // Unbiased gyro speed
  
  // //  Complementary filter   
  // roll_anglec = 0.98 * (roll_anglec + gx * Ts) + 0.02 * ax_angle;
  // pitch_anglec = 0.98 * (pitch_anglec + gy * Ts) + 0.02 * ay_angle;
  // yaw_anglec = 0.98 * (yaw_anglec + gz * Ts) + 0.02 * az_angle;

}  

// NIDEC functions
void MOTORcmd(int sp) {
  if (sp < 0) {
    digitalWrite(DIR, HIGH);
    sp = -sp;
  } else {
    digitalWrite(DIR, LOW);
  }
  ledcWrite(NIDEC_PWM_CH, int(sp > 255 ? 0 : 255 - sp));
}

// Remove SERVO function
// void SERVOangle(int angle) {
//   float dutyCycle = map(angle, 0, 180, 1500, 7900);
//   ledcWrite(SERVO_PWM_CH, int(dutyCycle));  
// }

//TUNING function
//     theta         dthehta         omega           integral 
// float K1 = -188,    K2 = -19.8,    K3 = -0.112,    K4 = 0.011;
int Tuning() {
  if (!SerialBT.available())  return 0;
  char param = SerialBT.read();               // get parameter byte
  if (!SerialBT.available()) return 0;
  char cmd = SerialBT.read();                 // get command byte
  switch (param) {
    case 'a':
      if (cmd == '+')    K1 += 2;
      if (cmd == '-')    K1 -= 2;
      print_values();
      break;
    case 'b':
      if (cmd == '+')    K2 += 0.5;
      if (cmd == '-')    K2 -= 0.5;
      print_values();
      break;  
    case 'c':
      if (cmd == '+')    K3 += 0.01;
      if (cmd == '-')    K3 -= 0.01;
      print_values();
      break;  
     case 'd':
      if (cmd == '+')    K4 += 0.01;
      if (cmd == '-')    K4 -= 0.01;
      print_values();
      break;                             
   }
   return 1;  
}

void print_values() {
  SerialBT.print("K1(a): "); SerialBT.print(K1);
  SerialBT.print(" K2(b): "); SerialBT.print(K2);
  SerialBT.print(" K3(c): "); SerialBT.print(K3,4);
  SerialBT.print(" K4(d): "); SerialBT.println(K4,4);
}
