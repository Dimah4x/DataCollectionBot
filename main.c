#include <SoftwareSerial.h>
#include <bit_def.h>
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#define left_m1 9   //topleft
#define left_m2 8
#define right_m1 11 //topright
#define right_m2 10
#define rxPin 2
#define txPin 3
#define trig_pin 4
#define echo_pin 5
int sensorPin = A0;    // select the input pin for the potentiometer
int sensorValue = 0; 
SoftwareSerial BlueTooth = SoftwareSerial( rxPin, txPin );
int distance;
long duration;
 unsigned char temp_char;
boolean autom = false;
String readString;
char data_recieve;
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
byte AngleX = 0 , AngleY = 0;
uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
  unsigned char distance_char ; 
  byte r= 0;
String state = "Nothing";
void setup() {
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  pinMode(left_m1, OUTPUT);
  pinMode(left_m2, OUTPUT);
  pinMode(right_m1, OUTPUT);
  pinMode(right_m2, OUTPUT);
 analogReference(INTERNAL);
  Serial.begin(9600);
  BlueTooth.begin(9600);
 Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }
  delay(100); // Wait for sensor to stabilize
  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;
  timer = micros();
}
void forward(){
  digitalWrite(left_m2, HIGH);
  digitalWrite(left_m1, LOW);
  digitalWrite(right_m2, HIGH);
  digitalWrite(right_m1, LOW);
}
void  stop_motor(){
  digitalWrite(left_m1, LOW);
  digitalWrite(left_m2, LOW);
  digitalWrite(right_m1, LOW);
  digitalWrite(right_m2, LOW);
}
void left(){
  digitalWrite(left_m1, HIGH);
  digitalWrite(left_m2, LOW);
  digitalWrite(right_m1, LOW);
  digitalWrite(right_m2, HIGH);
}
void back(){
  digitalWrite(left_m1, HIGH);
  digitalWrite(left_m2, LOW);
  digitalWrite(right_m1, HIGH);
  digitalWrite(right_m2, LOW);
}
void right(){
  digitalWrite(left_m1, LOW);
  digitalWrite(left_m2, HIGH);
 digitalWrite(right_m1, HIGH);
  digitalWrite(right_m2, LOW);
}
void GetData1(){
   /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];
  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();
  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif
  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;
  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
  /* Print Data */
#if 1 // Set to 1 to activate
#endif
#if 1 // Set to 1 to print the temperature
  double temperature = (double)tempRaw / 340.0 + 36.53;
  temp_char = temperature;
 if(kalAngleX < 0)
  kalAngleX = -kalAngleX;
 if(kalAngleY < 0)
  kalAngleY = -kalAngleY;
 AngleX = kalAngleX;
 AngleY = kalAngleY;
 delay(5);
 #endif
  delay(2); 
}
void getDistance(){
digitalWrite(trig_pin, LOW); 
 delayMicroseconds(2); 
 digitalWrite(trig_pin, HIGH);
 delayMicroseconds(10);  
 digitalWrite(trig_pin, LOW);
 duration = pulseIn(echo_pin, HIGH);
 //Calculate the distance (in cm) based on the speed of sound.
 distance = duration/58;
 if(distance<20) {
       stop_motor();
   }
 }
void loop (){
  if (BlueTooth.available()) 
 data_recieve= BlueTooth.read();
 switch(data_recieve){
   case 'f':{
    forward();
    if(distance<20) {
       stop_motor();
       }
    data_recieve=0;
    break;
    }
  case 'b':{
    back();
    data_recieve=0;
    break;
    }  
  case 'l':{
    left();
    data_recieve=0;
    break;
    }
  case 'r':{
    right();
    data_recieve=0;
    break;
  }  
  case 's':{
    stop_motor();
    data_recieve=0;
    break; 
    }    
  case 'd':{
    BlueTooth.print("d"); 
    BlueTooth.print(distance/100);
    BlueTooth.print(distance%100/10);
    BlueTooth.print(distance%100%10);       
    data_recieve=0;
    break;
    } 
  case 'x':{
    BlueTooth.print("x"); 
    BlueTooth.print(AngleX/100);
    BlueTooth.print(AngleX%100/10);
    BlueTooth.print(AngleX%100%10);     
    data_recieve=0;
    break;
    }  
  case 'y':{
    BlueTooth.print("y"); 
    BlueTooth.print(AngleY/100);
    BlueTooth.print(AngleY%100/10);
    BlueTooth.print(AngleY%100%10);   
    data_recieve=0;
    break;
    }  
  case 't':{
    BlueTooth.print("t"); 
    BlueTooth.print(temp_char/100);
    BlueTooth.print(temp_char%100/10);
    BlueTooth.print(temp_char%100%10); 
    data_recieve=0;
    break;
    } 
  case 'g':{
    BlueTooth.print("g"); 
    if(sensorValue>700)
      BlueTooth.print("h"); 
    else
      BlueTooth.print("l"); 
    data_recieve=0;
    break;
    } 
   }
getDistance();
GetData1();
sensorValue = analogRead(sensorPin);   
delay(10);
}
