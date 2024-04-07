#include <Servo.h>
#include <Wire.h>
#include <math.h>
#include <AS5600.h>
#include <NewPing.h>
#define TRIGGER_PIN_FORWARD 12 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_FORWARD 11 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL SerialUSB
#define SYS_VOL 3.3
#else
#define SERIAL Serial
#define SYS_VOL 5
#endif

Servo myservo;
Servo TurnServo;
NewPing sonar_forward(TRIGGER_PIN_FORWARD, ECHO_PIN_FORWARD, MAX_DISTANCE);
AMS_5600 ams5600;
int ang, lang = 0;

//Turn Servo變數
byte pin_TurnServo = 8;
byte turnAngle = 90;
const byte central = 90;
const byte angle_1 = 7;
const byte angle_2 = 14;
const byte angle_3 = 24;

//紅外線循跡變數
const byte NumberOfSensor = 7;
//int tcrtDetectorPin[NumberOfSensor] = {A11, A12, A13, A14, A15};
byte tcrtEmitterPin[NumberOfSensor] = {40, 42, 44, 46, 48, 50, 52};
byte a[NumberOfSensor];
byte SensorStatus = 0;
byte SensorStatus_Mask[NumberOfSensor] = {1 << 6, 1 << 5, 1 << 4, 1 << 3, 1 << 2, 1 << 1, 1};
boolean stop_flag = 0;
byte count_stop = 0; //計算競賽範圍 1前半段 2中間停止區 3後半段 4最後停止區
int count_time = 0;
int last_count_time = -2100;

float convertRawAngleToDegrees(word newAngle){
  float retVal = newAngle * 0.087;/* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  ang = retVal;
  return ang;
}

float distance = 0;
int distance_relative = 0;
float angular_speed = 0;
int currentAngle = 0;
int lastAngle = 0;
int lastAngle_dis = 0;
float lastTime = 0;
float currentTime = 0;
float deltaAngle = 0;
float deltaTime = 0;
int numOfCircle = 0;
bool flag_360 = false;

float MagnetEncoder_history[5];
float MagnetEncoder_smoother[5] = {0.25, 0.24, 0.22, 0.18, 0.11};
float MagnetEncoder_speed = 0;
byte MagnetEncoder_iteration = 0;

float desireSpeed = 0;
float desireDistance = 32;

int ultraSound_history[5];
int ultraSound_iteration;
float ultraSound_smoother[5] = {0.25, 0.24, 0.22, 0.18, 0.11};
bool detectAnother = false;
bool carPass = false;

float velocityErrorHistory[10];
int velocityErrorHisIter = 0;
float velocityErrorSum = 0;
float distanceErrorHistory[10];
int distanceErrorHisIter = 0;
float distanceErrorSum = 0;
float lastPIDOutput = 1050;
float PIDOutput = 1050;
float K_p[2] = {0.1, 0.0005};
float K_i[2] = {0.0007, 0};
float K_d[2] = {4.5, 25};
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup(){
  Serial.begin(115200);
  myservo.attach(7); // attaches the servo on pin 9 to the servo object
  myservo.writeMicroseconds(1000);
  Wire.begin();
  Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>>> ");
  if(ams5600.detectMagnet() == 0 ){
    while(1){
      if(ams5600.detectMagnet() == 1 ){
      Serial.print("Current Magnitude: ");
      Serial.println(ams5600.getMagnitude());
      break;
      }
      else{
        Serial.println("Can not detect magnet");
      }
    }
  }
  lastAngle_dis = convertRawAngleToDegrees(ams5600.getRawAngle());
  lastAngle = lastAngle_dis;
  //set up turn Servo
  TurnServo.attach(pin_TurnServo);
  TurnServo.write(90);
  turnAngle = 90;

  pinMode(31, INPUT_PULLUP);

  //set up tcrt5000
  for (int i = 0; i < NumberOfSensor; i++)
  pinMode(tcrtEmitterPin[i], INPUT);
  delay(4000);
}

bool shit = false;
void loop(){
  /*if(Serial.available()!=0){
    char a = Serial.read();
    if(a == 'A'){
    shit = true;
    }
  }
  if(shit){ }*/
  dejavu();
  mySpeed();
  //HC_SR04();
  Encoder2PID();

  if (digitalRead(31)){
    myservo.writeMicroseconds(1000);
    //Serial.println(digitalRead(31));
    while(1);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
int session = 0;
void dejavu(){
  switch(session){
    case 0:
      if(distance <= 2256) angle();
      else session++;
      break;
    case 1:
      if(distance<=2291){
        TurnServo.write(central - angle_1 - 4);
      }else session++;
      break;
    case 2:
      if(distance<=2306){
        angle_dejavu(1);
      }else session++;
      break;
    case 3:
      if(distance < 2450)angle();
      else if(distance >= 250){
        session++;
      }
      break;
    case 4:
      if(distance<=2485){
        TurnServo.write(central + angle_1 + 4);
      }else{
        session++;
      }
      break;
    case 5:
      if(distance<=2500) angle_dejavu(2);
      else session++;
      break;
    case 6:
      angle();
      break;
  }
}
void PID_time(){
  desireSpeed = getDesireSpeed(distance);
  velocityErrorHistory[velocityErrorHisIter]=(desireSpeed - MagnetEncoder_speed);
  velocityErrorSum = velocityErrorSum + velocityErrorHistory[velocityErrorHisIter];
  PIDOutput = lastPIDOutput + (K_p[0] * (velocityErrorHistory[velocityErrorHisIter]) + K_i[0] * velocityErrorSum + K_d[0] * (velocityErrorHistory[velocityErrorHisIter] - velocityErrorHistory[(velocityErrorHisIter+9)%10]));
  if(PIDOutput >= 1300) PIDOutput = 1300;
  else if(PIDOutput <= 1100) PIDOutput = 1100;
  PIDOutput = (int)PIDOutput;
  myservo.writeMicroseconds (PIDOutput);
  //Serial.println(PIDOutput);
  lastPIDOutput = PIDOutput;
  velocityErrorHisIter++;
  if(velocityErrorHisIter==10) velocityErrorHisIter = 0;
  velocityErrorHistory[velocityErrorHisIter] = 0;
  if(distance == 300) velocityErrorSum=0;/*error sum should be tuned*/
}

float getDesireSpeed(float dis){
  if( dis <= 2256 ){
    desireSpeed = 50;
  }
  else if(dis<=2600){
    desireSpeed=30;
  }
  else if(dis<=3000){
    desireSpeed=50;
  }
  return desireSpeed;
}
float last_pid_time = 0;
void Encoder2PID(){
  float pid_time = millis();
  String s;
  s="speed: ";
  s += MagnetEncoder_speed;
  //s += " , D: ";
  //s += distance;
  if(distance <= 6768){
    if( pid_time-last_pid_time >= 50 ){
    //s+=" , output: ";
    //PID_time_track();
    PID_time();
    Serial.println(s);
    last_pid_time = pid_time;
    }
  }
  else if(distance > 300){
    myservo.writeMicroseconds (1000);
  }
}
void mySpeed(){
  currentAngle = convertRawAngleToDegrees(ams5600.getRawAngle());
  currentTime = millis();
  deltaTime = currentTime - lastTime;
  if(currentAngle <= 1|| deltaAngle >= 355){
    flag_360 = true;
  }/*
  if(currentAngle == lastAngle+1){
    numOfCircle
  }*/
  if(deltaTime >= 50){/*max == 445*/
    myDistance(currentAngle);
    if(flag_360 == true){
      if(currentAngle < lastAngle){
        deltaAngle = (currentAngle + 360) - lastAngle;
      }
      else{
        deltaAngle = currentAngle - (lastAngle + 360);
      }
      flag_360 = false;
    }
    else{
      deltaAngle = currentAngle - lastAngle;
    }
    if(deltaAngle >= 0){
      angular_speed = (deltaAngle + numOfCircle*360)/deltaTime*1000/180*PI;
    }
    MagnetEncoder_history[MagnetEncoder_iteration] = angular_speed/*rad/s*/*5.7/2;
    MagnetEncoder_iteration++;
    if( MagnetEncoder_iteration == 5 ) MagnetEncoder_iteration = 0;//make the pointer back to the head of the array
    MagnetEncoder_speed = 0;
    for(int i =0; i<5;++i){
      MagnetEncoder_speed += MagnetEncoder_smoother[i]*MagnetEncoder_history[(MagnetEncoder_iteration+1+i)%5];
    } 
    //Serial.print("speed: ");
    //Serial.println(MagnetEncoder_speed);
    lastAngle = currentAngle;
    lastTime = currentTime;
    deltaAngle = 0 ;
    numOfCircle = 0;
  }
}
void myDistance(int angle){
  int small_delta_angle = (angle + 360 - lastAngle_dis)%360 < abs(angle - (lastAngle_dis + 360))%360 ? (angle + 360 - lastAngle_dis)%360 : abs(angle - (lastAngle_dis + 360))%360;
  if(small_delta_angle>3){
    //Serial.print(small_delta_angle);
    distance += (small_delta_angle/360.0)*PI*5.7;
    lastAngle_dis = angle;
  }
}
//循跡控制
void angle()
{
  for (int i = 0; i < NumberOfSensor; i++)
  {
    //a[i] = AnalogToDigital(analogRead(tcrtDetectorPin[i]));
    a[i] = digitalRead(tcrtEmitterPin[i]);
    //Serial.print(digitalRead(tcrtEmitterPin[i]));
    //Serial.print("\t");
    //Serial.println(a[i]);
    //Serial.println("straight");
    while (a[i])
    {
      SensorStatus |= SensorStatus_Mask[i];
      break;
    }
  }
  //Serial.println(SensorStatus);
  switch (SensorStatus)
  {
    case 0x8:
    //SteerAngle = 0;
    TurnServo.write(central);
    //Serial.println("straight");
    break;
    case 0x10:
    //SteerAngle = 1;
    TurnServo.write(central - angle_1);
    //delay(15);
    //Serial.println('R');
    break;
    case 0x4:
    //SteerAngle = -1;
    TurnServo.write(central + angle_1);
    //delay(15);
    //Serial.println('L');
    break;
    case 0x20:
    //SteerAngle = 2;
    TurnServo.write(central - angle_2);
    //delay(20);
    //Serial.println("RR");
    break;
    case 0x2:
    //SteerAngle = -2;
    TurnServo.write(central + angle_2);
    //delay(20);
    //Serial.println("LL");
    break;
    case 0x40:
    //SteerAngle = 3;
    TurnServo.write(central - angle_3);
    //delay(45);
    //Serial.println("RRR");
    break;
    case 0x1:
    //SteerAngle = -3;
    TurnServo.write(central + angle_3);
    //delay(45);
    //Serial.println("LLL");
    break;
    default:
    //SteerAngle = 0;
    TurnServo.write(central);
    //Serial.println("straight");
    break;
  }

  /*if(((SensorStatus & 0x36) == 0x36))
  {

  }*/
  SensorStatus = 0;
}

void angle_dejavu(byte id){
    for (int i = 0; i < NumberOfSensor; i++)
    {
      a[i] = digitalRead(tcrtEmitterPin[i]);
      while (a[i])
      {
        SensorStatus |= SensorStatus_Mask[i];
        break;
      }
    }
  if (id == 1){
    switch (SensorStatus)
    {
      case 0x20:
        TurnServo.write(central + angle_1);
        break;
      case 0x8:
        TurnServo.write(central + angle_1 + 5);
        session++;
        break;
      default:
        TurnServo.write(central);
        session++;
        break;
    }
  }
  else if(id==2){
    switch (SensorStatus){
      case 0x2:
        TurnServo.write(central - angle_1);
        break;
      case 0x8:
        TurnServo.write(central - angle_1 - 3);
        break;
      default:
        TurnServo.write(central);
        break;
    }
  }
}

void PID_time_track(){
  distanceErrorHistory[distanceErrorHisIter]= -(desireDistance - distance_relative);
  distanceErrorSum = distanceErrorSum + distanceErrorHistory[distanceErrorHisIter];
  PIDOutput = lastPIDOutput + (K_p[1] * (distanceErrorHistory[distanceErrorHisIter]) + K_i[1] * distanceErrorSum + K_d[1] * (distanceErrorHistory[distanceErrorHisIter] - distanceErrorHistory[(distanceErrorHisIter+9)%10]));
  if(PIDOutput >= 1240) PIDOutput = 1240;
  else if(PIDOutput <= 1150) PIDOutput = 1150;
  PIDOutput = (int)PIDOutput;
  myservo.writeMicroseconds (PIDOutput);
  lastPIDOutput = PIDOutput;
  distanceErrorHisIter++;
  if(distanceErrorHisIter==10) distanceErrorHisIter = 0;
  distanceErrorHistory[velocityErrorHisIter] = 0;
}

int ultraSoundRead = 0;
void HC_SR04(){
  ultraSoundRead = sonar_forward.ping_cm();
  if(ultraSoundRead == 0 || ultraSoundRead >= 80) ultraSoundRead = 80;
  if(abs(ultraSoundRead - distance_relative) >= 20) ultraSoundRead = (ultraSoundRead+distance_relative)/2;
  ultraSound_history[ultraSound_iteration] = ultraSoundRead;
  ultraSound_iteration++;
  if( ultraSound_iteration == 5 ) ultraSound_iteration = 0;//make the pointer back to the head of the array
  distance_relative = 0;
  for(int i =0; i<5;++i){
    distance_relative += ultraSound_smoother[i]*ultraSound_history[(ultraSound_iteration+1+i)%5];
  }
}
