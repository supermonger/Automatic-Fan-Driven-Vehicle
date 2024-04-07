
#include<Servo.h>
Servo Brushless;
int initial;
int rate = 1200;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Brushless.attach(9,1000,2000);
  pinMode(9,OUTPUT);
  

}

/*void initialization()//初始化,上限180 下限15
{
  Brushless.writeMicroseconds(2000);
  delay(2000);
  Brushless.writeMicroseconds(700);
  delay(2000);
  initial = 1;
}*/

void loop() {
  // put your main code here, to run repeatedly:
  rate = Serial.read();
  Serial.println(rate);
  Brushless.writeMicroseconds(rate);
  delay(3000);
  
  /*Brushless.writeMicroseconds(1300);
  delay(3000);
  Brushless.writeMicroseconds(1500);
  delay(3000);
  /*Brushless.writeMicroseconds(1700);
  delay(3000);
  Brushless.writeMicroseconds(1900);
  delay(3000);*/

}
