#include <Servo.h>

Servo myservo;  // create servo object to control a servo


void setup() {
  myservo.attach(7);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600);
  myservo.writeMicroseconds (1000);
}

void loop() {
  if(Serial.available()!=0){
    char a = Serial.read();
    if(a == 'A'){
      myservo. writeMicroseconds (1100);
      Serial.println("123456789");
    delay(20);
    }
    else if(a == 'B'){
      myservo. writeMicroseconds (1150);
    delay(20);
    }
    else if(a == 'C'){
      myservo. writeMicroseconds (1200);
    delay(20);
    }
    else if(a == 'D'){
      myservo. writeMicroseconds (1250);
    delay(20);
    }
    else if(a == 'E'){
      myservo. writeMicroseconds (1300);
    delay(20);
    }
    else if(a == 'F'){
      myservo. writeMicroseconds (1350);
    delay(20);
    }
    else if(a == 'G'){
      myservo. writeMicroseconds (1400);
    delay(20);
    }
    else if(a == 'H'){
      myservo. writeMicroseconds (1500);
    delay(20);
    }
    else if(a == 'I'){
      myservo. writeMicroseconds (1550);
    delay(20);
    }
    else if(a == 'J'){
      myservo. writeMicroseconds (1600);
    delay(20);
    }
    else if(a == 'S'){
      myservo. writeMicroseconds (1000);
    delay(20);
    }
  }
}
