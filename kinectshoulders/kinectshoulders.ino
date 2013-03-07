#include <Servo.h> 

// declare both servos
Servo shoulderLeft;
Servo shoulderRight;

// setup the array of servo positions ❷
int nextServo = 0;
int servoAngles[] = {0,0};

void setup() {
  // attach servos to their pins ❸
  shoulderLeft.attach(10);
  shoulderRight.attach(6);
  Serial.begin(9600);
}

void loop() {
  if(Serial.available()){ 
    int servoAngle = Serial.read();

    servoAngles[nextServo] = servoAngle;
    nextServo++;
    if(nextServo > 1){
      nextServo = 0;
    }

    shoulderLeft.write(servoAngles[0]); 
   shoulderRight.write(servoAngles[1]);
    Serial.println(servoAngles[1]); 

  }
}
