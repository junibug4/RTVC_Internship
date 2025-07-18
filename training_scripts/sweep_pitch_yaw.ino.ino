
#include <Servo.h>

Servo pitchServo;
Servo yawServo;

#define PITCH_SERVO_PIN 9
#define YAW_SERVO_PIN 11

void setup() {
  // put your setup code here, to run once:
  pitchServo.attach(PITCH_SERVO_PIN);
  yawServo.attach(YAW_SERVO_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:

  for (int i=45; i<=135; i++){
    pitchServo.write(i);
    delay(25);}

  for (int i=65; i<=135; i++){
    yawServo.write(i);
    delay(25);}
  
  for (int i=135; i>=65; i--){
    yawServo.write(i);
    delay(25);}

  for (int i=135; i>=45; i--){
    pitchServo.write(i);
    delay(25);}

}
