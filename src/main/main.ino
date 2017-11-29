/*
 * main.cpp
 *
 *  Created on: 29 lis 2017
 *      Author: szymon
 */




#define LEFT_FORWARD_PIN   2
#define LEFT_BACKWARD_PIN  4
#define LEFT_PWM_PIN       3

#define RIGHT_FORWARD_PIN  7
#define RIGHT_BACKWARD_PIN 5
#define RIGHT_PWM_PIN      6
void setup() {
  pinMode(LEFT_FORWARD_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_BACKWARD_PIN, OUTPUT);

  pinMode(RIGHT_BACKWARD_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_FORWARD_PIN, OUTPUT);


}

void loop() {
 analogWrite(LEFT_PWM_PIN, 255);
 digitalWrite(LEFT_FORWARD_PIN, HIGH);
 digitalWrite(LEFT_BACKWARD_PIN, LOW);

 analogWrite(RIGHT_PWM_PIN, 255);
 digitalWrite(RIGHT_BACKWARD_PIN, LOW);
 digitalWrite(RIGHT_FORWARD_PIN, HIGH);

 delay(3000);

 analogWrite(LEFT_PWM_PIN, 255);
 digitalWrite(LEFT_FORWARD_PIN, LOW);
 digitalWrite(LEFT_BACKWARD_PIN, HIGH);

 analogWrite(RIGHT_PWM_PIN, 255);
 digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
 digitalWrite(RIGHT_FORWARD_PIN, LOW);

 delay(3000);

analogWrite(LEFT_PWM_PIN, 255);
 digitalWrite(LEFT_FORWARD_PIN, HIGH);
 digitalWrite(LEFT_BACKWARD_PIN, LOW);

 analogWrite(RIGHT_PWM_PIN, 255);
 digitalWrite(RIGHT_BACKWARD_PIN, HIGH);
 digitalWrite(RIGHT_FORWARD_PIN, LOW);

 delay(3000);

 analogWrite(RIGHT_PWM_PIN, 255);
 digitalWrite(RIGHT_BACKWARD_PIN, LOW);
 digitalWrite(RIGHT_FORWARD_PIN, HIGH);

 analogWrite(LEFT_PWM_PIN, 255);
 digitalWrite(LEFT_FORWARD_PIN, LOW);
 digitalWrite(LEFT_BACKWARD_PIN, HIGH);

 delay(3000);

analogWrite(LEFT_PWM_PIN, 255);
 digitalWrite(LEFT_FORWARD_PIN, HIGH);
 digitalWrite(LEFT_BACKWARD_PIN, LOW);


 delay(3000);

 analogWrite(RIGHT_PWM_PIN, 255);
 digitalWrite(RIGHT_BACKWARD_PIN, LOW);
 digitalWrite(RIGHT_FORWARD_PIN, HIGH);
delay(3000);

}
