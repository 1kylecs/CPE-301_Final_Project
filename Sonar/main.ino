#include <Servo.h>

const int trigPin = 9;
const int echoPin = 10;
const int redPin = 3;
const int greenPin = 2;

// Servo Globals
Servo servo;  // global object for servo
const int servoPin = 6;
int servoPos; // global to track servo angle
bool rotateCCW; // gloabl to track direction of rotation

void setup() {
 
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  digitalWrite(greenPin, LOW);
  digitalWrite(redPin, HIGH);
  Serial.begin(9600);

  setupServo(servoPin);
  setupSonar(trigPin, echoPin);

}

void loop() {
  // get and display distance
  float distance = getDistance(trigPin, echoPin);

  if (distance <= 1.0) {
    digitalWrite(greenPin, HIGH);
    digitalWrite(redPin, LOW);
  } else {
    digitalWrite(greenPin, LOW);
    digitalWrite(redPin, HIGH);
  }
  
  Serial.print("Distance: ");
  Serial.println(distance);

  // rotate servo
  rotateServo();
  
}
