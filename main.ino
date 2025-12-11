/* #include <Servo.h>
#include <LiquidCrystal.h>

//set pin constants
const int trigPin = 9;
const int echoPin = 10;
const int servoPin = 8;
const int potPin = A0;

const int scanLED = 23; //yellow
const int idleLED = 25; //red
const int detectLED = 27; //green
const int errorLED = 29; //blue

const int buzzerPin = 12;

//create servo obj
Servo myservo;

//mapping the lcd
const int lcdRS = 7;
const int lcdEN = 6;
const int lcdD4 = 5;
const int lcdD5 = 4;
const int lcdD6 = 3;
const int lcdD7 = 2;
LiquidCrystal lcd(lcdRS, lcdEN, lcdD4, lcdD5, lcdD6, lcdD7);

//LED enum states
enum DeviceState { IDLE, SCAN, DETECT };
DeviceState currentState = IDLE; //default to idle

void setup() {

  Serial.begin(9600);

  pinMode(scanLED, OUTPUT);
  pinMode(idleLED, OUTPUT);
  pinMode(detectLED, OUTPUT);
  pinMode(errorLED, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  myservo.attach(servoPin);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("IDLE");
}

//rotate servo helper function, if servo angle isnt the same as before returns true
bool rotateServo() {
  int val = analogRead(potPin);
  int angle = map(val, 0, 1023, 0, 180);

  static int lastAngle = -1;
  if(angle != lastAngle) {
    myservo.write(angle);
    lastAngle = angle;
    delay(15);
    return true;
  }
  return false;
}

//distance helper function
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH, 30000);
  return (duration * 0.001125) / 2;
}

void loop() {
  bool servoMoved = rotateServo();
  float distance = getDistance();
  Serial.print("Distance: "); Serial.println(distance);

  DeviceState newState = IDLE;

  if(servoMoved) {
    if(distance > 0.0 && distance < 1.0) {
      newState = DETECT;
    } else {
      newState = SCAN;
    }
  } else {
    newState = IDLE;
  }

  //if state changed, update state otherwise leave as same (had to do it like this or led's and buzzer were really flickery)
  if(newState != currentState) {
    currentState = newState;

    //clear
    digitalWrite(scanLED, LOW);
    digitalWrite(detectLED, LOW);
    digitalWrite(idleLED, LOW);
    digitalWrite(errorLED, LOW);
    digitalWrite(buzzerPin, LOW);

    //switch for different states
    switch(currentState) {
      case IDLE:
        digitalWrite(idleLED, HIGH);
        lcd.setCursor(0, 1); lcd.print("IDLE");
        break;

      case SCAN:
        digitalWrite(scanLED, HIGH);
        lcd.setCursor(0, 1); lcd.print("SCANNING");
        break;

      case DETECT:
        digitalWrite(scanLED, HIGH);
        digitalWrite(detectLED, HIGH);
        digitalWrite(buzzerPin, HIGH);
        lcd.setCursor(0, 1); lcd.print("ITEM DETECTED");
        break;
    }
  }

  delay(50); //50ms delay
}
*/ 
#include <LiquidCrystal.h>
#include <Servo.h>

//set pin constants
const int trigPin = 3;
const int echoPin = 2;
const int servoPin = 4;
const int potPin = A0;

const int scanLED = 11; //yellow
const int offLED = 12; //blue
const int detectLED = 10; //green

const int buzzerPin = 13;

//create servo obj
Servo myservo;
int lastAngle = -1;

//mapping the lcd
const int lcdRS = 47;
const int lcdEN = 45;
const int lcdD4 = 43;
const int lcdD5 = 41;
const int lcdD6 = 39;
const int lcdD7 = 37;
LiquidCrystal lcd(lcdRS, lcdEN, lcdD4, lcdD5, lcdD6, lcdD7);

//LED enum states
enum DeviceState { OFF, SCAN, DETECT };
DeviceState currentState = OFF; //default to off

bool rotateServo() {
  int val = analogRead(potPin);
  int angle = map(val, 0, 1023, 0, 180);

  if(angle != lastAngle) {
    myservo.write(angle);
    lastAngle = angle;
    delay(15);
    return true;
  }
  return false;
}

//distance helper function
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  float duration = pulseIn(echoPin, HIGH, 30000);
  return (duration * 0.001125) / 2;
}

void setup() {
  Serial.begin(9600);
  
  pinMode(scanLED, OUTPUT);
  pinMode(detectLED, OUTPUT);
  pinMode(offLED, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(scanLED, LOW);
  digitalWrite(detectLED, LOW);
  digitalWrite(offLED, LOW);
  digitalWrite(buzzerPin, LOW);

  lcd.begin(16, 2);

  myservo.attach(servoPin);
  rotateServo();
  
}

void loop() {

  bool rotated = rotateServo();

  if (rotated) {
    Serial.println("Change State to OFF after rotating");
    currentState = OFF;
  }
  else {
    if (currentState == OFF) {
      currentState = SCAN;
      Serial.println("Finished moving, change state to scan.");
      
    } else {
      float distance = getDistance();

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Distance: ");
      lcd.print(distance);

      if (distance > 1.0 && currentState == DETECT) {
        currentState = SCAN;
        Serial.println("Stop detecting, moved outside 1ft");
      }
      else if (distance <= 1.0){
        if (currentState != DETECT) {
          currentState = DETECT;
          Serial.println("Moved within a foot, detecting");
        }

        
      }
    }
  }  

  digitalWrite(scanLED, LOW);
  digitalWrite(detectLED, LOW);
  digitalWrite(offLED, LOW);

  lcd.setCursor(0, 1);
  lcd.write("               "); // poor mans clear to only change bottom line
  lcd.setCursor(0,1);


  switch(currentState){
    case OFF:
        digitalWrite(offLED, HIGH);
        lcd.print("OFF");
        break;

    case SCAN:
        digitalWrite(scanLED, HIGH);
        lcd.print("SCAN");
        break;

    case DETECT:
        digitalWrite(detectLED, HIGH);
        lcd.print("DETECT");
        break;
  }

  delay(500);
  
}
