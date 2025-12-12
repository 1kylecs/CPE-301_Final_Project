#include <LiquidCrystal.h>
#include <Servo.h>

/* CONSTANTS */
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
LiquidCrystal lcd(47, 45, 43, 41, 39, 37);

//LED enum states
enum DeviceState { OFF, SCAN, DETECT };
DeviceState currentState = OFF; //default to off

/* MAIN FUNCTIONS */
void setup() {
  U0init(9600);
  
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
    U0print("Change State to OFF after rotating");
    currentState = OFF;
  }
  else {
    if (currentState == OFF) {
      currentState = SCAN;
      U0print("Finished moving, change state to scan.");
      
    } else {
      float distance = getDistance();

      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Distance: ");
      lcd.print(distance);

      if (distance > 1.0 && currentState == DETECT) {
        currentState = SCAN;
        U0print("Stop detecting, moved outside 1ft");
      }
      else if (distance <= 1.0){
        if (currentState != DETECT) {
          currentState = DETECT;
          U0print("Moved within a foot, detecting");
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



/* HELPER FUNCTIONS */
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




/* GIPO */
void U0init(unsigned long U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

void U0putchar(unsigned char U0pdata)
{
  while (!(*myUCSR0A & TBE));

 *myUDR0 = U0pdata;
}

//print function that uses the previous U0putchar to make strings
void U0print(const char* s) 
{
  while (*s) 
  { 
    U0putchar(*s);       
    s++;
  }
}

/* CUSTOM digitalWrite / pinMode */

void myPinMode(const int pin) {
  
}

void myDigitalWrite(const int pin, const bool high) {
  
}

/* ANALOG READ */

