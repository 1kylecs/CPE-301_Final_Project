#include <LiquidCrystal.h>
#include <Servo.h>

/* HARDWARE REGISTERS */
#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0

#define RDA 0x80
#define TBE 0x20
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int *myUBRR0 = (unsigned int *)0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;

volatile unsigned char *my_ADMUX = (unsigned char *)0x7C;
volatile unsigned char *my_ADCSRB = (unsigned char *)0x7B;
volatile unsigned char *my_ADCSRA = (unsigned char *)0x7A;
volatile unsigned int *my_ADC_DATA = (unsigned int *)0x78;

volatile unsigned char *my_DDRB = (unsigned char *)0x24;
volatile unsigned char *my_PORTB = (unsigned char *)0x25;
volatile unsigned char *my_PINB = (unsigned char *)0x23;

volatile unsigned char *my_DDRD = (unsigned char *)0x2A;
volatile unsigned char *my_PORTD = (unsigned char *)0x2B;
volatile unsigned char *my_PIND = (unsigned char *)0x29;

volatile unsigned char *my_DDRE = (unsigned char *)0x2D;   // DDRE
volatile unsigned char *my_PORTE = (unsigned char *)0x2E;  // PORTE
volatile unsigned char *my_PINE = (unsigned char *)0x2C;   // PINE

volatile unsigned char *my_DDRG = (unsigned char *)0x33;   // DDRG
volatile unsigned char *my_PORTG = (unsigned char *)0x34;  // PORTG
volatile unsigned char *my_PING = (unsigned char *)0x32;   // PING

/* CONSTANTS */
const int trigPin = 3;
const int echoPin = 2;
const int servoPin = 4;
const int buttonPin = 5;
const int potPin = 0;

const int scanLED = 11;    //yellow
const int offLED = 12;     //blue
const int detectLED = 10;  //green

const int buzzerPin = 13;

//millis
unsigned long lastPrint = 0;
static unsigned long lastUpdate = 0;

//servo globals for rotateServo()
unsigned long lastServoUpdate = 0;
const unsigned long servoDelay = 15;  //to replace delay(15) with just millis()

//create servo obj
Servo myservo;
int lastAngle = -1;

//mapping the lcd
LiquidCrystal lcd(47, 45, 43, 41, 39, 37);

//STATE enum
enum DeviceState { OFF,
                   SCAN,
                   DETECT };
//default states to off for
DeviceState currentState = OFF;  //default to off
DeviceState lastState = OFF;     //default to off


/* MAIN FUNCTIONS */
void setup() {
  U0init(9600);
  adc_init();

  myPinMode(scanLED, OUTPUT);
  myPinMode(detectLED, OUTPUT);
  myPinMode(offLED, OUTPUT);
  myPinMode(buzzerPin, OUTPUT);
  myPinMode(buttonPin, INPUT);

  myPinMode(trigPin, OUTPUT);
  myPinMode(echoPin, INPUT);

  myDigitalWrite(scanLED, LOW);
  myDigitalWrite(detectLED, LOW);
  myDigitalWrite(offLED, LOW);
  myDigitalWrite(buzzerPin, LOW);

  lcd.begin(16, 2);

  myservo.attach(servoPin);
}

void loop() {
    bool buttonClicked = (HIGH == myDigitalRead(buttonPin);

    switch (currentState) {
        case OFF: {
            // blue LED on, others off
            myDigitalWrite(scanLED, LOW);
            myDigitalWrite(detectLED, LOW);
            myDigitalWrite(offLED, HIGH);

            // read potentiometer and move servo
            rotateServo();

            // display angle on LCD
            lcd.setCursor(0, 0);
            lcd.print("Angle: ");
            lcd.print(lastAngle);

            // transition to SCAN if button clicked
            if (buttonClicked) {
                currentState = SCAN;
                U0println("Button clicked, transitioning into scan, locking servo");
            }
            break;
        }

        case SCAN: {
            // yellow LED on, others off
            myDigitalWrite(scanLED, HIGH);
            myDigitalWrite(detectLED, LOW);
            myDigitalWrite(offLED, LOW);

            // measure distance
            float distance = getDistance();

            // display distance
            lcd.setCursor(0, 0);
            lcd.print("Distance: ");
            lcd.print(distance);

            // transition to DETECT if distance < 5 but not 0 ( 0 means time out, so also invalid)
            if (distance > 0 && distance < 5.0) {
                currentState = DETECT;
                U0println("Moved within 5ft of scanner, detected");
            }

            // transition to OFF if button clicked
            if (buttonClicked) {
                currentState = OFF;
                U0prinln("Button pressed, turning sonar off, able to rotate servo");
            }
            break;
        }

        case DETECT: {
            // green LED on, others off
            myDigitalWrite(scanLED, LOW);
            myDigitalWrite(detectLED, HIGH);
            myDigitalWrite(offLED, LOW);

            // measure distance
            float distance = getDistance();

            // display distance
            lcd.setCursor(0, 0);
            lcd.print("Distance: ");
            lcd.print(distance);

            // transition to SCAN if distance > 5
            if (distance > 5.0) {
                currentState = SCAN;
                U0println("Moved outside 5ft, scanning");
                
            }

            // transition to OFF if button clicked
            if (buttonClicked) {
                currentState = OFF;
                U0prinln("Button pressed, turning sonar off, able to rotate servo");
            }
            break;
        }
    }

    delay(200); // short delay to debounce button and reduce flicker
}


/* HELPER FUNCTIONS */
bool rotateServo() {
  int val = adc_read(potPin);
  int angle = map(val, 0, 1023, 0, 180);

  //use millis() instead of delay() so program doesnt pause on rotate
  unsigned long now = millis();
  if (angle != lastAngle && (now - lastServoUpdate >= 15)) {
    myservo.write(angle);
    lastAngle = angle;
    lastServoUpdate = now;  //update var
    return true;
  }
  return false;
}

//distance helper function
float getDistance() {
  myDigitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  myDigitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  myDigitalWrite(trigPin, LOW);

  float duration = myPulseIn(echoPin, HIGH, 60000);
  return (duration * 0.001125) / 2;
}


/* GIPO */
void U0init(unsigned long U0baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0 = tbaud;
}

void U0putchar(unsigned char U0pdata) {
  while (!(*myUCSR0A & TBE))
    ;

  *myUDR0 = U0pdata;
}

//print function that uses the previous U0putchar to make strings
void U0println(const char *s) {
  while (*s) {
    U0putchar(*s);
    s++;
  }
  U0putchar('\n');
}

void debugPrint(const char *msg) {
  if (millis() - lastPrint >= 300) {  // print every 300ms max
    U0println(msg);
    lastPrint = millis();
  }
}


/* CUSTOM digitalWrite / pinMode */

void myPinMode(uint8_t pin, uint8_t mode) {
  // take what pin number given, find the right DDR and set to either 0 or 1
  switch (pin) {
    case 2:  // PE4
      if (mode) *my_DDRE |= (1 << 4);
      else *my_DDRE &= ~(1 << 4);
      break;

    case 3:  // PE5
      if (mode) *my_DDRE |= (1 << 5);
      else *my_DDRE &= ~(1 << 5);
      break;

    case 4:  // PG5
      if (mode) *my_DDRG |= (1 << 5);
      else *my_DDRG &= ~(1 << 5);
      break;

    case 10:  // PB4
      if (mode) *my_DDRB |= (1 << 4);
      else *my_DDRB &= ~(1 << 4);
      break;

    case 11:  // PB5
      if (mode) *my_DDRB |= (1 << 5);
      else *my_DDRB &= ~(1 << 5);
      break;

    case 12:  // PB6
      if (mode) *my_DDRB |= (1 << 6);
      else *my_DDRB &= ~(1 << 6);
      break;

    case 13:  // PB7
      if (mode) *my_DDRB |= (1 << 7);
      else *my_DDRB &= ~(1 << 7);
      break;
  }
}

void myDigitalWrite(uint8_t pin, uint8_t val) {
  // take what pin number given, find the right port and set to either 0 or 1
  switch (pin) {
    case 2:
      if (val) *my_PORTE |= (1 << 4);
      else *my_PORTE &= ~(1 << 4);
      break;

    case 3:
      if (val) *my_PORTE |= (1 << 5);
      else *my_PORTE &= ~(1 << 5);
      break;

    case 4:
      if (val) *my_PORTG |= (1 << 5);
      else *my_PORTG &= ~(1 << 5);
      break;

    case 10:
      if (val) *my_PORTB |= (1 << 4);
      else *my_PORTB &= ~(1 << 4);
      break;

    case 11:
      if (val) *my_PORTB |= (1 << 5);
      else *my_PORTB &= ~(1 << 5);
      break;

    case 12:
      if (val) *my_PORTB |= (1 << 6);
      else *my_PORTB &= ~(1 << 6);
      break;

    case 13:
      if (val) *my_PORTB |= (1 << 7);
      else *my_PORTB &= ~(1 << 7);
      break;
  }
}

uint8_t myDigitalRead(uint8_t pin) {
  switch (pin) {
    case 2: return (*my_PINE & (1 << 4)) ? HIGH : LOW;
    case 3: return (*my_PINE & (1 << 5)) ? HIGH : LOW;
    case 4: return (*my_PING & (1 << 5)) ? HIGH : LOW;

    case 10: return (*my_PINB & (1 << 4)) ? HIGH : LOW;
    case 11: return (*my_PINB & (1 << 5)) ? HIGH : LOW;
    case 12: return (*my_PINB & (1 << 6)) ? HIGH : LOW;
    case 13: return (*my_PINB & (1 << 7)) ? HIGH : LOW;
  }
  return LOW;
}



/* ANALOG READ */
/* ANALOG READ */
void adc_init() {
  // setup the A register
  // set bit   7 to 1 to enable the ADC
  *my_ADCSRA |= 0x80;

  // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= ~0x40;

  // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= ~0x08;

  // clear bit 0-2 to 0 to set prescaler selection to slow reading
  *my_ADCSRA &= ~0x07;

  // setup the B register
  // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= ~0x08;

  // clear bit 2-0 to 0 to set free running mode
  *my_ADCSRB &= ~0x07;

  // setup the MUX Register
  // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX &= ~0x80;

  // set bit 6 to 1 for AVCC analog reference
  *my_ADMUX |= 0x40;

  // clear bit 5 to 0 for right adjust result
  *my_ADMUX &= ~0x20;

  // clear bit 4-0 to 0 to reset the channel and gain bits
  *my_ADMUX &= ~0x1F;
}
unsigned int adc_read(unsigned char adc_channel_num)  //work with channel 0
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= ~0x1F;

  // clear the channel selection bits (MUX 5) hint: it's not in the ADMUX register
  *my_ADCSRB &= ~0x08;

  // set the channel selection bits for channel 0
  *my_ADMUX |= adc_channel_num;

  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;

  // wait for the conversion to complete
  while ((*my_ADCSRA & 0x40) != 0)
    ;

  // return the result in the ADC data register and format the data based on right justification (check the lecture slide)
  unsigned int val = *my_ADC_DATA;
  return val;
}

//pulseIn to work with getDistance() and myDigitalRead()
unsigned long myPulseIn(uint8_t pin, uint8_t state, unsigned long timeout) {
  unsigned long start = micros();

  // Wait for pin to go to desired state
  while (myDigitalRead(pin) != state) {
    if (micros() - start > timeout) return 0;
  }

  unsigned long pulseStart = micros();

  // Wait for pin to leave desired state
  while (myDigitalRead(pin) == state) {
    if (micros() - pulseStart > timeout) return 0;
  }

  return micros() - pulseStart;
}
