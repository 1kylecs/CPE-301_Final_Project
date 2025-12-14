// Miles Rasmussen
// Kyle Kaiser

#include <LiquidCrystal.h>
#include <Servo.h>
#include <Wire.h>
#include <RTClib.h>


/* HARDWARE REGISTERS */
#define OUTPUT 1
#define INPUT 0
#define HIGH 1
#define LOW 0

#define RDA 0x80
#define TBE 0x20
#define DEBOUNCE_TICKS 40

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

/*TIMER Pointers*/ //using timer2 because servo library uses timer1
volatile unsigned char *my_TCCR2A = (unsigned char *)0xB0;
volatile unsigned char *my_TCCR2B = (unsigned char *)0xB1;
volatile unsigned char *my_TIMSK2 = (unsigned char *)0x70;
volatile unsigned char *my_OCR2A  = (unsigned char *)0xB3;


/* CONSTANTS */
const int trigPin = 3;
const int echoPin = 2;
const int servoPin = 4;
const int potPin = 0;

const int scanLED = 11;    //yellow
const int offLED = 12;     //blue
const int detectLED = 10;  //green

const int buzzerPin = 13;

const int buttonPin = 5;

//millis
unsigned long lastPrint = 0;
static unsigned long lastUpdate = 0;

//servo globals for rotateServo()
unsigned long lastServoUpdate = 0;
const unsigned long servoDelay = 15;  //to replace delay(15) with just millis()

//loop delay interval
unsigned long lastLoopUpdate = 0;       // tracks last update time
const unsigned long loopInterval = 50;  // 50ms interval instead of delay()

//button
volatile bool buttonEvent = false;
volatile bool buttonStableState = HIGH;
volatile bool lastButtonSample = HIGH;
volatile unsigned int debounceCounter = 0;

//create servo obj
Servo myservo;
int lastAngle = -1;

//create rtc object
RTC_DS3231 rtc;

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

  //button ISR setup
  myPinMode(buttonPin, INPUT);
  *my_PORTE |= (1 << 3);  //pull-up enabled

  setupButtonTimer();
  sei();


  //setup rtc for timestamps
  Wire.begin();
  if (!rtc.begin()) {
    U0println("RTC not found!");
    while (1)
      ;
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));  //updates current time

  myPinMode(scanLED, OUTPUT);
  myPinMode(detectLED, OUTPUT);
  myPinMode(offLED, OUTPUT);
  myPinMode(buzzerPin, OUTPUT);

  myPinMode(trigPin, OUTPUT);
  myPinMode(echoPin, INPUT);

  myDigitalWrite(scanLED, LOW);
  myDigitalWrite(detectLED, LOW);
  myDigitalWrite(offLED, LOW);
  myDigitalWrite(buzzerPin, LOW);

  lcd.begin(16, 2);

  myservo.attach(servoPin);
  rotateServo();
}

void loop() {
  //button isr
  bool buttonClicked = false;

  if (buttonEvent) {
    buttonClicked = true;
    buttonEvent = false;
  }

  switch (currentState) {
    case OFF:
      {
        //leds
        myDigitalWrite(scanLED, LOW);
        myDigitalWrite(detectLED, LOW);
        myDigitalWrite(offLED, HIGH);
        myDigitalWrite(buzzerPin, LOW);

        //rotates servo
        rotateServo();

        //dispaly angle on first row
        lcd.setCursor(0, 0);
        lcd.print("Angle: ");
        lcd.print(lastAngle);
        //fill rest of row with spaces
        lcd.print("        ");  //enough spaces to fill 16 chars

        //sets status in second row
        lcd.setCursor(0, 1);
        lcd.print("OFF, NO SCAN    ");  //pad with spaces to clear row

        if (buttonClicked) {
          currentState = SCAN;
          logMessage("Button clicked, transitioning into scan, locking servo");
        }
        break;
      }

    case SCAN:
      {

        myDigitalWrite(scanLED, HIGH);
        myDigitalWrite(detectLED, LOW);
        myDigitalWrite(offLED, LOW);
        myDigitalWrite(buzzerPin, LOW);

        float distance = getDistance();


        lcd.setCursor(0, 0);
        lcd.print("Distance: ");
        lcd.print(distance, 2);  //print to 2 dec accuracy
        lcd.print("     ");      //pad with spaces to clear row


        lcd.setCursor(0, 1);
        lcd.print("SCANNING...     ");

        if (distance > 0 && distance <= 1.0) {
          currentState = DETECT;
          logMessage("Moved within 1ft of scanner, detected");
        }

        if (buttonClicked) {
          currentState = OFF;
          logMessage("Button pressed, turning sonar off, able to rotate servo");
        }
        break;
      }

    case DETECT:
      {

        myDigitalWrite(scanLED, LOW);
        myDigitalWrite(detectLED, HIGH);
        myDigitalWrite(offLED, LOW);
        myDigitalWrite(buzzerPin, HIGH);

        float distance = getDistance();


        lcd.setCursor(0, 0);
        lcd.print("Distance: ");
        lcd.print(distance, 2);
        lcd.print("     ");  //pad to clear


        lcd.setCursor(0, 1);
        lcd.print("DETECTED!       ");  //pad to clear

        if (distance > 1.0) {
          currentState = SCAN;
          logMessage("Moved outside 1ft, scanning");
        }

        if (buttonClicked) {
          currentState = OFF;
          logMessage("Button pressed, turning sonar off, able to rotate servo");
        }
        break;
      }
  }

  // non-blocking loop interval
  if (millis() - lastLoopUpdate >= loopInterval) {
    lastLoopUpdate = millis();
  }
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

    case 5:  // PE3
      if (mode) *my_DDRE |= (1 << 3);
      else *my_DDRE &= ~(1 << 3);
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

    case 5:
      if (val) *my_PORTE |= (1 << 3);
      else *my_PORTE &= ~(1 << 3);
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
    case 5: return (*my_PINE & (1 << 3)) ? HIGH : LOW;

    case 10: return (*my_PINB & (1 << 4)) ? HIGH : LOW;
    case 11: return (*my_PINB & (1 << 5)) ? HIGH : LOW;
    case 12: return (*my_PINB & (1 << 6)) ? HIGH : LOW;
    case 13: return (*my_PINB & (1 << 7)) ? HIGH : LOW;
  }
  return LOW;
}

// //button helper function for debounce with millis()
// bool checkButton() {
//   bool reading = myDigitalRead(buttonPin);
//   unsigned long now = millis();

//   // If button changed state
//   if (reading != lastButtonState) {
//     lastButtonTime = now;  // reset debounce timer
//   }

//   // Only consider the press if it’s stable past debounce time
//   if ((now - lastButtonTime) > buttonDebounce) {
//     if (reading == HIGH && !buttonPressed) {
//       buttonPressed = true;  // single press detected
//       lastButtonState = reading;
//       return true;
//     } else if (reading == LOW) {
//       buttonPressed = false;  // release detected
//     }
//   }

//   lastButtonState = reading;
//   return false;
// }

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

  //waits for pin to read a certain state
  while (myDigitalRead(pin) != state) {
    if (micros() - start > timeout) return 0;
  }

  unsigned long pulseStart = micros();

  //wait for pin to leave state
  while (myDigitalRead(pin) == state) {
    if (micros() - pulseStart > timeout) return 0;
  }

  return micros() - pulseStart;
}

/* --- RTC TIMESTAMP LOGGING --- */
void twoDigitStr(int num, char *buf) {
  buf[0] = '0' + num / 10;
  buf[1] = '0' + num % 10;
  buf[2] = 0;
}

void printTimestamp() {
  DateTime now = rtc.now();
  char buf[3];

  U0putchar('[');

  //hour
  twoDigitStr(now.hour(), buf);
  U0putchar(buf[0]);
  U0putchar(buf[1]);
  U0putchar(':');

  //minutes
  twoDigitStr(now.minute(), buf);
  U0putchar(buf[0]);
  U0putchar(buf[1]);
  U0putchar(':');

  //seconds
  twoDigitStr(now.second(), buf);
  U0putchar(buf[0]);
  U0putchar(buf[1]);

  U0putchar(']');
  U0putchar(' ');
}

//prints message with timestamp
void logMessage(const char *msg) {
  printTimestamp();
  U0println(msg);
}

/*Button ISR*/
void setupButtonTimer()
{
  *my_TCCR2A = (1 << 1);
  *my_TCCR2B = 0x00;
  *my_OCR2A = 124; //5ms
  *my_TIMSK2 |= (1 << 1); //enable interrupts
  *my_TCCR2B |= (1 << 2) | (1 << 0); //128 prescaler
}


//CTC ISR
ISR(TIMER2_COMPA_vect)
{
  bool sample = myDigitalRead(buttonPin);

  if (sample == lastButtonSample)
  {
    if (debounceCounter < DEBOUNCE_TICKS)
      debounceCounter++;
    else
    {
      if (sample != buttonStableState)
      {
        buttonStableState = sample;
        if (buttonStableState == LOW)
          buttonEvent = true;
      }
    }
  }
  else
  {
    debounceCounter = 0;
  }

  lastButtonSample = sample;
}
