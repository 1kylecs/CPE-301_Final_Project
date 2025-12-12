#include <LiquidCrystal.h>
#include <Servo.h>

/* HARDWARE REGISTERS */
#define RDA 0x80
#define TBE 0x20
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;
 
volatile unsigned char *my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char *my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char *my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int *my_ADC_DATA = (unsigned int*) 0x78;

volatile unsigned char *my_DDRB  = (unsigned char *)0x24;
volatile unsigned char *my_PORTB = (unsigned char *)0x25;
volatile unsigned char *my_PINB  = (unsigned char *)0x23;

volatile unsigned char *my_DDRD  = (unsigned char *)0x2A;
volatile unsigned char *my_PORTD = (unsigned char *)0x2B;
volatile unsigned char *my_PIND  = (unsigned char *)0x29;

volatile unsigned char *my_DDRH  = (unsigned char *)0x31;
volatile unsigned char *my_PORTH = (unsigned char *)0x30;
volatile unsigned char *my_PINH  = (unsigned char *)0x2F;


/* Pin lables
 *  echoPin   - pin 2  - regD bit 2
 *  trigPin   - pin 3  - regD bit 3
 *  
 *  button    - pin 4  - regD bit 4
 *  
 *  greenLED  - pin 10 - regB bit 4
 *  yellowLED - pin 11 - regB bit 5
 *  blueLED   - pin 12 - regB bit 6
 *  buzzer    - pin 13 - regB but 7
 */



/* CONSTANTS */


const int potPin = 0;
const int echoPin = 2; // need for pulseIn later
const int trigPin = 3;

//create servo obj
Servo myservo;
const int servoPin = 53;
int lastAngle = -1;

//mapping the lcd
LiquidCrystal lcd(47, 45, 43, 41, 39, 37);

//STATE enum
enum DeviceState { OFF, SCAN, DETECT };
DeviceState currentState = OFF; //default to off


/* MAIN FUNCTIONS */
void setup() {
  U0init(9600);
  
  // set output for LEDs, Buzzer
  *my_DDRB |= (0b11110000);

  // button is input
  //*my_DDRD &= ~(1 << 4);
  pinMode(4, INPUT);

  // set trig as output
  *my_DDRD |= (1 << 3);

  // set echo as input
  *my_DDRD &= ~(1<< 2);
  

  // start all outputs as low
  *my_PORTB &= ~(0b11110000);
  *my_PORTD &= ~(1 << 3);

  adc_init();

  // setup LCD and servo
  lcd.begin(16, 2);

  myservo.attach(servoPin);
  rotateServo();
  
}

void loop() {
    bool buttonClicked = (HIGH == digitalRead(4));

    switch (currentState) {
        case OFF: {
            // blue LED on, others off
            *my_PORTB &= ~(0b01110000);
            *my_PORTB |= (1 << 6);

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
            *my_PORTB &= ~(0b01110000);
            *my_PORTB |= (1 << 5);

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
            *my_PORTB &= ~(0b01110000);
            *my_PORTB |= (1 << 4);

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
  lcd.setCursor(0,0);
  lcd.print("Pot.: ");
  lcd.print(val);
  
  int angle = map(val, 0, 1023, 0, 180);
  lcd.setCursor(0,1);
  lcd.print("Angle: ");
  lcd.print(angle);

  if(angle != lastAngle) {
    myservo.write(angle);
    lastAngle = angle;
    delay(20);
    return true;
  }
  return false;
}

//distance helper function
float getDistance() {
    // trigger the sensor
    *my_PORTD &= ~(1 << 3);  // trig LOW
    delayMicroseconds(2);
    *my_PORTD |= (1 << 3);   // trig HIGH
    delayMicroseconds(10);
    *my_PORTD &= ~(1 << 3);  // trig LOW

    float duration = pulseIn(echoPin, HIGH, 30000);

    // convert duration to distance in feet
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
void U0println(const char* s) 
{
  while (*s) 
  { 
    U0putchar(*s);       
    s++;
  }
  U0putchar('\n');
}


/* ANALOG READ */
/* ANALOG READ */
void adc_init()
{
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
unsigned int adc_read(unsigned char adc_channel_num) //work with channel 0
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
  while((*my_ADCSRA & 0x40) != 0);

  // return the result in the ADC data register and format the data based on right justification (check the lecture slide)
  unsigned int val = *my_ADC_DATA;
  return val;
}
