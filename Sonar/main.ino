const int trigPin = 9;
const int echoPin = 10;
const int redPin = 3;
const int greenPin = 4;

float distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  digitalWrite(greenPin, LOW);
  digitalWrite(redPin, HIGH);
  Serial.begin(9600);

}

void loop() {
  distance = getDistance(trigPin, echoPin);

  if (distance <= 1.0) {
    digitalWrite(greenPin, HIGH);
    digitalWrite(redPin, LOW);
  } else {
    digitalWrite(greenPin, LOW);
    digitalWrite(redPin, HIGH);
  }
  
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);
}
