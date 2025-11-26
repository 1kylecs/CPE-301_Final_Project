
float getDistance(const int trigPin, const int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float duration = pulseIn(echoPin, HIGH, 30000);
  return (duration*0.001125)/2;  // distance = (time(microseconds) * speedofsound(feet/microsecond))/2
}
