void setupServo(const int servoPin) {
  servo.attach(servoPin);
  servoPos = 0;
  rotateCCW = true;
  servo.write(0); // start at 0 degrees
}

void rotateServo() {
  // check direction
  int offset = (rotateCCW?1:-1); // +1 if rotating CCW, -1 if rotating CW
  offset *= 5;
  servoPos += offset;

  // hit bounds, flip directions
  if (servoPos >= 180 || servoPos <= 0) {rotateCCW = !rotateCCW;}

  servo.write(servoPos);
  delay(40); // give enough time to move into position
}
