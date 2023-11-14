// Arm control

#include <FlexyStepper.h>

//#define DEBUG

const int MOTOR_STEP_PIN = 2;
const int MOTOR_DIRECTION_PIN = 3;

FlexyStepper stepper;

double x_coord = 0.0;
double theta = 0;
double arm_length = 11.75;  // length of arm in inches

const int potPin = A0;
float potVal;

void setup() {
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  pinMode(potPin, INPUT);
  Serial.begin(115200);
  stepper.setStepsPerRevolution(1600 * 3);
  stepper.setSpeedInRevolutionsPerSecond(5);
  stepper.setAccelerationInRevolutionsPerSecondPerSecond(10);
  pinMode(11, OUTPUT);
  digitalWrite(11, LOW);
}

void loop() {
  while (Serial.available() > 0) {
    // read the first byte in the serial buffer
    char inputChar = Serial.peek();

    if (inputChar == 'c') {
      // If it's 'c', set the stepper to zero and consume the character
      Serial.read();  // consume the 'c' character
      #ifdef DEBUG
        Serial.println("Setting stepper angle to zero, pos to 11.75");
      #endif
      stepper.setTargetPositionInRevolutions(0.25);
      stepper.setCurrentPositionInRevolutions(0.25);
    } else {
      // Otherwise, parse the float number and perform the calculations
      // float x_coord = Serial.parseFloat();
      String x_coordstring = Serial.readStringUntil('\n');
      float x_coord = x_coordstring.toFloat();
      // if (x_coord < 10.00) {
      //   digitalWrite(11, HIGH);
      // } else {
      //   digitalWrite(11, LOW);
      // }
      // if (x_coordstring == "10.00") {
      //   digitalWrite(13, HIGH);
      // } else {
      //   digitalWrite(13, LOW);
      // }
      #ifdef DEBUG
        Serial.println("Received x coordinate: ");
        Serial.println(x_coord);
      #endif
      float theta = acos(x_coord / arm_length);
      #ifdef DEBUG
        Serial.println("Moving to position in revolutions: ");
        Serial.println(theta / (2 * PI));
      #endif
      stepper.setTargetPositionInRevolutions(theta / (2 * PI));
    }
  }

  if(!stepper.motionComplete())
  {
    stepper.processMovement();       // this call moves the motor
  }
}