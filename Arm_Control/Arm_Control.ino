// Arm control 

#include <FlexyStepper.h>

const int MOTOR_STEP_PIN = 2;
const int MOTOR_DIRECTION_PIN = 3;

FlexyStepper stepper;

double x_coord = 0.0; 
double theta = 0; 
double arm_length = 11.75; // length of arm in inches

const int potPin = A0;
float potVal;

void setup() {
  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);
  pinMode(potPin, INPUT);
}

void loop() {
  stepper.setStepsPerRevolution(1600);
  stepper.setSpeedInRevolutionsPerSecond(20);
  stepper.setAccelerationInRevolutionsPerSecondPerSecond(20);

  if (Serial.available() > 0) {
    x_coord = Serial.parseFloat();
    theta = acos(x_coord / arm_length);
    stepper.moveToPositionInRevolutions(theta/(2*PI));
  }
  // x_coord = Serial.parseFloat();
  // Serial.print("X coordinate: ");
  // Serial.print(x_coord);

  // theta = acos(x_coord / d);
  // theta = theta * 180.0 / PI;
  // Serial.print(" Theta: ");
  // Serial.println(theta);
}