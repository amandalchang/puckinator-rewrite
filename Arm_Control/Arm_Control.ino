// Arm control

#include <FlexyStepper.h>

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
}

void loop() {
  stepper.setStepsPerRevolution(1600 * 3);
  stepper.setSpeedInRevolutionsPerSecond(1);
  stepper.setAccelerationInRevolutionsPerSecondPerSecond(1);

  while (Serial.available() > 0) {
    // read the first byte in the serial buffer
    char inputChar = Serial.peek();
    
    if (inputChar == 'c') {
      // If it's 'c', set the stepper to zero and consume the character
      Serial.read(); // consume the 'c' character
      Serial.println("Setting stepper position to zero.");
      stepper.setCurrentPositionInRevolutions(0);
    } else {
      // Otherwise, parse the float number and perform the calculations
      float x_coord = Serial.parseFloat();
      Serial.println("Received x coordinate: ");
      Serial.println(x_coord);
      float theta = acos(x_coord / arm_length);
      Serial.println("Moving to position in revolutions: ");
      Serial.println(theta / (2 * PI));
      stepper.setTargetPositionInRevolutions(theta / (2 * PI));
    }
  }

  while(!stepper.motionComplete())
  {
    stepper.processMovement();       // this call moves the motor
  }
  // x_coord = Serial.parseFloat();
  // Serial.print("X coordinate: ");
  // Serial.print(x_coord);

  // theta = acos(x_coord / d);
  // theta = theta * 180.0 / PI;
  // Serial.print(" Theta: ");
  // Serial.println(theta);
}