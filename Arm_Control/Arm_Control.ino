// Arm control 

#include <FlexyStepper.h>

const int MOTOR_STEP_PIN = 2;
const int MOTOR_DIRECTION_PIN = 3;

FlexyStepper stepper;

double x_coord = 0.0; 
double theta; 
double d; 

const int potPin = A0;
float potVal;

void setup() {
  Serial.begin(9600);
  theta = 0;
  d = 20.0; // length of arm, change when we have real arm

  stepper.connectToPins(MOTOR_STEP_PIN, MOTOR_DIRECTION_PIN);

  pinMode(potPin, INPUT);
}

void loop() {
  stepper.setStepsPerRevolution(1600);
  stepper.setSpeedInRevolutionsPerSecond(20);
  stepper.setAccelerationInRevolutionsPerSecondPerSecond(20);

  potVal = analogRead(potPin);
  // Serial.println(potVal);

  // while (Serial.available() == 0) {
  // }

  // x_coord = Serial.parseFloat();
  x_coord = ((40.0 / 1023.0) * potVal) - 20;
  Serial.print("X coordinate: ");
  Serial.print(x_coord);

  theta = acos(x_coord / d);
  theta = theta * 180.0 / PI;
  Serial.print(" Theta: ");
  Serial.println(theta);


  stepper.moveToPositionInRevolutions(theta/360.0);
}