#include <FlexyStepper.h>

const int MOTOR_STEP_PIN1 = 2;
const int MOTOR_DIRECTION_PIN1 = 3;

const int MOTOR_STEP_PIN2 = 5;
const int MOTOR_DIRECTION_PIN2 = 6;

FlexyStepper stepper1;

FlexyStepper stepper2;


double x_coord = 0.0;
double theta = 0;
double arm_length = 11.75;  // length of arm in inches

const int LIMIT_SWITCH_PIN_1 = 9;
const int LIMIT_SWITCH_PIN_2 = 10;

// how much serial data we expect before a newline
const unsigned int MAX_INPUT = 50;

#define DEBUG
#define HOME

void setup() {
  // put your setup code here, to run once:
  stepper1.connectToPins(MOTOR_STEP_PIN1, MOTOR_DIRECTION_PIN1);
  stepper2.connectToPins(MOTOR_STEP_PIN2, MOTOR_DIRECTION_PIN2);

  Serial.begin(115200);

  stepper1.setStepsPerRevolution(1600 * 3);
  stepper1.setSpeedInRevolutionsPerSecond(5);
  stepper1.setAccelerationInRevolutionsPerSecondPerSecond(10);

  stepper2.setStepsPerRevolution(1600 * 3);
  stepper2.setSpeedInRevolutionsPerSecond(5);
  stepper2.setAccelerationInRevolutionsPerSecondPerSecond(10);

#ifdef HOME
  if (stepper1.moveToHomeInRevolutions(1, 0.1, 10, LIMIT_SWITCH_PIN_1) != true) {
    while (true) {
      digitalWrite(11, HIGH);
    }
  } else {
    stepper1.setCurrentPositionInRevolutions(0.13);
    stepper1.setTargetPositionInRevolutions(0);
  }

  if (stepper2.moveToHomeInRevolutions(-1, 0.1, 10, LIMIT_SWITCH_PIN_2) != true) {
    while (true) {
      digitalWrite(11, HIGH);
    }
  } else {
    stepper2.setCurrentPositionInRevolutions(-0.17);
    stepper2.setTargetPositionInRevolutions(0);
  }
#endif
}


// here to process incoming serial data after a terminator received
void process_data(const char *data) {
#ifdef DEBUG
  Serial.println(data);  // Display the received data
#endif

  if (data[0] == 'i' && data[1] == '\0') {  // Check if the data is 'i'
    stepper1.setCurrentPositionInRevolutions(0.0);
    stepper1.setTargetPositionInRevolutions(0.0);

    stepper2.setCurrentPositionInRevolutions(0.0);
    stepper2.setTargetPositionInRevolutions(0.0);
  } else {
    // Tokenize the string and set the positions of the steppers
    char *token = strtok((char *)data, ",");  // Cast to non-const char* for strtok
    if (token != NULL) {
      float firstNumber = atof(token);  // Convert the first token to a float
      stepper1.setTargetPositionInRevolutions(firstNumber / (2.0 * PI));
#ifdef DEBUG
      Serial.print("Moving stepper one to position: ");
      Serial.println(firstNumber / (2.0 * PI));
#endif
      token = strtok(NULL, ",");  // Get the next token
      if (token != NULL) {
        float secondNumber = atof(token);  // Convert the second token to a float
        stepper2.setTargetPositionInRevolutions(secondNumber / (2.0 * PI));
#ifdef DEBUG
        Serial.print("Moving stepper two to position: ");
        Serial.println(secondNumber / (2.0 * PI));
#endif
      }
    }
  }
}


void processIncomingByte(const byte inByte) {
  static char input_line[MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte) {
    case '\n':                    // end of text
      input_line[input_pos] = 0;  // terminating null byte

      // terminator reached! process input_line here ...
      process_data(input_line);

      // reset buffer for next time
      input_pos = 0;
      break;

    case '\r':  // discard carriage return
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_INPUT - 1))
        input_line[input_pos++] = inByte;
      break;

  }  // end of switch

}  // end of processIncomingByte

void loop() {
  while (Serial.available() > 0) {
    processIncomingByte(Serial.read());
    // read the first byte in the serial buffer
  }

  if (!stepper1.motionComplete()) {
    stepper1.processMovement();  // this call moves the motor
  }


  if (!stepper2.motionComplete()) {
    stepper2.processMovement();  // this call moves the motor
  }
}
