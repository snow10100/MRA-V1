#include <AccelStepper.h>
#include <Servo.h>


// Number of motors
const int numMotors = 6;

// Define motor pins
const int motorPins[numMotors][2] = {
  {2, 3},  // Motor 1: STEP pin 2, DIR pin 3
  {4, 5},  // Motor 2: STEP pin 4, DIR pin 5
  {6, 7},  // Motor 3: STEP pin 6, DIR pin 7
  {8, 9},  // Motor 4: STEP pin 8, DIR pin 9
  {10, 11}, // Motor 5: STEP pin 10, DIR pin 11
  {12, 13} // Motor 6: STEP pin 12, DIR pin 13
};

const int servoClosePosition = 0; // 5
const int servoOpenPosition = 180; // 165
int pos = 5;
int servoCurrentPosition = 165; 
int servoRunning = 0;
#define WITHIN_BOUNDS(x) max(min((x), (servoOpenPosition)), (servoClosePosition))

Servo servo;
Servo rightMotor;
Servo leftMotor;


unsigned long lastServoMoveTime = 0;
const int servoSpeed = 10; // milliseconds between servo moves


// Create an array of AccelStepper objects
AccelStepper motors[numMotors] = {
  {1, motorPins[0][0], motorPins[0][1]},
  {1, motorPins[1][0], motorPins[1][1]},
  {1, motorPins[2][0], motorPins[2][1]},
  {1, motorPins[3][0], motorPins[3][1]},
  {1, motorPins[4][0], motorPins[4][1]},
  {1, motorPins[5][0], motorPins[5][1]}
};

String command = ""; // Initialize an empty string to store the command

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud rate
  // Initialize each motor
  for (int i = 0; i < numMotors; i++) {
    motors[i].setMaxSpeed(1000);
    motors[i].setAcceleration(500);
  }
  servo.attach(A0); // Attach the servo on servoPin to the servo object

  rightMotor.attach(A1);
  leftMotor.attach(A2);
}

void loop() {
  // if (Serial.available() > 0) {
  //   String command = Serial.readStringUntil('\n');
  //   executeMotorCommand(command);
  // }
  if (Serial.available() > 0) {
    char incomingByte = Serial.read(); // Read the first byte

    // Continue to read characters until a newline is found
    while (incomingByte != '\n' && incomingByte != -1) {
        command += incomingByte; // Add the byte to your command string
        if (!Serial.available()) {
            break; // Exit loop if no more data is available
        }
        incomingByte = Serial.read(); // Read the next byte
    }

    // Check if the last read character is newline
    if (incomingByte == '\n') {
        executeMotorCommand(command); // Execute the command
        command = "";
    }
  }

  for (int i = 0; i < numMotors; i++) {
    motors[i].run();
  }
}

void executeMotorCommand(String command) {
  command.trim();
  // for (int i = 0; i < numMotors; i++) {
  //   String motorCommand = "M" + String(i+1);
  //   if (command.startsWith(motorCommand + "CW")) {
  //     motors[i].move(-20000);  // Move motor i+1 clockwise
  //     Serial.println(command);
  //   } else if (command.startsWith(motorCommand + "CCW")) {
  //     motors[i].move(20000); // Move motor i+1 counterclockwise
  //     Serial.println(command);
  //   } else if (command.startsWith(motorCommand + "STOP")) {
  //     motors[i].stop(); // Stop motor i+1
  //     // motors[i].disableOutputs(); // Optionally disable outputs to save power
  //     Serial.println(command);
  //   }
  // }

  for (int i = 0; i < numMotors; i++) {
    String motorCommand = "M" + String(i+1);
    if (command.startsWith(motorCommand)) { // M1,1,1000
      motors[i].move(command.substring(5).toInt());  // Move motor i+1 clockwise
      Serial.println(command.substring(5).toInt());
    } else if (command.startsWith(motorCommand + "STOP")) {
      motors[i].stop(); // Stop motor i+1
      // motors[i].disableOutputs(); // Optionally disable outputs to save power
      Serial.println(command);
    }
  }

  if (command.startsWith("G")) {
    servo.write(command.substring(1).toInt());
    // Serial.println(command.substring(1));
  }

  if (command == "MRF") { // Move forward
    motorRun(rightMotor, command.substring(3).toInt());
  } else if (command == "MRB") { // backward
    motorRun(rightMotor, -1 * command.substring(3).toInt());
  } else if (command == "MRS") { // Stop right motor
    motorRun(rightMotor, 0);
  } 

  if (command == "MLF") { // Move forward
    motorRun(leftMotor, command.substring(3).toInt());
  } else if (command == "MLB") { // backward
    motorRun(leftMotor, -1 * command.substring(3).toInt());
  } else if (command == "MLS") { // Stop left motor
    motorRun(leftMotor, 0);
  }

}

void motorRun(Servo motor, int speed) {
  int pwm = map(speed, -100, 100, 0, 180);
  motor.write(pwm);
  // Serial.println(speed);
}