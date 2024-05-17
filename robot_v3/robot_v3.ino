#include <AccelStepper.h>
#include <Servo.h>

#define DEBUG 0    // SET TO 0 OUT TO REMOVE TRACES

#if DEBUG
#define D_SerialBegin(...) Serial.begin(__VA_ARGS__)
#define D_print(...)    Serial.print(__VA_ARGS__)
#define D_write(...)    Serial.write(__VA_ARGS__)
#define D_println(...)  Serial.println(__VA_ARGS__)
#else
#define D_SerialBegin(...)
#define D_print(...)
#define D_write(...)
#define D_println(...)
#endif

const int numMotors = 6;

// Define motor pins
const int motorPins[numMotors][2] = {
  {2, 3},   // Motor 1: STEP pin 2, DIR pin 3
  {4, 5},   // Motor 2: STEP pin 4, DIR pin 5
  {6, 7},   // Motor 3: STEP pin 6, DIR pin 7
  {8, 9},   // Motor 4: STEP pin 8, DIR pin 9
  {10, 11}, // Motor 5: STEP pin 10, DIR pin 11
  {12, 13}  // Motor 6: STEP pin 12, DIR pin 13
};

const int servoClosePosition = 10; // min   0
const int servoOpenPosition = 175; // max 180

// [Pi, -Pi] in joint space. (400 microsteps/rev * 38.4 ratio = 15360 steps/rev = (7680 - (-7680)))
#define JOINT_WITHIN_BOUNDS(x) constrain(x, -7680, 7680) 
#define SERVO_WITHIN_BOUNDS(x) constrain(x, servoClosePosition, servoOpenPosition)
#define MOTOR_WITHIN_BOUNDS(x) constrain(x, 0, 180)

Servo servo;
Servo rightMotor;
Servo leftMotor;




// Create an array of AccelStepper objects
AccelStepper motors[numMotors] = {
  {1, motorPins[0][0], motorPins[0][1]},
  {1, motorPins[1][0], motorPins[1][1]},
  {1, motorPins[2][0], motorPins[2][1]},
  {1, motorPins[3][0], motorPins[3][1]},
  {1, motorPins[4][0], motorPins[4][1]},
  {1, motorPins[5][0], motorPins[5][1]}
};

String command = "";

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud rate

  // Initialize each motor
  for (int i = 0; i < numMotors; i++) {
    motors[i].setMaxSpeed(1000);
    motors[i].setAcceleration(500);
  }
  servo.attach(A0); 

  rightMotor.attach(A1);
  leftMotor.attach(A2);
}

void loop() {
  readSerialandExecuteCommand();

  for (int i = 0; i < numMotors; i++) {
    motors[i].run();
  }
}

void readSerialandExecuteCommand() {
  if (Serial.available() > 0) {
    char incomingByte = Serial.read(); 

    while (incomingByte != '\n' && incomingByte != -1) {
        command += incomingByte; 
        if (!Serial.available()) {
            break; 
        }
        incomingByte = Serial.read(); 
    }

    if (incomingByte == '\n') {
        executeMotorCommand(command); 
        D_println(command);
        command = "";
    }
  }
}

void executeMotorCommand(String command) {
  command.trim();

  for (int i = 0; i < numMotors; i++) {
    String motorMoveCommand = "M" + String(i+1) + ",1,";
    String motorMoveToCommand = "M" + String(i+1) + ",2,";
    String readMotorStateCommand  = "M" + String(i+1) + ",3";
    int arg = command.substring(5).toInt();

    if (command.startsWith(motorMoveCommand + "STOP")) { // M1,1,1000
      motors[i].stop(); 
      // motors[i].disableOutputs(); // Optionally disable outputs to save power
      D_println("M" + String(i+1) + " stopped.");
    } else if (command.startsWith(motorMoveCommand)) {
      motors[i].move(arg);
      D_println(arg);  
    } else if (command.startsWith(motorMoveToCommand)) {
      int steps = JOINT_WITHIN_BOUNDS(arg);
      motors[i].moveTo(steps);
      D_println(steps);
      D_println("M" + String(i+1) + ", deg, " + String(steps * 360.0 / 15360.0));
    } else if (command.startsWith(readMotorStateCommand)) {
      int steps = motors[i].currentPosition();
      D_println("M" + String(i+1) + ", steps, " + String(steps));
      D_println("M" + String(i+1) + ", deg, " + String(steps * 360.0 / 15360.0));
    }
  }

  if (command.startsWith("G")) {
    int angle = SERVO_WITHIN_BOUNDS(command.substring(1).toInt());
    servo.write(angle);
    D_println(angle);
  }

  if (command.startsWith("MR")) {
    motorRun(rightMotor, command.substring(2).toInt());
  }

  if (command.startsWith("ML")) {
    motorRun(leftMotor, command.substring(2).toInt());
  }
}

void motorRun(Servo motor, int speed) {
  int pwm = MOTOR_WITHIN_BOUNDS(map(speed, -100, 100, 0, 180));
  motor.write(pwm);
  // D_println(speed);
  D_println(pwm);
}