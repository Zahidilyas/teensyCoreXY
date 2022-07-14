#include <BasicLinearAlgebra.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#include <Servo.h>
#include <SimpleFOC.h>
#include <Arduino.h>
#include "TeensyThreads.h"


#pragma region


StepperMotor motorLeft = StepperMotor(50);
StepperMotor motorRight = StepperMotor(50);
StepperDriver4PWM driver = StepperDriver4PWM(5,6,7,8,9,10);
StepperDriver4PWM driver2 = StepperDriver4PWM(2,3,4,11,12,32);

// encoder instance
Encoder encoderLeft = Encoder(28, 29, 1000);
Encoder encoderRight = Encoder(30, 31, 1000);

// Interrupt routine intialisation
// channel A and B callbacks
void doA(){encoderLeft.handleA();}
void doB(){encoderLeft.handleB();}
void doA2(){encoderRight.handleA();}
void doB2(){encoderRight.handleB();}

// velocity set point variable
float targetLeftMotor = 0;
float targetRightMotor = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&targetLeftMotor, cmd); }
void doTarget2(char* cmd2) { command.scalar(&targetRightMotor, cmd2); }
void doMotor(char* cmd) { command.motor(&motorLeft, cmd); }
void domotorRight(char* cmd) { command.motor(&motorRight, cmd); }

// System parameters
double
distanceBetweenMotorShafts = 43.0, // 30 for nema-11 and 43 for nema-17
l1f = 100.0,
l2f = 121.5, // 115 for nema-11 and 121.5 for nema-17
l1r = 100.0,
l2r = 121.5,

calibrationAngleRight = 53.1, //45.0,
calibrationAngleLeft = 211.2; //225.0

int maxSpeed = 200; //400, 500

double
xMinWorkspace = -125.2 + distanceBetweenMotorShafts / 2, //-120 + d/2
xMaxWorkspace = 125.2 + distanceBetweenMotorShafts / 2,
yMinWorkspace = 60,  // 50
yMaxWorkspace = 165.8; //160
int numberOfSegments  = 100;

//Servo Constants
int zTopAt = 30,
zDownAt = 160,
currentZPosition,
gripCloseAt = 180,
gripOpenAt = 100,
currentGripper;

// Define Variables
double
X0,
Y0,
xa,
ya,
angleOfLeftMotorDeg,
angleOfRightMotorDeg;

int
X1 = 40,
Y1 = 170,
angleOfLeftMotorSteps,
angleOfRightMotorSteps;
int delayBetweenSteps = 1000000 / double(maxSpeed); // Delay between steps to control speed

int
/*
   XY plotters only deal in integer steps.
*/
THIS_X = 0,                         //"scaled" x co-ordinate (rounded)
THIS_Y = 0,                         //"scaled" y co-ordinate (rounded)
LAST_X = 0,                         //"scaled" x co-ordinate (rounded)
LAST_Y = 0;                         //"scaled" y co-ordinate (rounded)

// PIN Constants
#define SW1 33
#define SW2 34
#define ROT_SERVO 14
#define Z_SERVO 36
#define GRIP_SERVO 37


#pragma endregion

// Create servo and stepper objects
Servo rotationServoObject;  // create servo object to control a servo
Servo gripperServoObject;
Servo zAxisServoObject;

// ----- gcode definitions
#define STRING_SIZE 128             //string size
char BUFFER[STRING_SIZE + 1], INPUT_CHAR;
String INPUT_STRING, SUB_STRING;
int INDEX = 0,                        //buffer index
START,                            //used for sub_string extraction
FINISH;
float X, Y, I, J;//gcode float values held here

// ----- plotter definitions
#define BAUD 9600
#define XOFF 0x13                   //pause transmission (19 decimal)
#define XON 0x11                    //resume transmission (17 decimal)

using namespace BLA;


double draw_line(double x1, double y1, double x2, double y2, int NumberOfSegments, double *x, double *y);
void MovetoFrom( double x_current, double y_current, double x_goal, double y_goal, int numberOfSegments);
void move_to(float x, float y);

// -----------------------
// SETUP
// -----------------------
void setup()
{
  
  ////////////////////////////////////////////////////////////////////////////////////////////
  // initialize encoder sensor hardware
  encoderLeft.init();
  encoderRight.init();
  encoderLeft.enableInterrupts(doA, doB);
  encoderRight.enableInterrupts(doA2, doB2);
  // link the motor to the sensor
  motorLeft.linkSensor(&encoderLeft);
  motorRight.linkSensor(&encoderRight);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver2.voltage_power_supply = 12;
  driver.init();
  driver2.init();
  // link the motor and the driver
  motorLeft.linkDriver(&driver);
  motorRight.linkDriver(&driver2);

  // aligning voltage [V]
  motorLeft.voltage_sensor_align = 5;
  motorRight.voltage_sensor_align = 5;

  motorLeft.controller = MotionControlType::angle;
  motorRight.controller = MotionControlType::angle;
  
//  motorLeft.torque_controller = TorqueControlType::voltage;
//  motorLeft.controller = MotionControlType::torque;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motorLeft.PID_velocity.P = 2.0f; //0.5f
  motorLeft.PID_velocity.I = 20; //20
  motorLeft.PID_velocity.D = 0.01;

  motorRight.PID_velocity.P = 2.0f; //0.5f
  motorRight.PID_velocity.I = 20; //20
  motorRight.PID_velocity.D = 0.01;

  motorLeft.P_angle.P = 10;
  motorRight.P_angle.P = 10;
  // default voltage_power_supply
  motorLeft.voltage_limit = 12;
  motorRight.voltage_limit = 12;

  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motorLeft.PID_velocity.output_ramp = 1000;
  motorRight.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motorLeft.LPF_velocity.Tf = 0.01f;
  motorRight.LPF_velocity.Tf = 0.01f;

  // initialize motor
  motorLeft.init();
  motorRight.init();
  // align sensor and start FOC
  motorLeft.initFOC();
  motorRight.initFOC();

  // set the initial target value
  motorLeft.target = 0;
  motorRight.target = 0;

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('Y', doTarget2, "target velocity 2");
  command.add('M',doMotor,"motor");
  command.add('N',domotorRight,"motor");
  
  // ----- establish serial link
  Serial.begin(BAUD);
  
//  Serial.println(F("Motor ready."));
//  Serial.println(F("Set the target velocity using serial terminal:"));
 ///////////////////////////////////////////////////////////////////////////////////////
  

  // ----- Configure calibration switches and servos
  pinMode(SW1, INPUT); //Calibration Switch 1
  pinMode(SW2, INPUT); //Calibration Switch 2
  rotationServoObject.attach(ROT_SERVO); // Rotational servo
  gripperServoObject.attach(GRIP_SERVO);
  zAxisServoObject.attach(Z_SERVO);
    //set initial servo values
  zAxisServoObject.write(zTopAt);
  currentZPosition = zTopAt;
  gripperServoObject.write(gripOpenAt);
  currentGripper = gripOpenAt;

  _delay(1000);

  motorLeft.controller = MotionControlType::velocity;
  motorRight.controller = MotionControlType::velocity;
 // initialize motor
  motorLeft.init();
  motorRight.init();
  // align sensor and start FOC
  motorLeft.initFOC();
  motorRight.initFOC();


  threads.addThread(calibrate_scara);
//   targetLeftMotor = 2;
//   targetRightMotor = 2;
   threads.addThread(serialread);


//   calibrate_scara();
  
  
//  //Starting location
//  MovetoFrom(xMinWorkspace - 20, yMinWorkspace, xMinWorkspace - 20, yMinWorkspace, numberOfSegments);
//  THIS_X = xMinWorkspace - 20;
//  THIS_Y = yMinWorkspace;
//  LAST_X = xMinWorkspace - 20;
//  LAST_Y = yMinWorkspace;
//
//  // ----- plotter setup
//  memset(BUFFER, '\0', sizeof(BUFFER));     //fill with string terminators
//  INPUT_STRING.reserve(STRING_SIZE);
//  INPUT_STRING = "";
//  // ----- flush the buffers
//  Serial.flush();                           //clear TX buffer
//  while (Serial.available()) Serial.read(); //clear RX buffer
  // ----- display commands
  menu();
}


/**
 * Motor control loop thread
 */
//void motorControlLoop(){
//  while(1){
//    motorLeft.loopFOC();
//    motorRight.loopFOC(); //RIGHT MOTOR
//
//    motorLeft.move(targetLeftMotor);
//    motorRight.move(targetRightMotor);
//  //  command.run();
//  }
//}

/**
 * Serial Reading thread 
 * 
 */
void serialread(){
    // ----- get the next instruction
    while(1){
        while (Serial.available()) {
        INPUT_CHAR = (char)Serial.read();         //read character
        Serial.write(INPUT_CHAR);                 //echo character to the screen
        BUFFER[INDEX++] = INPUT_CHAR;             //add char to buffer
        if (INPUT_CHAR == '\n') {                 //check for line feed
            Serial.flush();                         //clear TX buffer
            Serial.write(XOFF);                     //pause transmission
            INPUT_STRING = BUFFER;                  //convert to string
            process(INPUT_STRING);                  //interpret string and perform task
            memset(BUFFER, '\0', sizeof(BUFFER));   //fill buffer with string terminators
            INDEX = 0;                              //point to buffer start
            INPUT_STRING = "";                      //empty string
            Serial.flush();                         //clear TX buffer
            Serial.write(XON);                      //resume transmission
        }
        }
        threads.delay(10);
        threads.yield();
    }
}

/**
 * Calibrate parallel scara
 */
void calibrate_scara(){

  // Set control to velocity Control
  motorLeft.controller = MotionControlType::velocity;
  motorRight.controller = MotionControlType::velocity;

  // Rotate till right switch triggered
  while (digitalRead(SW2) == LOW) {
    targetLeftMotor = 2;
    targetRightMotor = 2;
    threads.delay(50);
  }
  targetLeftMotor = 0;
  targetRightMotor = 0;
  // reset right motor encoder zero position
  motorRight.sensor_offset = motorRight.sensor_offset + motorRight.shaft_angle;

  // Rotate till left switch triggered
  while (digitalRead(SW1) == LOW) {
    targetLeftMotor = -2;
    targetRightMotor = -2;
    threads.delay(50);
  }
  targetLeftMotor = 0;
  targetRightMotor = 0;
  // reset left motor encoder zero position
  motorLeft.sensor_offset = motorLeft.sensor_offset + motorLeft.shaft_angle;

  motorLeft.controller = MotionControlType::angle;
  motorRight.controller = MotionControlType::angle;

  motorRight.target = motorRight.shaft_angle;
  targetRightMotor = motorRight.shaft_angle;
  motorLeft.target = motorLeft.shaft_angle;

//  MovetoFrom(xMinWorkspace - 20, yMinWorkspace, xMinWorkspace - 20, yMinWorkspace, numberOfSegments);
  threads.suspend(threads.id());
}

//--------------------------------------------------------------------------
// MAIN LOOP
//--------------------------------------------------------------------------

void loop()
{
    motorLeft.loopFOC();
    motorRight.loopFOC(); //RIGHT MOTOR

    motorLeft.move(targetLeftMotor);
    motorRight.move(targetRightMotor);
//  //   zAxisServoObject.write(zTopAt);
//  //   delay(1000);
//  //   gripperServoObject.write(gripCloseAt); // close
//  //   delay(1000);
//  //   MovetoFrom(xMinWorkspace, yMinWorkspace, xMinWorkspace, yMaxWorkspace, numberOfSegments);
//  //   delay(100);
//  //   MovetoFrom(xMinWorkspace, yMaxWorkspace, xMaxWorkspace, yMaxWorkspace, numberOfSegments);
//  //   delay(100);
//  //   zAxisServoObject.write(zDownAt);
//  //   delay(1000);
//  //   gripperServoObject.write(gripOpenAt); // open
////    delay(1000);
//  //   MovetoFrom(xMaxWorkspace, yMaxWorkspace, xMaxWorkspace, yMinWorkspace, numberOfSegments);
//  //   delay(100);
//  //   MovetoFrom(xMaxWorkspace, yMinWorkspace, xMinWorkspace, yMinWorkspace, numberOfSegments);
//  //   delay(100);
//    // pen_up();
//    // move_to(100, 100);
//    // pen_down();
//    // move_to(50,150);
//
////  move_parameter();
//
//  // ----- get the next instruction
    // while (Serial.available()) {
    //   INPUT_CHAR = (char)Serial.read();         //read character
    //   Serial.write(INPUT_CHAR);                 //echo character to the screen
    //   BUFFER[INDEX++] = INPUT_CHAR;             //add char to buffer
    //   if (INPUT_CHAR == '\n') {                 //check for line feed
    //     Serial.flush();                         //clear TX buffer
    //     Serial.write(XOFF);                     //pause transmission
    //     INPUT_STRING = BUFFER;                  //convert to string
    //     process(INPUT_STRING);                  //interpret string and perform task
    //     memset(BUFFER, '\0', sizeof(BUFFER));   //fill buffer with string terminators
    //     INDEX = 0;                              //point to buffer start
    //     INPUT_STRING = "";                      //empty string
    //     Serial.flush();                         //clear TX buffer
    //     Serial.write(XON);                      //resume transmission
    //   }
    // }
////    Serial.println(BUFFER);
}
//
//void move_parameter() {
//  move_to(xMinWorkspace, yMinWorkspace);
//  delay(100);
//  move_to(xMinWorkspace, yMaxWorkspace);
//  delay(100);
//  move_to(xMaxWorkspace, yMaxWorkspace);
//  delay(100);
//  move_to(xMaxWorkspace, yMinWorkspace);
//  delay(100);
//}
//
//---------------------------------------------------------------------------
// MENU
//---------------------------------------------------------------------------
/*
   The Arduino F() flash macro is used to conserve RAM.
*/
void menu() {
  Serial.println(F(""));
  Serial.println(F("  ------------------------------------------------------"));
  Serial.println(F("                         MENU"));
  Serial.println(F("  ------------------------------------------------------"));
  Serial.println(F("    MENU ............... menu"));
  Serial.println(F("    G00 X## Y## ........ goto XY (pen-up)"));
  Serial.println(F("    G01 X## Y## ........ goto XY (pen-down)"));
  Serial.println(F("    T1 ................. manual control"));
  Serial.println(F("    T2 S##.## .......... set drawing Scale (1=100%)"));
  Serial.println(F("    T3 ................. pen up"));
  Serial.println(F("    T4 ................. pen down"));
  Serial.println(F("    E3 ................. gripper open"));
  Serial.println(F("    E4 ................. gripper close"));
  Serial.println(F("    T5 ................. test pattern: ABC"));
  Serial.println(F("    T6 ................. test pattern: target"));
  Serial.println(F("    T7 ................. test pattern: radials"));
  Serial.println(F("  ------------------------------------------------------"));
}

// --------------------------------------------------------------------------------
// PREOCESS
// --------------------------------------------------------------------------------
void process(String string) {
  numberOfSegments = 100; // only so i can change it to 2 in manual mode
  // ----- convert string to upper case
  INPUT_STRING = string;
  INPUT_STRING.toUpperCase();

  // ----------------------------------
  // T1   control the arm
  // ----------------------------------
  if(INPUT_STRING.startsWith("T1")) {
    // ----- instructions
    Serial.println(F(""));
    Serial.println(F("  ----------------------------------------------"));
    Serial.println(F("    Position the pen over the 0,0 co-ordinate:"));
    Serial.println(F("  ----------------------------------------------"));
    Serial.println(F("    X-axis:             Y-axis:"));
    Serial.println(F("   'A'  'D'            'W'  'S'"));
    Serial.println(F("   <-    ->            <-    ->"));
    Serial.println(F("    Exit = 'E'"));

    numberOfSegments = 1; // only so i can change it to 2 in manual mode
    // ----- flush the buffer
    while (Serial.available() > 0) Serial.read();

    // ----- control motors
    char keystroke = ' ';
    while (keystroke != 'E') // press 'E' to exit
    {
      // ----- check for keypress
      if (Serial.available() > 0) {
        keystroke = (char) Serial.read();
      }

      // ----- select task
      switch (keystroke) {
        case 'a':
        case 'A': {
            // ----- move left
            move_to(LAST_X-5,LAST_Y);
            keystroke = ' ';    //otherwise motor will continue to rotate
            break;
          }
        case 'd':
        case 'D': {
            // ------ move right
            move_to(LAST_X+5,LAST_Y);
            keystroke = ' ';
            break;
          }
        case 'w':
        case 'W': {
            // ----- move up
            move_to(LAST_X,LAST_Y+5);
            keystroke = ' ';
            break;
          }
        case 's':
        case 'S': {
            // ----- move down
            move_to(LAST_X,LAST_Y-5);
            keystroke = ' ';
            break;
          }
        case 'e':
        case 'E': {
            // ----- exit
            Serial.println(F(" "));
            Serial.println(F("  Exit ..."));
            keystroke = 'E';
            break;
          }
        // ----- default for keystroke
        default: {
            break;
          }
      }
    }
    
  }

  // ----------------------------------
  // G00   linear move with pen_up
  // ----------------------------------
  if (INPUT_STRING.startsWith("G00")) {

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = START + 8;
      //      FINISH = INPUT_STRING.indexOf('Y');
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      //      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH - 1);
      X = SUB_STRING.toFloat();
      Serial.println(F("X ==== "));
      Serial.println(X);
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      Y = SUB_STRING.toFloat();
      Serial.println(F("Y ==== "));
      Serial.println(Y);
    }

//    pen_up();
    move_to(X, Y);
  }

  // ----------------------------------
  // G01   linear move with pen_down
  // ----------------------------------
  if (INPUT_STRING.startsWith("G01")) {

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      X = SUB_STRING.toFloat();
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      Y = SUB_STRING.toFloat();
    }

//    pen_down();
    move_to(X, Y);
  }

  // ----------------------------------
  // T3   pen up
  // ----------------------------------
  if (INPUT_STRING.startsWith("T3")) {
//    pen_up();
  }

  // ----------------------------------
  // T4   pen down
  // ----------------------------------
  if (INPUT_STRING.startsWith("T4")) {
//    pen_down();
  }

  // ----------------------------------
  // E3   gripperOpen
  // ----------------------------------
  if (INPUT_STRING.startsWith("E3")) {
//    gripperOpen();
  }

  // ----------------------------------
  // E4   gripperClose
  // ----------------------------------
  if (INPUT_STRING.startsWith("E4")) {
//    gripperClose();
  }

  // ----------------------------------
  // G02   clockwise arc with pen_down
  // ----------------------------------
  if (INPUT_STRING.startsWith("G02")) {

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('X'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      X = SUB_STRING.toFloat();
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('Y'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      Y = SUB_STRING.toFloat();
    }

    // ----- extract I
    START = INPUT_STRING.indexOf('I');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('I'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      I = SUB_STRING.toFloat();
    }

    // ----- extract J
    START = INPUT_STRING.indexOf('J');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('J'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      J = SUB_STRING.toFloat();
    }

//    pen_down();
//    draw_arc_cw(X, Y, I, J);
  }

  // ------------------------------------------
  // G03   counter-clockwise arc with pen_down
  // ------------------------------------------
  if (INPUT_STRING.startsWith("G03")) {

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('X'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      X = SUB_STRING.toFloat();
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('Y'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      Y = SUB_STRING.toFloat();
    }

    // ----- extract I
    START = INPUT_STRING.indexOf('I');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('I'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      I = SUB_STRING.toFloat();
    }

    // ----- extract J
    START = INPUT_STRING.indexOf('J');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('J'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      J = SUB_STRING.toFloat();
    }

//    pen_down();
//    draw_arc_ccw(X, Y, I, J);
  }

  // ----------------------------------
  // T5   ABC test pattern
  // ----------------------------------
  if (INPUT_STRING.startsWith("T5")) {
//    abc();
  }
}

////---------------------------------------------------------------------------
//// PEN_UP, PEN_DOWN
////---------------------------------------------------------------------------
//void pen_up() {
//  for (int i = currentZPosition; i >= zTopAt; i--){
//    zAxisServoObject.write(i);
//    delay(5);
//  }
//  currentZPosition = zTopAt;
//  delay(20);                 //give pen-lift time to respond
//}
//void pen_down() {
//  for (int i = currentZPosition; i <= zDownAt; i++){
//    zAxisServoObject.write(i);
//    delay(5);
//  }
//  currentZPosition = zDownAt;
//  delay(20);                 //give pen-lift time to respond
//}
////---------------------------------------------------------------------------
//// Gripper OPEN, CLOSE
////---------------------------------------------------------------------------
//void gripperOpen() {
//  for (int i = currentGripper; i>= gripOpenAt; i-- ){
//    gripperServoObject.write(i);
//    delay(5);
//  }  
//  currentGripper = gripOpenAt;
//  delay(20);                 //give pen-lift time to respond
//}
//void gripperClose() {
//  for (int i = currentGripper; i<= gripCloseAt; i++ ){
//    gripperServoObject.write(i);
//    delay(5);
//  }  
//  currentGripper = gripCloseAt;
//  delay(20);                 //give pen-lift time to respond
//}

// -------------------------------
// MOVE_TO
// -------------------------------
void move_to(float x, float y) {                        //x,y are absolute co-ordinates
 // ----- apply scale factor
 THIS_X = x;      //scale x and y
 THIS_Y = y;

 // ----- draw a line between these "scaled" co-ordinates
 // draw_line(LAST_X, LAST_Y, THIS_X, THIS_Y, );
 MovetoFrom( LAST_X, LAST_Y, THIS_X, THIS_Y, numberOfSegments);

 // ----- remember last "scaled" co-ordinate
 LAST_X = THIS_X;
 LAST_Y = THIS_Y;
}

void MovetoFrom( double x_current, double y_current, double x_goal, double y_goal, int numberOfSegments) {

  double x[numberOfSegments];
  double y[numberOfSegments];
  long positions[2]; // Array of desired stepper positions

  //find intermidiate positions between current postion and goal position
  draw_line(x_current, y_current, x_goal, y_goal, numberOfSegments, x, y);

  if(numberOfSegments == 1){
    x[0] = x_goal;
    y[0] = y_goal;
  }

  // Serial.print(" x_current = ");
  //  Serial.println(x_current);
  //  Serial.print(" y_current = ");
  //  Serial.println(y_current);

  for (int i = 0; i < numberOfSegments; i += 1) {
    //Calculate q1 and q2 using the inverse kinematics functions
    angleOfLeftMotorDeg = -1*InverseKinematics_q1 (l2f, l1f, x[i], y[i]);
    angleOfRightMotorDeg = InverseKinematics_q2 (l2r, l1r, x[i], y[i], distanceBetweenMotorShafts);

//    angleOfLeftMotorSteps = degreesToSteps(angleOfLeftMotorDeg);
//    angleOfRightMotorSteps = degreesToSteps(angleOfRightMotorDeg);

    //Setting target to move the motor
    Serial.println(angleOfLeftMotorDeg);
    targetLeftMotor = angleOfLeftMotorDeg;
    targetRightMotor = angleOfRightMotorDeg;
    delay(1000);

    // Fix arm rotation
    BLA::Matrix<3, 3> gripperPose;
    ForwardKinematics(angleOfLeftMotorDeg, angleOfRightMotorDeg, &gripperPose);
    double angleOfGripper = atan2( gripperPose(1, 0), gripperPose(0, 0) ) * 180 / PI;
    // rotationServoObject.write(int(angleOfGripper) + 90);
 }
}
//
//
//void constantAccel(int steps) {
//  // Control direction
//  digitalWrite(LEFT_STEPPER_DIR_PIN, LOW); //Changes the rotations direction
//  digitalWrite(RIGHT_STEPPER_DIR_PIN, HIGH);
//  int delays[steps];
//  float c0 = 15000; // controls acceleration
//  float lastDelay = 0;
//  int highSpeed = 400;
//
//  for (int i = 0; i < steps; i++) {
//    float _delay = c0;
//    if (i > 0) {
//      _delay = lastDelay - (2 * lastDelay) / (4 * i + 1);
//    }
//    if (_delay < highSpeed) {
//      _delay = highSpeed;
//    }
//    delays[i] = _delay;
//    lastDelay = _delay;
//  }
//
//  // use delays from the array, forward
//  for (int x = 0; x < steps; x++) {
//    digitalWrite(LEFT_STEPPER_STEP_PIN, HIGH);
//    digitalWrite(RIGHT_STEPPER_STEP_PIN, HIGH);
//    delayMicroseconds(20);
//    digitalWrite(LEFT_STEPPER_STEP_PIN, LOW);
//    digitalWrite(RIGHT_STEPPER_STEP_PIN, LOW);
//    delayMicroseconds( delays[x]);
//    //    Serial.println(delays[x]);
//  }
//
//  //  // use delays from the array, backward
//  //  for(int x = 0; x < steps; x++) {
//  //    digitalWrite(stepPin,HIGH);
//  //    digitalWrite(stepPin2,HIGH);
//  //    delayMicroseconds(20);
//  //    digitalWrite(stepPin,LOW);
//  //    digitalWrite(stepPin2,LOW);
//  //    delayMicroseconds( delays[steps-x+1]);
//  ////    Serial.println(delays[steps-x+1]);
//  //  }
//}
//
//// Function to get the angle from the steps
//double stepsToDegrees (int steps)
//{
//  double deg;
//  deg = double(steps) * 360 / double(stepsPerRevolution);
//  return deg;
//}
//
//// Function to get the steps from the angle
//int degreesToSteps (double deg)
//{
//  int steps;
//  steps = int( deg * (double(stepsPerRevolution) / 360) );
//  return steps;
//}
//

//Function to get the angle q1 using the inverse kinematics equation
double InverseKinematics_q1 (double l2f, double l1f, double x, double y)
{
 double q1;
 q1 = atan2(y, x) + acos((-1 * pow(l2f, 2) + pow(l1f, 2) + pow(x, 2) + pow(y, 2)) / (2 * l1f * sqrt(pow(x, 2) + pow(y, 2))));
 return q1 * (360 / (2 * PI)); // return q1 in degrees
 //return q1;  // return q1 in radians
}

//Function to get the angle q2 using the inverse kinematics equation
double InverseKinematics_q2 (double l2r, double l1r, double x, double y, double d)
{
 double dif_dx = (d - x);
 double q2;

 q2 = PI - atan2(y, dif_dx) - acos((-1 * pow(l2r, 2) + pow(l1r, 2) + pow(dif_dx, 2) + pow(y, 2)) / (2 * l1r * sqrt(pow(dif_dx, 2) + pow(y, 2))));

 return q2 * (360 / (2 * PI)); // return q2 in degrees
 //return q2; // return q2 in radians
}

//Function to get the x coordinate of the end effector
double ForwardKinematics_x (double l2f, double l1f, double l2r, double l1r, double d, double q1_deg, double q2_deg)
{
 double q1;
 double q2;
 double a;
 double b;
 double c;
 double x0;
 double theta1;
 double theta2;

 q1 = q1_deg * PI / 180;
 q2 = q2_deg * PI / 180;
 a = 2 * l2r * l1r * sin(q2) - 2 * l1f * l2r * sin(q1);
 b = 2 * l2r * d - 2 * l1f * l2r * cos(q1) + 2 * l2r * l1r * cos(q2);
 c = pow(l1f, 2) - pow(l2f, 2) + pow(l2r, 2) + pow(l1r, 2) + pow(d, 2) - l1f * l1r * sin(q1) * sin(q2) - 2 * l1f * d * cos(q1) + 2 * l1r * d * cos(q2) - 2 * l1f * l1r * cos(q1) * cos(q2);
 theta2 = 2 * atan2((a + sqrt(pow(a, 2) + pow(b, 2) - pow(c, 2))), (b - c));
 theta1 = asin((l2r * sin(theta2) + l1r * sin(q2) - l1f * sin(q1)) / l2f);
 x0 = l1f * cos(q1) + l2f * cos(theta1);

 Serial.println(q1);
 Serial.println(q2);
 Serial.println(a);
 Serial.println(b);
 Serial.println(c);
 Serial.println(theta1);
 Serial.println(theta2);
 return x0;
}

//Function to get the y coordinate of the end effector
double ForwardKinematics_y (double l2f, double l1f, double l2r, double l1r, double d, double q1_deg, double q2_deg)
{
 double q1;
 double q2;
 double a;
 double b;
 double c;
 double y0;
 double theta1;
 double theta2;

 q1 = q1_deg * PI / 180;
 q2 = q2_deg * PI / 180;
 a = 2 * l2r * l1r * sin(q2) - 2 * l1f * l2r * sin(q1);
 b = 2 * l2r * d - 2 * l1f * l2r * cos(q1) + 2 * l2r * l1r * cos(q2);
 c = pow(l1f, 2) - pow(l2f, 2) + pow(l2r, 2) + pow(l1r, 2) + pow(d, 2) - l1f * l1r * sin(q1) * sin(q2) - 2 * l1f * d * cos(q1) + 2 * l1r * d * cos(q2) - 2 * l1f * l1r * cos(q1) * cos(q2);
 theta2 = 2 * atan2((a + sqrt(pow(a, 2) + pow(b, 2) - pow(c, 2))), (b - c));
 theta1 = asin((l2r * sin(theta2) + l1r * sin(q2) - l1f * sin(q1)) / l2f);
 y0 = l1f * sin(q1) + l2f * sin(theta1);
 return y0;
}
//
////Function to rotate the left motor CCW
//void TurnCounterClockwise_q1 (int delayBetweenSteps)
//{
//  digitalWrite(LEFT_STEPPER_DIR_PIN, 1);
//  digitalWrite(LEFT_STEPPER_STEP_PIN, HIGH);
//  //delayMicroseconds(delayBetweenSteps);
//  digitalWrite(LEFT_STEPPER_STEP_PIN, LOW);
//  delayMicroseconds(delayBetweenSteps);
//}
//
////Function to rotate the right motor CCW
//void TurnCounterClockwise_q2 (int delayBetweenSteps)
//{
//  digitalWrite(RIGHT_STEPPER_DIR_PIN, 1);
//  digitalWrite(RIGHT_STEPPER_STEP_PIN, HIGH);
//  //delayMicroseconds(delayBetweenSteps);
//  digitalWrite(RIGHT_STEPPER_STEP_PIN, LOW);
//  delayMicroseconds(delayBetweenSteps);
//}
////
//////Function to rotate the left motor CW
//void TurnClockwise_q1 (int delayBetweenSteps)
//{
//  motorLeft.controller = MotionControlType::angle;
//  motorRight.controller = MotionControlType::angle;
//  
//  digitalWrite(LEFT_STEPPER_DIR_PIN, 0);
//  digitalWrite(LEFT_STEPPER_STEP_PIN, HIGH);
//  //delayMicroseconds(delayBetweenSteps);
//  digitalWrite(LEFT_STEPPER_STEP_PIN, LOW);
//  delayMicroseconds(delayBetweenSteps);
//}
////
//////Function to rotate the right motor CW
//void TurnClockwise_q2 (int delayBetweenSteps)
//{
//  digitalWrite(RIGHT_STEPPER_DIR_PIN, 0);
//  digitalWrite(RIGHT_STEPPER_STEP_PIN, HIGH);
//  //delayMicroseconds(delayBetweenSteps);
//  digitalWrite(RIGHT_STEPPER_STEP_PIN, LOW);
//  delayMicroseconds(delayBetweenSteps);
//}
//
//

/**
 *   Find intermidiate positions between current postion and goal position
 *   
 *   @param x1 - current x position
 *   @param y1 - current y position
 *   @param x2 - target x position
 *   @param y2 - target y position
 *   @param NumberOfSegments - the number of intermidiate points needed
 *   @param x, y - pointer to the array, to store the results
 */
double draw_line(double x1, double y1, double x2, double y2, int NumberOfSegments, double *x, double *y) {

 double delx = (x2 - x1) / (double(NumberOfSegments) - 1);
 double gradient = (y2 - y1) / (x2 - x1);
 double y_change = (y2 - y1) / double(NumberOfSegments);

 if (x2 - x1 == 0) {
   for ( int i = 0; i < NumberOfSegments ; i++) {
     x[i] = x1;
     y[i] = y_change * double(i) + y1;
   }
 } else {
   for ( int i = 0; i < NumberOfSegments ; i++) {
     x[i] = x1 + double(i) * delx;
     y[i] = (x[i] - x1) * gradient + y1;
   }
 }
}
//
//void Controller_const_accel(int LeftMotorStepsToMove, int RightMotorStepsToMove, int MaxSpeed, int Acceleration) {
//  double Steps_t1_min = double(MaxSpeed) * double(MaxSpeed) / double(Acceleration);
//  double LargeSteps = double(LeftMotorStepsToMove);
//  double SmallSteps = double(RightMotorStepsToMove);
//  double t1;
//  int MoveMotorNumber = 2;
//  double set_Speed;
//
//  if (RightMotorStepsToMove > LeftMotorStepsToMove) {
//    //change setspeed for motor 2
//    LargeSteps = double(RightMotorStepsToMove);
//    SmallSteps = double(LeftMotorStepsToMove);
//    MoveMotorNumber = 1;
//  }
//
//  if (LargeSteps >= Steps_t1_min) {
//    t1 = (LargeSteps / double(MaxSpeed)) + (double(MaxSpeed) / double(Acceleration));
//  } else {
//    t1 = sqrt( (4 * LargeSteps) / double(Acceleration) );
//  }
//
//  set_Speed = 0.5 * ( double(Acceleration) * t1 - sqrt( double(Acceleration) * double(Acceleration) * t1 * t1 - 4 * SmallSteps * double(Acceleration) ) );
//  //  Serial.print(" set_Speed = ");
//  //  Serial.println(set_Speed);
//  if (MoveMotorNumber == 1) {
//    LeftStepperObject.setMaxSpeed(int(set_Speed));
//    while (RightStepperObject.distanceToGo() != 0) {
//      LeftStepperObject.run();
//      RightStepperObject.run();
//    }
//    LeftStepperObject.setMaxSpeed(MaxSpeed);
//  } else {
//    RightStepperObject.setMaxSpeed(int(set_Speed));
//    while (LeftStepperObject.distanceToGo() != 0) {
//      LeftStepperObject.run();
//      RightStepperObject.run();
//    }
//    RightStepperObject.setMaxSpeed(MaxSpeed);
//
//  }
//}
//
//void FixEndEffectorRotationToHorizontal(){
//  // Finding end effector pose
//  BLA::Matrix<3, 3> gripperPose;
//  ForwardKinematics(angleOfLeftMotorDeg, angleOfRightMotorDeg, &gripperPose);
//  double angleOfGripper = atan2( gripperPose(1, 0), gripperPose(0, 0) ) * 180 / PI;
//  X0 = gripperPose(0, 2);
//  Y0 = gripperPose(1, 2);
//  rotationServoObject.write(int(angleOfGripper) + 90);
//}

/**
 * Perform forward kinematics
 */
void ForwardKinematics(double q1_deg, double q2_deg, BLA::Matrix<3, 3> *arm) {

 double q1;
 double q2;
 double angle_vBD;
 double vBD_norm;
 double gamma;
 double theta1;
 double angleEndEffectorFromBase;
 double angleEndEffectorAttachment = 44 * PI / 180;
 double endEffectorDisplacementX = 15;
 double endEffectorDisplacementY = 33;

 q1 = q1_deg * PI / 180;
 q2 = q2_deg * PI / 180;

 BLA::Matrix<3, 3> T10 = {cos(q1 - PI / 2 ), -sin(q1 - PI / 2 ), -distanceBetweenMotorShafts / 2,
                          sin(q1 - PI / 2 ),  cos(q1 - PI / 2 ), 0,
                          0              ,  0              , 1
                         };

 BLA::Matrix<3, 3> TA0 = {cos(q2 - PI / 2 ), -sin(q2 - PI / 2 ), distanceBetweenMotorShafts / 2,
                          sin(q2 - PI / 2 ),  cos(q2 - PI / 2 ), 0,
                          0              ,  0              , 1
                         };

 BLA::Matrix<3> vB_select = {0, l1f, 1};
 BLA::Matrix<3> vD_select = {0, l1r, 1};

 BLA::Matrix<3> vB = T10 * vB_select;
 BLA::Matrix<3> vD = TA0 * vD_select;

 BLA::Matrix<3> vBD = -vB + vD;
 angle_vBD = atan2(vBD(1), vBD(0));

 vBD_norm = sqrt( pow(vBD(1), 2) + pow(vBD(0), 2) );
 gamma = acos( vBD_norm / (2 * l2f) );

 theta1 = gamma + angle_vBD;

 BLA::Matrix<3, 3> T21 = {cos(theta1 - q1), -sin(theta1 - q1), 0,
                          sin(theta1 - q1),  cos(theta1 - q1), l1f,
                          0             ,   0            , 1
                         };

 BLA::Matrix<3, 3> T32 = {cos(angleEndEffectorAttachment), -sin(angleEndEffectorAttachment), 0,
                          sin(angleEndEffectorAttachment),  cos(angleEndEffectorAttachment), l2f,
                          0                              ,  0                              , 1
                         };

 BLA::Matrix<3, 3> T43 = {1, 0, endEffectorDisplacementX,
                          0, 1, endEffectorDisplacementY,
                          0, 0, 1
                         };

 *arm = T10 * T21 * T32 * T43;

}
//
////----------------------------------------------------------------------------
//// DRAW ARC CLOCKWISE (G02)
////----------------------------------------------------------------------------
//void draw_arc_cw(float x, float y, float i, float j) {
//
//  // ----- inkscape sometimes produces some crazy values for i,j
//  if ((i < -100) || (i > 100) || (j < -100) || (j > 100)) {
//    move_to(x, y);
//  } else {
//
//    // ----- variables
//    float
//    thisX = LAST_X, //current unscaled X co-ordinate
//    thisY = LAST_Y, //current unscaled Y co-ordinate
//    nextX = x,                    //next X co-ordinate
//    nextY = y,                    //next Y co-ordinate
//    newX,                         //interpolated X co-ordinate
//    newY,                         //interpolated Y co-ordinate
//    I = i,                        //horizontal distance thisX from circle center
//    J = j,                        //vertical distance thisY from circle center
//    circleX = thisX + I,          //circle X co-ordinate
//    circleY = thisY + J,          //circle Y co-ordinate
//    delta_x,                      //horizontal distance between thisX and nextX
//    delta_y,                      //vertical distance between thisY and nextY
//    chord,                        //line_length between lastXY and nextXY
//    radius,                       //circle radius
//    alpha,                        //interior angle of arc
//    beta,                         //fraction of alpha
//    arc,                          //subtended by alpha
//    current_angle,                //measured CCW from 3 o'clock
//    next_angle,                   //measured CCW from 3 o'clock
//    ARC_MAX = 2.0;
//
//    // ----- calculate arc
//    delta_x = thisX - nextX;
//    delta_y = thisY - nextY;
//    chord = sqrt(delta_x * delta_x + delta_y * delta_y);
//    radius = sqrt(I * I + J * J);
//    alpha = 2 * asin(chord / (2 * radius)); //see construction lines
//    arc = alpha * radius;         //radians
//
//    // ----- sub-divide alpha
//    int segments = 1;
//    if (arc > ARC_MAX) {
//      segments = (int)(arc / ARC_MAX);
//      beta = alpha / segments;
//    } else {
//      beta = alpha;
//    }
//
//    // ----- calculate current angle
//    /*
//      atan2() angles between 0 and PI are CCW +ve from 3 o'clock.
//      atan2() angles between 2*PI and PI are CW -ve relative to 3 o'clock
//    */
//    current_angle = atan2(-J, -I);
//    if (current_angle <= 0) current_angle += 2 * PI;        //angles now 360..0 degrees CW
//
//    // ----- plot intermediate CW co-ordinates
//    next_angle = current_angle;                             //initialise angle
//    for (int segment = 1; segment < segments; segment++) {
//      next_angle -= beta;                                   //move CW around circle
//      if (next_angle < 0) next_angle += 2 * PI;             //check if angle crosses zero
//      newX = circleX + radius * cos(next_angle);            //standard circle formula
//      newY = circleY + radius * sin(next_angle);
//      move_to(newX, newY);
//    }
//
//    // ----- draw final line
//    move_to(nextX, nextY);
//  }
//}
//
////----------------------------------------------------------------------------
//// DRAW ARC COUNTER-CLOCKWISE (G03)
////----------------------------------------------------------------------------
///*
//   We know the start and finish co-ordinates which allows us to calculate the
//   chord length. We can also calculate the radius using the biarc I,J values.
//   If we bisect the chord the center angle becomes 2*asin(chord/(2*radius)).
//   The arc length may now be calculated using the formula arc_length = radius*angle.
//*/
//void draw_arc_ccw(float x, float y, float i, float j) {
//
//  // ----- inkscape sometimes produces some crazy values for i,j
//  if ((i < -100) || (i > 100) || (j < -100) || (j > 100)) {
//    move_to(x, y);
//  } else {
//
//    // ----- variables
//    float
//    thisX = LAST_X,  //current unscaled X co-ordinate
//    thisY = LAST_Y,  //current unscaled Y co-ordinate
//    nextX = x,                      //next X co-ordinate
//    nextY = y,                      //next Y co-ordinate
//    newX,                           //interpolated X co-ordinate
//    newY,                           //interpolated Y co-ordinate
//    I = i,                          //horizontal distance thisX from circle center
//    J = j,                          //vertical distance thisY from circle center
//    circleX = thisX + I,            //circle X co-ordinate
//    circleY = thisY + J,            //circle Y co-ordinate
//    delta_x,                        //horizontal distance between thisX and nextX
//    delta_y,                        //vertical distance between thisY and nextY
//    chord,                          //line_length between lastXY and nextXY
//    radius,                         //circle radius
//    alpha,                          //interior angle of arc
//    beta,                           //fraction of alpha
//    arc,                            //subtended by alpha
//    current_angle,                  //measured CCW from 3 o'clock
//    next_angle,                     //measured CCW from 3 o'clock
//    ARC_MAX = 2.0;
//
//    // ----- calculate arc
//    delta_x = thisX - nextX;
//    delta_y = thisY - nextY;
//    chord = sqrt(delta_x * delta_x + delta_y * delta_y);
//    radius = sqrt(I * I + J * J);
//    alpha = 2 * asin(chord / (2 * radius));     //see construction lines
//    arc = alpha * radius;                       //radians
//
//    // ----- sub-divide alpha
//    int segments = 1;
//    if (arc > ARC_MAX) {
//      segments = (int)(arc / ARC_MAX);
//      beta = alpha / segments;
//    } else {
//      beta = alpha;
//    }
//
//    // ----- calculate current angle
//    /*
//        tan2() angles between 0 and PI are CCW +ve from 3 o'clock.
//        atan2() angles between 2*PI and PI are CW -ve relative to 3 o'clock
//    */
//    current_angle = atan2(-J, -I);
//    if (current_angle <= 0) current_angle += 2 * PI;        //angles now 360..0 degrees CW
//
//    // ----- plot intermediate CCW co-ordinates
//    next_angle = current_angle;                             //initialise angle
//    for (int segment = 1; segment < segments; segment++) {
//      next_angle += beta;                                   //move CCW around circle
//      if (next_angle > 2 * PI) next_angle -= 2 * PI;        //check if angle crosses zero
//      newX = circleX + radius * cos(next_angle);            //standard circle formula
//      newY = circleY + radius * sin(next_angle);
//      move_to(newX, newY);
//    }
//
//    // ----- draw final line
//    move_to(nextX, nextY);
//  }
//}
//
//// ----------------------------------------
//// ABC
//// ----------------------------------------
//void abc() {
//  process(F("G00 X50.600359 Y23.420344"));
//  //  process(F("G02 X50.752716 Y22.976260 I-3.135884 J-1.324038"));
//  //  process(F("G02 X50.785093 Y22.730023 I-0.920147 J-0.246237"));
//  //  process(F("G02 X50.395324 Y21.695296 I-1.568337 J0.000000"));
//  //  process(F("G02 X48.616901 Y20.260423 I-5.033669 J4.419324"));
//  //  process(F("G02 X46.381993 Y19.348409 I-4.838496 J8.662483"));
//  //  process(F("G02 X44.183295 Y19.054795 I-2.198698 J8.085548"));
//  //  process(F("G02 X41.865268 Y19.467670 I0.000000 J6.713555"));
//  //  process(F("G02 X40.245550 Y20.503495 I1.545608 J4.201152"));
//  //  process(F("G02 X39.219290 Y22.122336 I3.157768 J3.136575"));
//  //  process(F("G02 X38.806572 Y24.470408 I6.473066 J2.348072"));
//  //  process(F("G02 X39.490101 Y28.182255 I10.420197 J0.000000"));
//  //  process(F("G02 X41.412290 Y31.305554 I9.193131 J-3.504638"));
//  //  process(F("G02 X44.336973 Y33.441702 I6.709781 J-6.116396"));
//  //  process(F("G02 X47.644620 Y34.164064 I3.307648 J-7.211572"));
//  //  process(F("G02 X49.085783 Y34.013721 I-0.000000 J-6.982526"));
//  //  process(F("G02 X50.133662 Y33.639032 I-0.850084 J-4.030028"));
//  //  process(F("G02 X50.927697 Y32.982080 I-1.126976 J-2.170474"));
//  //  process(F("G02 X51.144836 Y32.355618 I-0.795126 J-0.626462"));
//  //  process(F("G02 X50.979946 Y31.746676 I-1.206859 J0.000000"));
//  //  process(F("G02 X50.269784 Y30.858305 I-3.386486 J1.979114"));
//  //  process(F("G03 X48.739474 Y32.638692 I-4.305181 J-2.152593"));
//  //  process(F("G03 X46.934854 Y33.211228 I-1.804620 J-2.557788"));
//  //  process(F("G03 X44.865511 Y32.640761 I-0.000000 J-4.038459"));
//  //  process(F("G03 X42.812375 Y30.751354 I3.496454 J-5.859673"));
//  //  process(F("G03 X41.521944 Y28.150097 I7.294760 J-5.239488"));
//  //  process(F("G03 X41.052544 Y25.024608 I10.170799 J-3.125489"));
//  //  process(F("G03 X41.358190 Y23.047268 I6.548917 J0.000000"));
//  //  process(F("G03 X42.102608 Y21.709126 I3.656766 J1.158154"));
//  //  process(F("G03 X43.314946 Y20.829195 I2.521185 J2.198476"));
//  //  process(F("G03 X44.961119 Y20.493773 I1.646173 J3.871797"));
//  //  process(F("G03 X47.727663 Y21.168894 I0.000000 J6.006005"));
//  //  process(F("G03 X50.600359 Y23.420344 I-4.544548 J8.756936"));
//  //  process(F("G01 X50.600359 Y23.420344"));
//  //  process(F("G00 X23.454230 Y28.699836"));
//  //  process(F("G02 X23.258509 Y29.247403 I2.678175 J1.266042"));
//  //  process(F("G02 X23.201437 Y29.711010 I1.854425 J0.463606"));
//  //  process(F("G02 X23.715287 Y31.069809 I2.053497 J0.000000"));
//  //  process(F("G02 X25.904382 Y32.773699 I5.614239 J-4.954789"));
//  //  process(F("G02 X28.704691 Y33.800933 I5.587639 J-10.901752"));
//  //  process(F("G02 X31.854753 Y34.164064 I3.150061 J-13.481375"));
//  //  process(F("G02 X33.604787 Y33.959798 I0.000000 J-7.598769"));
//  //  process(F("G02 X34.771598 Y33.473743 I-0.900681 J-3.805687"));
//  //  process(F("G02 X35.615458 Y32.630428 I-1.363090 J-2.207829"));
//  //  process(F("G02 X35.889723 Y31.665300 I-1.560988 J-0.965128"));
//  //  process(F("G02 X35.303747 Y29.921282 I-2.888311 J0.000000"));
//  //  process(F("G02 X32.943707 Y27.776167 I-6.918162 J5.240505"));
//  //  process(F("G02 X34.567939 Y27.139583 I-2.104158 J-7.759112"));
//  //  process(F("G02 X35.636931 Y26.366360 I-2.132406 J-4.073640"));
//  //  process(F("G02 X36.379807 Y25.321353 I-2.380949 J-2.479122"));
//  //  process(F("G02 X36.618935 Y24.227336 I-2.383015 J-1.094017"));
//  //  process(F("G02 X36.026166 Y22.197076 I-3.773251 J0.000000"));
//  //  process(F("G02 X34.061832 Y20.221533 I-5.518247 J3.522571"));
//  //  process(F("G02 X31.369264 Y19.027768 I-5.128625 J7.934286"));
//  //  process(F("G02 X27.868393 Y18.568653 I-3.500871 J13.117974"));
//  //  process(F("G02 X25.793101 Y19.167138 I0.000000 J3.897363"));
//  //  process(F("G02 X25.194616 Y20.250700 I0.681659 J1.083563"));
//  //  process(F("G02 X25.225337 Y20.646314 I2.562654 J-0.000000"));
//  //  process(F("G02 X25.486301 Y22.117485 I35.898712 J-5.609165"));
//  //  process(F("G01 X26.857219 Y29.010969"));
//  //  process(F("G03 X26.998207 Y29.839815 I-14.262137 J2.852427"));
//  //  process(F("G03 X27.022507 Y30.187429 I-2.474194 J0.347614"));
//  //  process(F("G03 X26.964582 Y30.506866 I-0.909758 J0.000000"));
//  //  process(F("G03 X26.789159 Y30.809689 I-0.989871 J-0.371202"));
//  //  process(F("G02 X28.084797 Y31.229045 I4.918140 J-12.983875"));
//  //  process(F("G02 X28.587881 Y31.295831 I0.503083 J-1.861405"));
//  //  process(F("G02 X29.002366 Y31.107501 I0.000000 J-0.550275"));
//  //  process(F("G02 X29.190696 Y30.605510 I-0.574861 J-0.501991"));
//  //  process(F("G02 X29.174585 Y30.432163 I-0.940630 J0.000000"));
//  //  process(F("G02 X28.869843 Y28.894292 I-74.974595 J14.057713"));
//  //  process(F("G01 X27.596154 Y22.506398"));
//  //  process(F("G03 X27.407065 Y21.395624 I25.119200 J-4.847563"));
//  //  process(F("G03 X27.382252 Y21.057695 I2.288700 J-0.337928"));
//  //  process(F("G03 X27.843772 Y20.294140 I0.862387 J0.000000"));
//  //  process(F("G03 X29.715731 Y19.832619 I1.871959 J3.565639"));
//  //  process(F("G03 X31.526735 Y20.128159 I0.000000 J5.696494"));
//  //  process(F("G03 X32.992323 Y20.931299 I-1.502222 J4.480076"));
//  //  process(F("G03 X34.026640 Y22.200410 I-2.503366 J3.096265"));
//  //  process(F("G03 X34.363240 Y23.585630 I-2.682024 J1.385220"));
//  //  process(F("G03 X34.148489 Y24.695436 I-2.975045 J-0.000000"));
//  //  process(F("G03 X33.604861 Y25.471860 I-1.940658 J-0.780263"));
//  //  process(F("G03 X32.768796 Y25.999000 I-1.964159 J-2.188645"));
//  //  process(F("G03 X31.076926 Y26.521924 I-3.621282 J-8.717909"));
//  //  process(F("G02 X30.432077 Y26.335937 I-1.252000 J3.130013"));
//  //  process(F("G02 X29.696283 Y26.269132 I-0.735794 J4.018591"));
//  //  process(F("G02 X29.445459 Y26.286414 I0.000000 J1.828837"));
//  //  process(F("G02 X29.064300 Y26.356635 I0.596050 J4.304919"));
//  //  process(F("G02 X29.438640 Y27.338746 I5.106656 J-1.384047"));
//  //  process(F("G02 X29.764344 Y27.756722 I1.202868 J-0.601434"));
//  //  process(F("G02 X30.234517 Y28.000252 I0.776340 J-0.923211"));
//  //  process(F("G02 X31.135261 Y28.116468 I0.900744 J-3.432550"));
//  //  process(F("G02 X31.378139 Y28.111033 I0.000000 J-5.429361"));
//  //  process(F("G02 X31.786692 Y28.087298 I-0.689212 J-15.391372"));
//  //  process(F("G03 X33.457741 Y29.806863 I-4.247308 J5.799206"));
//  //  process(F("G03 X33.857653 Y31.091652 I-1.863851 J1.284789"));
//  //  process(F("G03 X33.240526 Y32.234356 I-1.366508 J-0.000000"));
//  //  process(F("G03 X31.174154 Y32.851482 I-2.066372 J-3.150932"));
//  //  process(F("G03 X27.293198 Y31.874110 I-0.000000 J-8.193950"));
//  //  process(F("G03 X23.454230 Y28.699836 I6.012387 J-11.179941"));
//  //  process(F("G01 X23.454230 Y28.699836"));
//  //  process(F("G00 X12.370209 Y25.345461"));
//  //  process(F("G01 X14.334220 Y25.296848"));
//  //  process(F("G03 X16.344033 Y25.341207 I0.000000 J45.552596"));
//  //  process(F("G03 X17.416355 Y25.432967 I-0.576039 J13.043259"));
//  //  process(F("G02 X17.057498 Y28.851447 I66.264688 J8.684258"));
//  //  process(F("G02 X16.969105 Y31.091652 I28.343468 J2.240205"));
//  //  process(F("G01 X16.978828 Y31.820863"));
//  //  process(F("G02 X16.654582 Y31.415231 I-24.362686 J19.142120"));
//  //  process(F("G02 X15.539850 Y30.051310 I-276.590024 J224.919481"));
//  //  process(F("G03 X13.629409 Y27.498969 I29.639011 J-24.176118"));
//  //  process(F("G03 X12.574388 Y25.724652 I13.226843 J-9.065587"));
//  //  process(F("G01 X12.370209 Y25.345461"));
//  //  process(F("G00 X11.670166 Y24.198168"));
//  //  process(F("G01 X11.475709 Y23.828703"));
//  //  process(F("G03 X10.453024 Y21.493002 I14.155155 J-7.589567"));
//  //  process(F("G03 X10.250633 Y20.289593 I3.476515 J-1.203409"));
//  //  process(F("G03 X10.316083 Y19.943208 I0.949331 J-0.000000"));
//  //  process(F("G03 X10.532595 Y19.570105 I1.323570 J0.518696"));
//  //  process(F("G03 X10.300257 Y19.489445 I3.137841 J-9.413445"));
//  //  process(F("G03 X9.482530 Y19.190914 I42.222649 J-116.924015"));
//  //  process(F("G02 X9.097875 Y19.086190 I-0.855158 J2.382242"));
//  //  process(F("G02 X8.763041 Y19.054795 I-0.334835 J1.769854"));
//  //  process(F("G02 X8.277401 Y19.249753 I0.000000 J0.702344"));
//  //  process(F("G02 X8.082443 Y19.706223 I0.436907 J0.456470"));
//  //  process(F("G02 X8.333756 Y20.987389 I3.391281 J0.000000"));
//  //  process(F("G02 X9.764492 Y23.828703 I18.642339 J-7.606424"));
//  //  process(F("G01 X10.065899 Y24.324564"));
//  //  process(F("G01 X9.570035 Y24.324564"));
//  //  process(F("G03 X8.016212 Y24.170112 I-0.000000 J-7.893143"));
//  //  process(F("G03 X7.100438 Y23.828703 I0.608728 J-3.031721"));
//  //  process(F("G02 X7.932520 Y24.978194 I1.964955 J-0.546467"));
//  //  process(F("G02 X9.560313 Y25.471860 I1.627793 J-2.436874"));
//  //  process(F("G01 X10.765943 Y25.442689"));
//  //  process(F("G01 X11.028459 Y25.831603"));
//  //  process(F("G02 X12.621837 Y28.077029 I38.056380 J-25.317154"));
//  //  process(F("G02 X15.695415 Y32.005598 I119.673321 J-90.461712"));
//  //  process(F("G02 X16.806236 Y33.260000 I14.322595 J-11.564166"));
//  //  process(F("G02 X17.280236 Y33.639032 I1.733405 J-1.681816"));
//  //  process(F("G02 X17.834414 Y33.882572 I1.177026 J-1.926044"));
//  //  process(F("G02 X19.312306 Y34.222402 I3.908941 J-13.616125"));
//  //  process(F("G03 X19.095228 Y32.467101 I23.799291 J-3.834329"));
//  //  process(F("G03 X19.030344 Y30.965256 I17.348913 J-1.501845"));
//  //  process(F("G03 X19.256050 Y26.994264 I35.045048 J-0.000000"));
//  //  process(F("G03 X19.954013 Y22.866141 I38.580488 J4.399934"));
//  //  process(F("G03 X20.720782 Y20.795305 I8.056900 J1.805858"));
//  //  process(F("G03 X21.499942 Y19.959018 I1.953184 J1.038650"));
//  //  process(F("G02 X20.521336 Y19.212088 I-3.388359 J3.424790"));
//  //  process(F("G02 X19.934567 Y19.054795 I-0.586769 J1.015804"));
//  //  process(F("G02 X18.944735 Y19.694699 I-0.000000 J1.085511"));
//  //  process(F("G02 X17.562198 Y24.324564 I16.634711 J7.488692"));
//  //  process(F("G02 X15.955536 Y24.193605 I-3.198664 J29.321420"));
//  //  process(F("G02 X14.324497 Y24.149555 I-1.631039 J30.174575"));
//  //  process(F("G01 X11.670166 Y24.198168"));
//  //  process(F("G00 X0.0000 Y0.0000"));
//}
