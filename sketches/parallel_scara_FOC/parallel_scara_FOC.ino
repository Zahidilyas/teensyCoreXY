#include <BasicLinearAlgebra.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <math.h>
#include <Servo.h>
#include <SimpleFOC.h>
#include <Arduino.h>
#include "TeensyThreads.h"


#pragma region


int serialthreadID;
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
void doMotorLeft(char* cmd) { command.motor(&motorLeft, cmd); }
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
int numberOfSegments  = 10;

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

  motorLeft.P_angle.P = 20;
  motorRight.P_angle.P = 20;
  // default voltage_power_supply
  motorLeft.voltage_limit = 6;
  motorRight.voltage_limit = 6;

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
  command.add('M',doMotorLeft,"motor left");
  command.add('N',domotorRight,"motor right");
  
  // ----- establish serial link
  Serial.begin(115200);

  // motor.useMonitoring(Serial);
  
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


  threads.setDefaultStackSize(5120);
  threads.addThread(calibrate_scara);
  threads.addThread(serialread);

  // targetLeftMotor = 2;
  // targetRightMotor = 2;
  // calibrate_scara();
  
  
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
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the targets position using serial terminal:"));
  menu();
}

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
        threads.delay(10); // delay to give motor majority of cpu cycles
        threads.yield();
    }
}

/**
 * Calibrate parallel scara
 */
void calibrate_scara(){

  // Set control to velocity Control
  // motorLeft.controller = MotionControlType::velocity;
  // motorRight.controller = MotionControlType::velocity;

  // Rotate till right switch triggered
  while (digitalRead(SW2) == LOW) {
    targetLeftMotor = 1;
    targetRightMotor = 1;
    threads.delay(50);
  }
  targetLeftMotor = 0;
  targetRightMotor = 0;

  // NOTE: Positive angles are in CW direction for both motors 
  // reset right motor encoder zero position
  threads.delay(1000); // delays are important to let "motorRight.shaft_angle" settle
  motorRight.sensor_offset = motorRight.sensor_offset + motorRight.shaft_angle - calibrationAngleRight*PI/180;
  threads.delay(1000);

  // Rotate till left switch triggered
  while (digitalRead(SW1) == LOW) {
    targetLeftMotor = -1;
    targetRightMotor = -1;
    threads.delay(50);
  }
  targetLeftMotor = 0;
  targetRightMotor = 0;

  // reset left motor encoder zero position
  threads.delay(1000);
  motorLeft.sensor_offset = motorLeft.sensor_offset + motorLeft.shaft_angle + calibrationAngleLeft*PI/180;
  threads.delay(1000);

  motorLeft.controller = MotionControlType::angle;
  motorRight.controller = MotionControlType::angle;

  targetRightMotor = motorRight.shaft_angle;
  targetLeftMotor = motorLeft.shaft_angle;

  // ----- remember last "scaled" co-ordinate
  LAST_X = xMinWorkspace - 20;
  LAST_Y = yMinWorkspace;

  Serial.println(motorLeft.shaft_angle*180/PI);
  Serial.println(motorRight.shaft_angle*180/PI);
  // targetLeftMotor = 0;
  // targetRightMotor = 0;
  MovetoFrom(xMinWorkspace - 20, yMinWorkspace, 0, yMinWorkspace, numberOfSegments);
  // ----- remember last "scaled" co-ordinate
 LAST_X = 0;
 LAST_Y = yMinWorkspace;

  for(int z = 0; z<2; z++){
    // move_to(-100,150);
    move_to(100,150);
    move_to(100,60);
    move_to(-100,60);
    move_to(-100,60);
    move_to(-100,150);
  }

  threads.kill(threads.id());
}

//--------------------------------------------------------------------------
// MAIN LOOP
//--------------------------------------------------------------------------

void loop()
{
    // threads.delay(5000);
    motorLeft.loopFOC();
    motorRight.loopFOC(); //RIGHT MOTOR

    motorLeft.move(targetLeftMotor);
    motorRight.move(targetRightMotor);

    // user communication
    command.run();
}

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
  int numberOfSegmentsdefault = numberOfSegments;
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
    numberOfSegments = numberOfSegmentsdefault; // only so i can change it to 2 in manual mode
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
      Serial.println("HERE");
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
//  Serial.println(threads.id());
 MovetoFrom( LAST_X, LAST_Y, THIS_X, THIS_Y, numberOfSegments);

 Serial.println(F("LAST_X ==== "));
 Serial.println(LAST_X);
 Serial.println(F("LAST_Y ==== "));
 Serial.println(LAST_Y);
 Serial.println(F("THIS_X ==== "));
 Serial.println(THIS_X);
 Serial.println(F("THIS_Y ==== "));
 Serial.println(THIS_Y);

 // ----- remember last "scaled" co-ordinate
 LAST_X = THIS_X;
 LAST_Y = THIS_Y;
}

void MovetoFrom( double x_current, double y_current, double x_goal, double y_goal, const int numberOfSegments) {

  // Serial.println(threads.id());
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
  // Serial.println(x_current);
  // Serial.print(" y_current = ");
  // Serial.println(y_current);
  // Serial.print(" x_goal = ");
  // Serial.println(x_goal);
  // Serial.print(" y_goal = ");
  // Serial.println(y_goal);

  for (int i = 0; i < numberOfSegments; i += 1) {

    // Serial.print(" x_current = ");
    // Serial.print(x[i]);
    // Serial.print(",  y_current = ");
    // Serial.println(y[i]);
    //Calculate q1 and q2 using the inverse kinematics functions (in degree)
    angleOfLeftMotorDeg =  InverseKinematics_q1 (l2f, l1f, x[i], y[i]);
    angleOfRightMotorDeg = InverseKinematics_q2 (l2r, l1r, x[i], y[i], distanceBetweenMotorShafts);

    // Serial.print(" x_current = ");
    // Serial.print(0);
    // Serial.print(",  y_current = ");
    // Serial.println(60);
    // angleOfLeftMotorDeg =  InverseKinematics_q1 (l2f, l1f,0,60);
    // angleOfRightMotorDeg = InverseKinematics_q2 (l2r, l1r,0,60, distanceBetweenMotorShafts);

  //  angleOfLeftMotorSteps = degreesToSteps(angleOfLeftMotorDeg);
  //  angleOfRightMotorSteps = degreesToSteps(angleOfRightMotorDeg);

    //Setting target to move the motor
    Serial.print("left motor angle go to = ");
    Serial.println(angleOfLeftMotorDeg);
    Serial.print("right motor angle go to = ");
    Serial.println(angleOfRightMotorDeg);
    targetLeftMotor = -1*angleOfLeftMotorDeg*PI/180;
    targetRightMotor = -1*angleOfRightMotorDeg*PI/180;
    threads.delay(50);
    // delay(1000);

    // Fix arm rotation
    BLA::Matrix<3, 3> gripperPose;
    ForwardKinematics(angleOfLeftMotorDeg, angleOfRightMotorDeg, &gripperPose);
    double angleOfGripper = atan2( gripperPose(1, 0), gripperPose(0, 0) ) * 180 / PI;
    // Serial.println("endpose = ");
    // Serial.println(gripperPose(0,2));
    // Serial.println(gripperPose(1,2));
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
 double y_change = (y2 - y1) / double(NumberOfSegments);

 if (x2 - x1 == 0) {
   for ( int i = 0; i < NumberOfSegments ; i++) {
     x[i] = x1;
     y[i] = y_change * double(i) + y1;
   }
 } else {
   double gradient = (y2 - y1) / (x2 - x1);
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