#include <SimpleFOC.h>

StepperMotor motor = StepperMotor(50);
StepperMotor motor2 = StepperMotor(50);
StepperDriver4PWM driver = StepperDriver4PWM(5, 6, 7, 8, 9, 10);
StepperDriver4PWM driver2 = StepperDriver4PWM(2, 3, 4, 11, 12, 32);
InlineCurrentSense current_sense = InlineCurrentSense(0.0015, 154.16667, A0, A2);

// encoder instance
Encoder encoder = Encoder(28, 29, 1000);
Encoder encoder2 = Encoder(30, 31, 1000);

// Interrupt routine intialisation
// channel A and B callbacks
void doA() {
  encoder.handleA();
}
void doB() {
  encoder.handleB();
}
void doA2() {
  encoder2.handleA();
}
void doB2() {
  encoder2.handleB();
}


// velocity set point variable
float target_velocity = 0;
float target_velocity_2 = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) {
  command.scalar(&target_velocity, cmd);
}
void doTarget2(char* cmd2) {
  command.scalar(&target_velocity_2, cmd2);
}
void doMotor(char* cmd) {
  command.motor(&motor, cmd);
}
void doMotor2(char* cmd) {
  command.motor(&motor2, cmd);
}


void setup() {

  // initialize encoder sensor hardware
  encoder.init();
  encoder2.init();
  encoder.enableInterrupts(doA, doB);
  encoder2.enableInterrupts(doA2, doB2);
  // link the motor to the sensor
  motor.linkSensor(&encoder);
  motor2.linkSensor(&encoder2);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver2.voltage_power_supply = 12;
  driver.init();
  driver2.init();

  // link driver to current sense after initializing the driver
  current_sense.linkDriver(&driver);

  // default values of per phase gains
  //  current_sense.gain_a = 1.0 / shunt_resistor / gain;
  //  current_sense.gain_b = 1.0 / shunt_resistor / gain;

  // link the motor and the driver
  motor.linkDriver(&driver);
  motor2.linkDriver(&driver2);

  // aligning voltage [V]
  motor.voltage_sensor_align = 5;
  motor2.voltage_sensor_align = 5;
  // index search velocity [rad/s]
  //  motor.velocity_index_search = 3;

  // set motion control loop to be used
  //  motor.controller = MotionControlType::velocity;

  motor.controller = MotionControlType::angle;
  motor2.controller = MotionControlType::angle;

  //  motor.torque_controller = TorqueControlType::voltage;
  //  motor.controller = MotionControlType::torque;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 2.0f; //0.5f
  motor.PID_velocity.I = 20; //20
  motor.PID_velocity.D = 0.01;

  motor2.PID_velocity.P = 2.0f; //0.5f
  motor2.PID_velocity.I = 20; //20
  motor2.PID_velocity.D = 0.01;

  motor.P_angle.P = 10;
  motor2.P_angle.P = 10;
  // default voltage_power_supply
  motor.voltage_limit = 12;
  motor2.voltage_limit = 12;

  // jerk control using voltage voltage ramp
  // default value is 300 volts per sec  ~ 0.3V per millisecond
  motor.PID_velocity.output_ramp = 1000;
  motor2.PID_velocity.output_ramp = 1000;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01f;
  motor2.LPF_velocity.Tf = 0.01f;

  // use monitoring with serial
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  motor2.init();
  // align sensor and start FOC
  motor.initFOC();
  motor2.initFOC();

  // initialize current sense after initializing the motor
  //  current_sense.init();
  // init current sense
  if (current_sense.init())  Serial.println("Current sense init success!");
  else {
    Serial.println("Current sense init failed!");
    return;
  }
  // link the current sense to the motor
  motor.linkCurrentSense(&current_sense);

  // set the initial target value
  motor.target = 0;
  motor2.target = 0;

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('Y', doTarget2, "target velocity 2");
  command.add('M', doMotor, "motor");
  command.add('N', doMotor2, "motor");


  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));

  _delay(1000);
}


void loop() {
  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();
  motor2.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_velocity);
  motor2.move(target_velocity);

  // function intended to be used with serial plotter to monitor motor variables
  // significantly slowing the execution down!!!!
  motor.monitor();

  //  Serial.print(encoder.getAngle());
  //  Serial.print("\t");
  //  Serial.println(encoder.getVelocity());

  //  Serial.print(encoder2.getAngle());
  //  Serial.print("\t");
  //  Serial.println(encoder2.getVelocity());

  // user communication
  command.run();
}
