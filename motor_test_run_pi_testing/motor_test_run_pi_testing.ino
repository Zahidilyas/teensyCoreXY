#include <Stepper.h>

const byte buffSize = 32;
char inputSeveral[buffSize]; // space for 31 chars and a terminator

byte maxChars = 12; // a shorter limit to make it easier to see what happens
                           //   if too many chars are entered

// Limit switches
const int x_switch = 4;
const int y_switch = 7;

//Initialize the stepper library on pins 2,5 and 3,6

// Stepper Motor X
const int stepPin = 2; //X.STEP
const int dirPin = 5; // X.DIR
const int stepPin2 = 3; //Y.STEP
const int dirPin2 = 6; // Y.DIR
const int stepsPerRevolution = 200;
int rpm = 400; // max = 480
byte direction = 0; // 0 for cw and 1 for ACW

long steptime = 300000/rpm; // rpm = a/360 * 1/steptime * 60
int steptime_ms = 0;

int position_x = 100; // # steps
int position_y = 100; // # steps
bool buttonState_x = HIGH;
bool buttonState_y = HIGH;

void setup() {
  // Set switched pins as Inputs
  pinMode(x_switch, INPUT);
  pinMode(y_switch, INPUT);
  
  // Sets the two pins as Outputs
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(stepPin2,OUTPUT); 
  pinMode(dirPin2,OUTPUT);
  
  digitalWrite(dirPin,direction); // Enables the motor to move in a particular direction
  digitalWrite(dirPin2,direction); // Enables the motor to move in a particular direction
  digitalWrite(dirPin,0); // Enables the motor to move in a particular direction
  digitalWrite(dirPin2,1);
  
  Serial.begin(9600);
  
  cli();//stop interrupts
  
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  
  sei();//allow interrupts

  calibrate(); 
//  while(Serial.available() == 0) { }
//  moveto(3000,4000);

//  int no_steps = 200;
//  int x,y,m;
//  for(int i=0; i<no_steps+1;i++){
//    m = 3000/no_steps;
//    x=i*m;
//    
//    m = 4000/no_steps;
//    y=i*m;
//    moveto(x,y);
//  }

  for(int t=0; t<4000;t++){
    float theta_rad = 1*t*PI/180;
    float xf =1000*cos(theta_rad)+2500;
    int x = (int)xf;
    float yf = 1000*sin(theta_rad)+2500;
    int y = (int)yf;
//    Serial.println("x = ");
    Serial.println(x);
//    Serial.println("y = ");
//    Serial.println(y);
    
    moveto(x,y);
  }
  
//  Serial.println("out ");
//  for(int i=0; i<4*120;i++){
//    left();
//    left();
//    left();
//    left();
//    up();
//    up();
//    up();
//    up();
//  }
//  for(int i=0; i<4*200;i++){
//    diag_ul();
//    diag_ul();
//    diag_ul();
//    diag_ul();
//  }
//  for(int i=0; i<4*500;i++){
//    diag_ur();
//  }

  
//  moveto(3000,4000);
//  moveto(3000,4000);
//  delay(3000);
//  moveto(1000,3000);
//  delay(3000);
//  moveto(2000,1000);
//  delay(3000);
//  moveto(4000,2000);
//  delay(3000);
//  moveto(3000,4000);

//  moveto(30,40);
//  delay(3000);
//  moveto(10,30);
//  delay(3000);
//  moveto(20,10);
//  delay(3000);
//  moveto(40,20);
}

void calibrate(){

  digitalWrite(dirPin,1); // Enables the motor to move in a particular direction
  digitalWrite(dirPin2,1); // Enables the motor to move in a particular direction
    
  while(digitalRead(x_switch)==1){
      digitalWrite(stepPin,HIGH);
      digitalWrite(stepPin2,HIGH); 
      delayMicroseconds(20); 
      digitalWrite(stepPin,LOW); 
      digitalWrite(stepPin2,LOW);
      delayMicroseconds(20); 
    
      if ((steptime-40)<=10000){
        delayMicroseconds(steptime - 40);
      }else {
        steptime_ms = steptime/1000;
        delay(steptime_ms);
      } 
  }

  digitalWrite(dirPin,1); // Enables the motor to move in a particular direction
  digitalWrite(dirPin2,0); // Enables the motor to move in a particular direction
    
  while(digitalRead(y_switch)==1){
      digitalWrite(stepPin,HIGH);
      digitalWrite(stepPin2,HIGH); 
      delayMicroseconds(20); 
      digitalWrite(stepPin,LOW); 
      digitalWrite(stepPin2,LOW);
      delayMicroseconds(20); 
    
      if ((steptime-40)<=10000){
        delayMicroseconds(steptime - 40);
      }else {
        steptime_ms = steptime/1000;
        delay(steptime_ms);
      } 
  }

  position_x = 0;
  position_y = 0;
  
}

void moveto(int x,int y){

    int dx, dy, shortest, longest, remaining;
    dx = x - position_x; // positive left
    dy = y - position_y; // positive up
    shortest = min(abs(dy),abs(dx)); // shortest axis
    longest = max(abs(dy),abs(dx)); // longest axis
    remaining = longest-shortest;

//    Serial.println("dx , dy = ");
//    Serial.print(dx);
//    Serial.print(" , ");
//    Serial.println(dy);
    // move diagonally
    for(int i = 0; i <= 2*shortest; i++) {
      if (dx>=0 and dy>=0){
        diag_ul();
//        Serial.println("Moving ul");
      }else if (dx<=0 and dy>=0){
        diag_ur();
//        Serial.println("Moving ur");
      }else if (dx>=0 and dy<=0){
        diag_bl();
//        Serial.println("Moving bl");
      }else {
        diag_br();
//        Serial.println("Moving br");
      }
    }

    // move vertically/ horizontally
    for(int i=0; i <= remaining; i++){
      if (abs(dx) >= abs(dy)) {
        if (dx>=dy){
          left();
//          Serial.println("Left");
        }else {
          right();
//          Serial.println("Right");
        }
      }else {
        if (dy>=dx){
          up();
//          Serial.println("Up");
        }else {
          down();
//          Serial.println("Down");
        }
      }
    }

    position_x += dx;
    position_y += dy;
}

bool CW = false;
bool CCW = true;

void diag_ul() {
  digitalWrite(dirPin,CW); 
  digitalWrite(stepPin,HIGH);
  delayMicroseconds(20);
  digitalWrite(stepPin,LOW); 
  delayMicroseconds(20); 
  wait();
}
void diag_br() {
  digitalWrite(dirPin,CCW); 
  digitalWrite(stepPin,HIGH);
  delayMicroseconds(20);
  digitalWrite(stepPin,LOW); 
  delayMicroseconds(20); 
  wait();
}
void diag_ur() {
  digitalWrite(dirPin2,CCW);
  digitalWrite(stepPin2,HIGH);
  delayMicroseconds(20);
  digitalWrite(stepPin2,LOW); 
  delayMicroseconds(20); 
  wait();
}
void diag_bl() {
  digitalWrite(dirPin2,CW); 
  digitalWrite(stepPin2,HIGH);
  delayMicroseconds(20);
  digitalWrite(stepPin2,LOW); 
  delayMicroseconds(20); 
  wait();
}
void left() {
  digitalWrite(dirPin,0); 
  digitalWrite(dirPin2,0);
  step_motors();
}
void right() {
  digitalWrite(dirPin,1); 
  digitalWrite(dirPin2,1);
  step_motors();
}
void down() {
  digitalWrite(dirPin,1); 
  digitalWrite(dirPin2,0);
  step_motors();
}
void up() {
  digitalWrite(dirPin,0); 
  digitalWrite(dirPin2,1);
  step_motors();
}


void wait() {
    if ((steptime-40)<=10000){
    delayMicroseconds(steptime - 40);
  }else {
    steptime_ms = steptime/1000;
    delay(steptime_ms);
  } 
}
void step_motors() {
  digitalWrite(stepPin,HIGH); 
  digitalWrite(stepPin2,HIGH); 
  delayMicroseconds(20); 
  digitalWrite(stepPin,LOW); 
  digitalWrite(stepPin2,LOW); 
  delayMicroseconds(20); 
  wait();
}

ISR(TIMER1_COMPA_vect) {
    inputSeveral[0] = 0; 
    byte charCount = 0;  
    byte ndx = 0;        
    
    if (Serial.available() > 0) {
      while (Serial.available() > 0) { 
        if (ndx > maxChars - 1) {
          ndx = maxChars;
        } 
        inputSeveral[ndx] = Serial.read();
        ndx ++;        
        charCount ++;
      }
      if (ndx > maxChars) { 
        ndx = maxChars;
      }
      inputSeveral[ndx] = 0; 

      rpm = atoi(inputSeveral); // atof gives 0.0 if the characters are not a valid number
      if(rpm !=0){
        steptime = 300000/rpm;
      }
//      Serial.print(rpm);
    }

     // and then convert the string into a floating point number
     
    
//    
} 

void set_rpm(int speed_rpm){ //sets the speed
  if(speed_rpm == 0){
    rpm = 0;
  }else {
    rpm = speed_rpm;
    steptime = 300000/speed_rpm;
  }
  
}

int movedir = 0;
bool move_square = false;
bool move_line = false;
bool move_line_vert=false;
    
void loop() {
//digitalWrite(dirPin,0); 
//digitalWrite(dirPin2,0);

if (move_square == true){
  set_rpm(300);
  if(movedir == 0){
    digitalWrite(dirPin,0); // Enables the motor to move in a particular direction
    digitalWrite(dirPin2,0); // Enables the motor to move in a particular direction
    movedir += 1; //becomes 1
  }else if(movedir == 1){
    digitalWrite(dirPin,0); // Enables the motor to move in a particular direction
    digitalWrite(dirPin2,1); // Enables the motor to move in a particular direction
    movedir += 1; //becomes 2
  }else if(movedir == 2){
    digitalWrite(dirPin,1); // Enables the motor to move in a particular direction
    digitalWrite(dirPin2,1); // Enables the motor to move in a particular direction
    movedir +=1; //becomes 3
  }else {
    digitalWrite(dirPin,1); // Enables the motor to move in a particular direction
    digitalWrite(dirPin2,0); // Enables the motor to move in a particular direction
    movedir = 0; //becomes 0 again
  }
}

if (move_line == true){ //left first
  set_rpm(300);
  if(movedir == 0){
    digitalWrite(dirPin,0); // Enables the motor to move in a particular direction
    digitalWrite(dirPin2,0); // Enables the motor to move in a particular direction
    movedir += 1; //becomes 1
  }else {
    digitalWrite(dirPin,1); // Enables the motor to move in a particular direction
    digitalWrite(dirPin2,1); // Enables the motor to move in a particular direction
    movedir = 0; //becomes 0 again
  }
}

if (move_line_vert == true){ // up first
  set_rpm(200);
  if(movedir == 0){
    digitalWrite(dirPin,0); // Enables the motor to move in a particular direction
    digitalWrite(dirPin2,1); // Enables the motor to move in a particular direction
    movedir += 1; //becomes 1
  }else {
    digitalWrite(dirPin,1); // Enables the motor to move in a particular direction
    digitalWrite(dirPin2,0); // Enables the motor to move in a particular direction
    movedir = 0; //becomes 0 again
  }
}

if (rpm!=0 and 1==2){
  for(int x = 0; x < 500; x++) {
    digitalWrite(stepPin,HIGH); 
    digitalWrite(stepPin2,HIGH); 
    delayMicroseconds(20); 
    digitalWrite(stepPin,LOW); 
    digitalWrite(stepPin2,LOW); 
    delayMicroseconds(20); 
  
    if ((steptime-40)<=10000){
      delayMicroseconds(steptime - 40);
    }else {
      steptime_ms = steptime/1000;
      delay(steptime_ms);
    } 
  }
}




delay(5000);
//while(Serial.available() == 0) { }
//for(int i=0; i<=5300 ; i++){
//  up();
//}


}
