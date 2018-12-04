/*

State Machine Gizmo

Author: Tom Pais [] special thanks to @j-bellavance for his online tutorials on state machines and to @pierreazalbert for his FSM structure 

*/

#include <Adafruit_NeoPixel.h>
#include "SR04.h"
#include <Stepper.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define IDLE_STATE 0
#define OPEN_STATE 1
#define CLOSED_STATE 2
#define CONTROL_STATE 3
#define DISCO_STATE 4

#define MOTOR_LEFT 4096
#define MOTOR_RIGHT -4096
#define MOTOR_STOP 0
#define MOTOR_DELAY 5000

#define SERVOMIN  150
#define SERVOMAX  550

#define NUMPIXELS 60

// ************ PIN DEFINITIONS ************************************************************

// Ultrasonic sensor
const int trigPin = 13; // digital pin
const int echoPin = 11; // digital pin

// Motor output
const int stepsPerRevolution = 2048; 
Stepper myStepper(stepsPerRevolution, 5, 9, 6, 10); // initializing servo motor through servo driver pins

// Touch sensor
const int touchPin = A2; // analog pin

// Soft potentiometer
const int softpotPin = A4; // analog pin

// LEDs
const int ledPin = A3; // analog pin written to as digital I/O pin

// Joystick
const int swPin = 12; // digital pin connected to switch output
const int xPin = A0; // analog pin connected to X output
const int yPin = A1; // analog pin connected to Y output

// ************ VAR DEFINITIONS ************************************************************

// Ultrasonic sensor
SR04 sr04 = SR04(echoPin, trigPin);
int startFlag; // triggers start of time count using millis() if distance <= 10cm
int startTime; // initial time when new rangeduration count event is started
int previousTime; // holds value of previous time for conditions
int duration;
int rangeDuration; // holds value of 1 (user present for more than 5 seconds) or 0 (user not present for more than 5 seconds)
int distance; // stores distance from analogread()
int newRange; // 0:stop, 1:mid speed, 2: full speed
int openRange; // holds value of 1 in OPEN state and 0 in CLOSED state

// Motor output
int currentSteps = 0; // current amount of steps taken of stepper motor
const int rolePerMinute = 15; // adjustable range of 28BYJ-48 stepper is 0~17 rpm

// Touch sensor
int currentTouch = 0; // touch value from pin 11
int controlON = 0;// disco control status
int touched = 0;// touch status

// Servo wig
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40); // initializing servo driver
//Servo servoWig;
//int initialPosition = 0; // assigning 0 degrees to initial position of servo

// LEDs
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, ledPin, NEO_GRB + NEO_KHZ800);
uint32_t low = pixels.Color(0, 0, 0); // define an off state for the Neopixel strip 
uint32_t high = pixels.Color(255, 255, 255); // define a white on state for the Neopixel strip 

// Finite State Machine
unsigned int state;

//// ************ CUSTOM FUNCTION

int getDistance() {

  // get distance from SR04 ultrasound distance sensor

  int distance;
  int dur;
  distance = sr04.Distance();
  dur = getRangeduration();

  Serial.print("Distance: ");
  Serial.println(distance);

  // return 0 if distance is more than 10 cm
  if (distance > 10) {
    return 0;
  }
  // return 1 if distance is less than or equal to 10 cm
  else if (distance <= 10) {
    return 1;
  }
  // return 2 if distance of less than or equal to 10 cm is maintained for over 5 seconds
  else if (dur == 1) {
    return 2;
    Serial.print("dur: ");
    Serial.println(dur);
}
}

int getRangeduration() {

// detect whether user is present within 10 cm for more than 5 seconds, if yes return 1, if not return 0
distance = sr04.Distance();

if  ((distance <= 10) && (startFlag == 0) ){ // start new time sequence
   startFlag = 1;
   startTime = millis();
   previousTime = startTime;
}
if ( (startFlag ==1) && ((millis() - previousTime) >= duration) ) {  // duration = 100, sample 10 times a second
   previousTime = previousTime + duration;
   if ((millis() - startTime )<=5000){ // still waiting it out
    if (distance > 10){
       // pin went low
       startFlag = 0;
       return 0;
       }
    }
   if ((millis() - startTime) >5000){ 
       if (distance >= 10){
       // pin went low
       startFlag = 0;
       return 0;
       }
      else{  // made it!
      startFlag = 0;
      return 1;
      }
   }
}
}



int getJoyXpos() {
  
  // detect position on x axis of joystick, return 0 if down, 1 if up
  int xPos; 
  xPos = analogRead (xPin);
  
  // Prints the distance on the Serial Monitor
  Serial.print("X position: ");
  Serial.println(xPos);

  if (xPos < 300) {
    return 0;
  }
  else if (xPos > 900) {
    return 1;
  }
}

int getClickbutton() {

  // detect whether joystick is clicked or not, if clicked return 0, if not clicked return 1
  int clicked; 
  clicked = digitalRead (swPin);
  
  // Prints the distance on the Serial Monitor
  Serial.print("Joystick clicked: ");
  Serial.println(clicked);

  if (clicked == LOW) {
    return 1;
  }
  else {
    return 0;
  }
}

// declaring all functions before void setup() and void loop(), to circumvent compiling issues
void activateLEDs();
void motorStop();
void motorLeft();
void setServoPulse(uint8_t n, double pulse);
void detectRange();
void detectTouch();
void detectStartbutton();
void moveWig();
void activateLEDsDisco(int SpeedDelay);


// ************ SET UP **********
void setup() {

  // Debugging
  Serial.begin(9600); // starts the serial communication

  // Ultrasonic sensor
  pinMode(trigPin, OUTPUT); // sets the trigPin as an Output
  pinMode(echoPin, INPUT); // sets the echoPin as an Input

  // Motor
  myStepper.setSpeed(rolePerMinute); // sets the speed of stepper motor to predefined roleperminute

  // Touch sensor
  pinMode(touchPin, INPUT); // sets the touchPin as an Input

  // Servo wig
  pwm.begin(); // initializing the Servodriver library.
  pwm.setPWMFreq(60);
  pwm.setPWM(0,0,SERVOMAX); // writing starting position to servoWig using PWM
  delay(500);

  // LEDs
  pixels.begin(); // initializing the NeoPixel library.
  pixels.show(); // initializing the NeoPixel strip to an off mode

  // Joystick
  pinMode(xPin, INPUT); // sets the xPin as an Input
  pinMode(yPin, INPUT); // sets the yPin as an Input
  pinMode(swPin, INPUT_PULLUP); // sets the swPin as an Input using built-in resistor
  
  // State Machine
  state = IDLE_STATE; // initialized IDLE_STATE as current state
 
}

// ************ STATES **********

void loop() {

  switch (state)
  {

    // IDLE - Check for presence of user - If present go to OPEN state to open door
    case IDLE_STATE:

      // Turn off motor
      motorStop();

      // Check if user is here - if there is a user go to OPEN state
      detectRange();

      break;

    // OPEN - User detected, door moves left. Check for presence of 5 seconds within 7 cm, if so then move to CLOSED
    case OPEN_STATE:

      // Move door left
      motorLeft();

      // Check if user is present (again) for 5+ seconds, if so then move to CLOSED state
      detectRange();

      break;

      // CLOSED - User within range again for 5+ seconds, door moves right, LEDs are activated. Check for touch.
    case CLOSED_STATE: 

      // Move door right
      motorRight();

      // Detect touch on wooden base, if touched then move to CONTROL state 
      detectTouch();

      break;

    // CONTROL - Wooden base touched on secret spot, check for state of start button, if clicked then move to DISCO state
    case CONTROL_STATE:

      // Detect click on start button, if clicked then move to DISCO state
      detectStartbutton();

      break;

    // DISCO - Start button clicked, wig moved up, LEDsDisco activated. Detect touch to stop.
    case DISCO_STATE:

      // Move wig up and down
      moveWig();

      // Activate disco LEDs  
      activateLEDsDisco(20);

      // Detect touch for turning off DISCO mode 
      detectTouch();

      break;
      

  }
}


// ************ CUSTOM FUNCTIONS **********


void motorStop() {
  // Move door to the middle
  if (currentSteps != MOTOR_STOP) {
 if (currentSteps == MOTOR_LEFT) { 
    Serial.print("Returning to Idle position");
    myStepper.step(2*stepsPerRevolution);
    delay(MOTOR_DELAY);
    }
    currentSteps = MOTOR_STOP;
  }
  else if (currentSteps == MOTOR_STOP) {
  currentSteps = MOTOR_STOP;
  delay(MOTOR_DELAY);
  }
}


void motorLeft() {
  // Move door to the left (= open position)
  // If current amount of steps does not match steps taken to move door left, then move door left
  if (currentSteps != MOTOR_LEFT) {
    if (currentSteps == MOTOR_STOP) {
      Serial.println("Door opening");
      myStepper.step(MOTOR_LEFT);
      delay(MOTOR_DELAY);
      }
      currentSteps = MOTOR_LEFT;
    }
  }


void motorRight() {
  // Move door to the right (= closed position)
  // If current amount of steps does not match steps taken to move door back right, then move door right
  if (currentSteps != MOTOR_RIGHT) {
    if (currentSteps == MOTOR_LEFT) {
      Serial.println("Door closing");
      myStepper.step(MOTOR_RIGHT);
      delay(MOTOR_DELAY);
      }
      currentSteps = MOTOR_RIGHT;
    }
  }


void detectRange() {

  // Detect presence and/or distance of user
  int newRange;
  newRange = getDistance();
  rangeDuration= getRangeduration();

 if ((newRange == 0) && (openRange == 0)){
    state = IDLE_STATE;
    Serial.println("Transitioned to IDLE state");
    Serial.println(openRange);
  }

  else if ((newRange == 1) && (openRange == 0)) {
    state = OPEN_STATE;
    Serial.println("Transitioned to OPEN state");
    openRange = 1;
    Serial.println(openRange);
      }

//  if ((newRange == 1) && (rangeDuration == 1)) {
  else if ((newRange == 1) && (openRange == 1)) {
    state = CLOSED_STATE;
    Serial.println("Transitioned to CLOSED state");
    motorRight();
    delay(2000);
    activateLEDs();
  }       
}


void detectTouch() {
  
  // using capacitive touch sensor as switch between OPEN, CLOSED & DISCO states
  currentTouch = digitalRead(touchPin);
  if(currentTouch == HIGH && controlON == LOW){
    touched = 1-touched;
    delay(100);
  }    
  controlON = currentTouch;
  
  if((touched == HIGH) && (state == CLOSED_STATE)){
     Serial.println("Transitioned to CONTROL state");
     state = CONTROL_STATE;
     }
  else if ((touched == HIGH) && (state == DISCO_STATE)){
   Serial.println("Byeee");
   state = OPEN_STATE; 
  }
  else{
  Serial.println("Control mode not engaged");
     }     
  delay(100); 
}


void detectStartbutton() {
  
  // detect when start button is clicked
  int startButton;
  startButton = getClickbutton(); 
  if (startButton == 1){
    state = DISCO_STATE;
    Serial.print("Transitioned to DISCO state");
  }
}

void moveWig() {
  
  // move wig up and down according to  x movement on joystick
  int wigMove;
  wigMove = getJoyXpos();
  
  if (wigMove == 1) {
  Serial.println("WIG UP");
   pwm.setPWM(0,0,200);
    delay(500);
  }
  else if (wigMove == 0) {
    Serial.println("WIG DOWN");
    pwm.setPWM(0,0,550);
    delay(500);
  }
}


void activateLEDs() {
  
   
  // set LEDs to activated white mode when state is equal to CLOSED
  if (state == CLOSED_STATE){
  for( int i = 0; i<NUMPIXELS; i++){
    pixels.setPixelColor(i, high);
    pixels.show();
    }
  }

  // set LEDs to activated white mode when state is equal to CONTROL
  else if (state == CONTROL_STATE){
  for( int i = 0; i<NUMPIXELS; i++){
    pixels.setPixelColor(i, high);
    pixels.show();
    }
  }

  // set LEDs to de-activated  mode when state is not equal to CLOSED or CONTROL
  else {
  for( int i = 0; i<NUMPIXELS; i++){
    pixels.setPixelColor(i, low);
    pixels.show();
    }
  }
}

void activateLEDsDisco(int SpeedDelay) {
  
  // activate LEDsDisco by cycling different colours through different pixels
  byte *c;
  uint16_t i, j;
  
  // 5 cycles of all colors on wheel
  for(j=0; j<256*5; j++) { 
    for(i=0; i< NUMPIXELS; i++) {
      c=Wheel(((i * 256 / NUMPIXELS) + j) & 255);
      pixels.setPixelColor(i, *c, *(c+1), *(c+2));
    }
    pixels.show();
    delay(SpeedDelay);
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
byte * Wheel(byte WheelPos) {
  static byte c[3];
  
  if(WheelPos < 85) {
   c[0]=WheelPos * 3;
   c[1]=255 - WheelPos * 3;
   c[2]=0;
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   c[0]=255 - WheelPos * 3;
   c[1]=0;
   c[2]=WheelPos * 3;
  } else {
   WheelPos -= 170;
   c[0]=0;
   c[1]=WheelPos * 3;
   c[2]=255 - WheelPos * 3;
  }

  return c;
}
