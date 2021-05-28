/*
 * singleAxisControl
 * 
 * This program sets a pre-defined angle as a setpoint, MPU9250 as input, and drives a motor as a output via PID controller
 * 
 * Written by Rhett A. Smith,
 * Advisors: Dr. Cheng Yang, Dr. Jichul Kim
 */

// libraries and definitions
 #include "MPU9250.h"     //Hideaki Tai

 MPU9250 mpu;             //re-define library by a shorter name

// define Pins
#define MOT_PIN 5         // motor enable
#define MOT_CCW 6         // counterclockwise spin
#define MOT_CW 7          // clockwise spin
#define LED 13            // LED pin

#define YAW_READ_DELAY 25 //MPU 9250 max read rate ~10ms !!!CHECK THIS!!! 

// define PID settings and gains
#define tol 4             // tolorance (deg)
#define OUT_MIN -250      // minimum PID output
#define OUT_MAX 250       // maximum PID output
#define KP 0.8            // proportional gain
#define KI 0.2            // integral gain
#define KD 0.6            // derivative gain
#define MOT_Saturate 100  // minimum input to saturate motor
#define MOT_Ignore 00     // ignore output if below this value


// reference variables
double yaw, setPoint, outputVal, currentError, preError, sumError, start, finish;
float P, I, D;

unsigned long lastYawUpdate;      //tracks clock time of last temp update

// Functions

//blinks the LED
void blinkLED() {
  digitalWrite(LED, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
  delay(100);
}//blinkLED()

// call repeatedly in loop, only updates after a certain time interval
// returns true if update happened
bool updateYaw() {
  if ((millis() - lastYawUpdate) > YAW_READ_DELAY) {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 25) {
            prev_ms = millis();
        }
    }
    yaw = mpu.getYaw();           // get yaw reading
    lastYawUpdate = millis();     // resets timer
    return true;
  }
  return false;
}// bool updateYaw()


// Error Calculation
double error(double setPoint,double yaw) {
  double Error;
  Error = setPoint-yaw;       // error calculation
  if (Error > 180) {          //Keeps error within [-180 180]
    Error = Error - 360;
  }
  else if (Error < -180) {
    Error = Error + 360;
  }
  currentError = Error;
  return currentError;
}//double error()


// PID Controller
double PID(double currentError, double preError, double sumError) {
  double OUT;
  P = KP * currentError;
  I = KI * sumError;
  D = KD * (currentError-preError)/(YAW_READ_DELAY)*1000;
  OUT = P + I + D;
  if (OUT > OUT_MAX) {    // Limit output to OUT_MAX
    OUT =  OUT_MAX;
  }
  else if (OUT < OUT_MIN) {
    OUT = OUT_MIN;
  }//limiter
  return OUT;
}//double PID()

// End functions

void setup() {
  pinMode(MOT_PIN, OUTPUT);
  pinMode(MOT_CCW, OUTPUT);
  pinMode(MOT_CW, OUTPUT);
  pinMode(LED, OUTPUT);
  
  // mpu9250setup
  Serial.begin(115200);
  Wire.begin();
  delay(1000);
  blinkLED();
  
  mpu.setup(0x68);  // change to your own address
  delay(1000);
  blinkLED();
  
  // calibrate anytime you want to
  mpu.verbose(true);
  mpu.calibrateAccelGyro();
  delay(500);
  //mpu.calibrateMag();
  delay(500);
  mpu.verbose(false);
  blinkLED();
  
  // !!!SETPOINT VALUE INPUT HERE!!!
  setPoint = 50;

  // Initial Error Values
  mpu.update();
  preError = setPoint - mpu.getYaw();
  sumError = 0;
  lastYawUpdate = millis();             // resets timer

  finish = 0;

  delay(1000);
  blinkLED(); blinkLED(); blinkLED();

  // allows operator to position the sat correctly
  while (start <= 200) {                // for the 1st 5 seconds
    // Measure Yaw
    while(updateYaw() == false) {};

    // Calculate Error
    currentError = error(setPoint, yaw);

    if (abs(currentError) <= 30 && abs(currentError >= 10)) {
      digitalWrite(LED, HIGH);
    }
    else {
      digitalWrite(LED, LOW);
    }
    start += 1;
  }
}// void setup()


// put your main code here, to run repeatedly:
void loop() {
  // Measure Yaw
  while (updateYaw() == false) {};                                // waits until yaw value is updated

  // Calculate Error
  currentError = error(setPoint, yaw);                            // finds the current error

  // PID Control
  outputVal = PID(currentError, preError, sumError);              // PID controller
  
  // !!!ADD A DIRECTION IF CHECK HERE!!! (may change the range for outputVal to be -255:255 and change based on polarity
  // outputVal = outputVal-250;                                      // changes values from [0 500] to [-250 250] NOT NEEDED SINCE PID CAN BE NEG
  
  if (outputVal < 0) { // && abs(outputVal) > abs(MOT_Ignore)) {  // if the output is negative and greater than the ignore point
    digitalWrite(MOT_CCW, LOW);
    digitalWrite(MOT_CW, HIGH);
    /*
    if (abs(outputVal) < MOT_Saturate) {
      outputVal = MOT_Saturate;
    }
    */
  }
  else if (outputVal > 0) { // && abs(outputVal) > abs(MOT_Ignore)) {     //if the output is positive and greater than the ignore point
    digitalWrite(MOT_CCW, HIGH);
    digitalWrite(MOT_CW, LOW);
    /*
    if (abs(outputVal) < MOT_Saturate) {
      outputVal = MOT_Saturate;
    }
    */
  }
  else if (abs(outputVal) <= MOT_Ignore) {                          //if the output is below the ignore point
    digitalWrite(MOT_CCW, LOW);
    digitalWrite(MOT_CW, LOW);
    outputVal = 0;
  }

  // Outputs PID control to motor
  analogWrite(MOT_PIN, abs(outputVal)); //if polar, use abs(outputVal)

  // Set preError and sumError
  preError = currentError;
  sumError += currentError;

  // Light up LED if within tolorance of setPoint
  if (abs(currentError) < tol) {
    digitalWrite(LED, HIGH);
    finish += 1;
    // if more than 2 seconds pass and it is still at setPoint, end program
    if (finish >= 80) {
      analogWrite(MOT_PIN, 0);
      while(1) {
        blinkLED();
      }
    }
  }
  else if (abs(currentError) > tol) { // Turns off the LED if it leaves setPoint
    digitalWrite(LED, LOW);
    finish = 0;
  }
  
  //Print Check
  /*
  Serial.print(outputVal);
  Serial.print(F(" | "));
  Serial.print(currentError);
  Serial.print(F(" | "));
  Serial.println(yaw);
  */
  
  Serial.print(setPoint);
  Serial.print(',');
  Serial.println(yaw);
}// void loop()
