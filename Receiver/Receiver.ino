#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

//Motor 1 -> links
const int enableMotor1 = 2;
const int forwardMotor1 = 10;
const int backwardsMotor1 = 9;

//Motor 2 -> rechts
const int enableMotor2 = 4;
const int forwardMotor2 = 5;
const int backwardsMotor2 = 3;

//Adress for comunication
const byte address[6] = "00001";
//Ofset for joystick values
const int ctrlX_Offset = 514;
const int ctrlY_Offset = 525;
const float SPEED_ADAPTION_RATIO_X = 0.35;
const float SPEED_ADAPTION_RATIO_Y = 0.5;
//If you only drive straigt this ist the max value that the motors can see to make the car more controleable
const int MAX_PWM_MOTOR_X_DIRECTION = 160;
//If you drive only right this is the max values that the motors can see. It is supposed to be about half of X_Direction because this enables you to drive straigt and right/left
const int MAX_PWM_MOTOR_Y_DIRECTION = 80;
const int MOTOR_DRIVER_HIGHEST_VALUE = 255;
const int MOTOR_DRIVER_LOWEST_VALUE = 0;
int ctrlX = 0;
int ctrlY = 0;
int pwmMotorX = 0;
int pwmMotorY = 0;
//Calculated from the values above and adapted continiously.
int pwmMotorRight = 0;
int pwmMotorLeft = 0;
bool drivesForeward = false;

//Radio transmission object
RF24 radio(7, 8); // CE, CSN

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  //radio.setPALevel(RF24_PA_MIN); -> Not used :) I think this is usefull if you have unstable power supply but not 100% sure. Default = High PA Level = stronger signal
  radio.startListening();

  pinMode(enableMotor1, OUTPUT);
  digitalWrite(forwardMotor1, LOW); //Set them low at the start so they don't randomly start spinning
  digitalWrite(backwardsMotor1, LOW); //Set them low at the start so they don't randomly start spinning
  digitalWrite(enableMotor1, HIGH);
  pinMode(forwardMotor1, OUTPUT);
  pinMode(backwardsMotor1, OUTPUT);

  //Motor2
  pinMode(enableMotor2, OUTPUT);
  digitalWrite(forwardMotor2, LOW); //Set them low at the start so they don't randomly start spinning
  digitalWrite(backwardsMotor2, LOW); //Set them low at the start so they don't randomly start spinning
  digitalWrite(enableMotor2, HIGH);
  pinMode(forwardMotor2, OUTPUT);
  pinMode(backwardsMotor2, OUTPUT);
}

/**
 * Main Loop:
 * 1. Wait for any availabe radio signals
 * 2. Read the radio signals from the transmitter (Array consisting of the following things):
 * data[0] -> Joystick x-value (Raw data red with 10 bit resolution = 0...1023)
 * data[1] -> Joystick y-value (Raw data red with 10 bit resolution = 0...1023)
 * data[2] -> Joystick button-value (Pressed = 0; Not pressed = 1);
 * 3. Update the PWM into the x-direction: pwmMotorX
 * 4. Update the PWM into the y-direction: pwmMotorY
 * 5. Calculate the Motor speeds with pwmMotorX and pwmMotorY -> calculateMotorSpeeds()
 * 6. Adapt the Motor speed -> adaptMotorSpeed()
 */
void loop() {
  if (radio.available()) {
    int data[3] = {0};
    radio.read(&data, sizeof(data));
    //driving foreward should correspond to positive values
    ctrlX = -(data[0] - ctrlX_Offset);
    //driving right should correspond to positive values
    ctrlY = -(data[1] - ctrlY_Offset);
    calculateXPWM();
    calculateYPWM();
    calculateMotorSpeeds();
    adaptMotorSpeed();
  }
}

/**
 * From the Centered Joystick value in the X-direction (ctrlX) calculate the pwmMotor output into the X-direction
 * Difficulty: Joystick is in rest at the value 514 which has to be changed for different Joysticks. -> ctrlX_Offset
 * Therefore the values are: -509 -> 514. With 0 beeing the joystick in rest position. 
 * A deadband is added between -5 and 5 to make the motors stop if the joystick is in rest. Otherwise, due to small sensor value changes the the motors would always try to move.
 * To calculate the pwmMotorX output we use a compl. filter to reduce high frequencies. -> The car is easier to maneuver. 
 * pwmMotorX = SPEED_ADAPTION_RATIO_X * scaledCtrlX * MAX_PWM_MOTOR_X_DIRECTION + (1-SPEED_ADAPTION_RATIO_X) * pwmMotorX
 * 
 * SPEED_ADAPTION_RATIO_X * scaledCtrlX * MAX_PWM_MOTOR_X_DIRECTION -> First part of the filter: Fast changing Joystick values
 * SPEED_ADAPTION_RATIO_X -> Filter constant to integrate old values and reduce fast changes which result in an uncontrollable car behaviour
 * scaledCtrlX * MAX_PWM_MOTOR_X_DIRECTION -> scaledCtrlX (= from [0,1]/[-1,0] depending on the Joystick position) * MAX_PWM_MOTOR_X_DIRECTION (=Max possible speed if you only drive straight for example)
 * scaledCtrlX * MAX_PWM_MOTOR_X_DIRECTION -> returns a value from 0 to MAX_PWM_MOTOR_X_DIRECTION depending on the current Joystick positioning in the x-direction
 * (1-SPEED_ADAPTION_RATIO_X) * pwmMotorX -> Second part of the filter: Keep the old values to reduce high frequency changes
 * pwmMotorX -> The old pwmMotorX value
 * 
 * This is done for positive and negative pwmMotorX-values. Depending on wheter the Joystick points into the front or back we get positive or negative pwmValues between 0 and MAX_PWM_MOTOR_X_DIRECTION.
 * Those are then used to calculate the PWM signal for the left and right motors. Furthermore we set the drivesForeward boolean value. This is needed to adapt the motor directions later.
 */
void calculateXPWM() {
  if (ctrlX > 5) { //Driving left
      //Scale the control value to values between 0 and 1. You have to substract 5 first to compensate the deadzone.
      float scaledCtrlX = (ctrlX - 5) / 511.0;
      //Adapt the motor speed but not too fast -> Incooperate the old speed value.
      pwmMotorX = SPEED_ADAPTION_RATIO_X * scaledCtrlX * MAX_PWM_MOTOR_X_DIRECTION + (1 - SPEED_ADAPTION_RATIO_X) * pwmMotorX;
      drivesForeward = true;
    } else if (ctrlX < -5) { //Driving backward
      //Scale the control value to values between -1 and 0. You have to add 5 first to compensate the deadzone.
      float scaledCtrlX = (ctrlX + 5) / 504.0;
      //Adapt the motor speed but not too fast -> Incooperate the old speed value.
      pwmMotorX = SPEED_ADAPTION_RATIO_X * scaledCtrlX * MAX_PWM_MOTOR_X_DIRECTION + (1 - SPEED_ADAPTION_RATIO_X) * pwmMotorX;
      drivesForeward = false;
    } else {  //Stop
      // -> scaledCtrlX = 0 Therefore SPEED_ADAPTION_RATIO * scaledCtrlX * MAX_PWM_MOTOR == 0
      pwmMotorX = (1 - SPEED_ADAPTION_RATIO_X) * pwmMotorX;
      //The direction is not supposed to change. It should just slow down.
    }
}

/**
 * From the Centered Joystick value in the Y-direction (ctrlY) calculate the pwmMotor output into the Y-direction
 * Difficulty: Joystick is in rest at the value 525 which has to be changed for different Joysticks. -> ctrlY_Offset
 * Therefore the values are: -498 -> 525. With 0 beeing the joystick in rest position. 
 * A deadband is added between -5 and 5 to make the motors stop if the joystick is in rest. Otherwise, due to small sensor value changes the the motors would always try to move.
 * To calculate the pwmMotorY output we use a compl. filter to reduce high frequencies. -> The car is easier to maneuver. 
 * pwmMotorY = SPEED_ADAPTION_RATIO_Y * scaledCtrlY * MAX_PWM_MOTOR_Y_DIRECTION + (1-SPEED_ADAPTION_RATIO_Y) * pwmMotorY
 * 
 * SPEED_ADAPTION_RATIO_Y * scaledCtrlY * MAX_PWM_MOTOR_Y_DIRECTION -> First part of the filter: Fast changing Joystick values
 * SPEED_ADAPTION_RATIO_Y -> Filter constant to integrate old values and reduce fast changes which result in an uncontrollable car behaviour
 * scaledCtrlY * MAX_PWM_MOTOR_Y_DIRECTION -> scaledCtrlY (= from [0,1]/[-1,0] depending on the Joystick position) * MAX_PWM_MOTOR_Y_DIRECTION (= Max possible speed if you only drive right for example)
 * scaledCtrlY * MAX_PWM_MOTOR_Y_DIRECTION -> returns a value from 0 to MAX_PWM_MOTOR_Y_DIRECTION depending on the current Joystick positioning in the y-direction
 * (1-SPEED_ADAPTION_RATIO_Y) * pwmMotorY -> Second part of the filter: Keep the old values to reduce high frequency changes
 * pwmMotorY -> The old pwmMotorY value
 * 
 * This is done for positive and negative pwmMotorY-values. Depending on wheter the Joystick points into the right or left direction we get positive or negative pwmValues between 0 and MAX_PWM_MOTOR_Y_DIRECTION.
 * Those are then used to calculate the PWM signal for the left and right motors
 */
void calculateYPWM() {
  if (ctrlY > 5 ) { //Driving right
      float scaledCtrlY = (ctrlY - 5) / 520.0;
      pwmMotorY = SPEED_ADAPTION_RATIO_Y * scaledCtrlY * MAX_PWM_MOTOR_Y_DIRECTION + (1-SPEED_ADAPTION_RATIO_Y) * pwmMotorY;
    } else if (ctrlY < -5) { //Nach rechts
      float scaledCtrlY = (ctrlY + 5) / 493.0;
      pwmMotorY = SPEED_ADAPTION_RATIO_Y * scaledCtrlY * MAX_PWM_MOTOR_Y_DIRECTION + (1-SPEED_ADAPTION_RATIO_Y) * pwmMotorY;
    } else {
      pwmMotorY = (1-SPEED_ADAPTION_RATIO_Y) * pwmMotorY;
    }
}
/**
 * Calculate the pwm Signal for the right and left motor. It uses the PWM-Motor signals into the X and Y direction:
 * pwmMotorX > 0 -> Driving front 
 * pwmMotorX < 0 -> Driving backwards
 * pwmMotorY > 0 -> Drive right, left motor has to spin faster; pwmMotorX + pwmMotorY
 * pwmMotorY < 0 -> Drive left, right motor has to spin faster; pwmMotorX - pwmMotorY (pwmMotorY is neg so this term increases)
 * If the values are above 255 or below 0 the L298N would stop spinning -> Set them to those max values
 */
void calculateMotorSpeeds() {
  pwmMotorRight = abs(pwmMotorX) - pwmMotorY;
  pwmMotorLeft = abs(pwmMotorX) + pwmMotorY;
  if (abs(pwmMotorRight) > MOTOR_DRIVER_HIGHEST_VALUE) {
    pwmMotorRight = MOTOR_DRIVER_HIGHEST_VALUE;
  } else if ( pwmMotorRight < MOTOR_DRIVER_LOWEST_VALUE) {
    pwmMotorRight = MOTOR_DRIVER_LOWEST_VALUE;
  }
  if (abs(pwmMotorLeft) > MOTOR_DRIVER_HIGHEST_VALUE) {
    pwmMotorLeft = MOTOR_DRIVER_HIGHEST_VALUE;
  } else if ( pwmMotorLeft < MOTOR_DRIVER_LOWEST_VALUE) {
    pwmMotorLeft = MOTOR_DRIVER_LOWEST_VALUE;
  }
}
/**
 * Adapts the Motor speed according to the previously calculated pwm-Signales. 
 * Changes the direction into which the cars wheels are spinning.
 */
void adaptMotorSpeed() {
  if (drivesForeward) {
    analogWrite(forwardMotor2, pwmMotorRight);
    digitalWrite(backwardsMotor2, LOW);
    analogWrite(forwardMotor1, pwmMotorLeft);
    digitalWrite(backwardsMotor1, LOW);
  } else {
    analogWrite(backwardsMotor2, pwmMotorRight);
    digitalWrite(forwardMotor2, LOW);
    analogWrite(backwardsMotor1, pwmMotorLeft);
    digitalWrite(forwardMotor1, LOW);
  }
}
