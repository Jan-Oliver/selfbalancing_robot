#include <Wire.h>
#include <math.h>
#define MPU_I2C 0b1101000

//NRF - RC Includes
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>


//#define DEBUG 
//Offsets for the IMU values
const float X_ACCEL_OFFSET = 890.21;
const float Y_ACCEL_OFFSET = 489.55;
const float Z_ACCEL_OFFSET = 45.49;
const float X_GYRO_OFFSET = -9.55;
const float Y_GYRO_OFFSET = 85.11;
const float Z_GYRO_OFFSET = 108.02;


//Constants which depend on the selected sensitivity of the IMU
const float ACCEL_SENSITIVITY_CONVERSION = 16384.0;
const float GYRO_SENSITIVITY_CONVERSION = 131.0;

//The Complementary filter needs a weight for the gyro data and the accel data to melt them
const float COMPL_FILTER_GYRO_WEIGHT = 0.9996;
struct AxisValues {
  float X;
  float Y;
  float Z;
}; 
//The raw data from the IMU
AxisValues accel_Raw;
AxisValues gyro_Raw;

//The values adapted to the current sensivity of the IMU
AxisValues accel;
AxisValues gyro;

//The estimated roll and pitch angles estimated by the accel data
float roll_Angle_Accel;
float pitch_Angle_Accel;
//The estimated roll and pitch angles estimated by the gyro data
float roll_Angle_Gyro;
float pitch_Angle_Gyro;

//To integrate the gyro-data you have to store the last timestempt to form a deltaTime
long oldTime;

//The estimated roll and pitch angel by the Complementary filter
float roll_Angle;
float pitch_Angle;

/**
 * Variables for PID
 */
float pid_P_Gain = 1.7;                                       //Gain setting for the P-controller 
float pid_I_Gain = 0.16;                                       //Gain setting for the I-controller 
float pid_D_Gain = 1;                                       //Gain setting for the D-controller 

float pid_Error,pitch_Error, pidI_Cum_Error, pidD_Last_Error, pid_Setpoint, pid_Output, self_balance_pid_setpoint;
float pid_Output_Left, pid_Output_Right;

/**
 * Variables for the motors
 */
const int right_Motor_Step = 7; 
const int right_Motor_Dir = 8; 
const int left_Motor_Step = 6; 
const int left_Motor_Dir = 5; 

int throttle_Counter_Motor_Right,throttle_Counter_Motor_Left,throttle_Motor_Memory_Right,throttle_Motor_Memory_Left;
int throttle_Motor_Left, throttle_Motor_Right;
/**
 * Variables for the voltage-measurement
 */
const int battery_Voltage_Measurement = A0;
const int BATTERY_VOLTAGE_NEW_VAL_WEIGHT = 0.05;
int battery_Voltage;
boolean batteryIsLow = false;

/**
 * RC Variables for NRF24L
 */
 //Variables to store the transmitted data
float pitchAngleNRF = 0.0;          //Raw pitch angle value which is received
float rollAngleNRF = 0.0;           //Raw roll angle value which is received

//Variables to setup NRF
const byte address[6] = "00001";
RF24 radio(9, 10); // CE, CSN
//Variable for control logic if no data is sent
int cyclesNoDataReceived = 0;
int maxSpeedMotors = 10;
void setup() {
  Serial.begin(9600);
  Wire.begin();

  //Set all Motor-Pins as Outputs
  pinMode(right_Motor_Dir,OUTPUT);
  pinMode(left_Motor_Dir,OUTPUT);
  pinMode(right_Motor_Step,OUTPUT);
  pinMode(left_Motor_Step,OUTPUT);

  //Measure the Battery Voltage
  pinMode(battery_Voltage_Measurement, INPUT);
  
  //To create a variable pulse for controlling the stepper motors a timer is created that will execute a piece of code (subroutine) every 20us
  //This subroutine is called TIMER2_COMPA_vect
  TCCR1A = 0;                                                               //Make sure that the TCCR1A register is set to zero initially
  TCCR1B = 0;                                                               //Make sure that the TCCR1B register is set to zero initially
  
  TCCR1B |= (1 << WGM12);                                                   //Set counter 1 to CTC (clear timer on compare) mode -> So you don't have to reset the timer in the ISR

  TCCR1B &= ~(1 << CS12);                                                   // 0 1 0 clkI/O/8 (prescaler) -> Set the 0
  TCCR1B |= (1 << CS11);                                                    // 0 1 0 clkI/O/8 (prescaler) -> Set the 1
  TCCR1B &= ~(1 << CS10);                                                   // 0 1 0 clkI/O/8 (prescaler) -> Set the 0

  TCNT1  = 0;                                                               //Set the initial timer value to 0
  OCR1A = 39;                                                               //Set the compare value to 39 -> 16MHz/[8(prescaler)*(1/20 * 10 ^-6)] - 1 = 39 
 
  TIMSK1 = (1 << OCIE1A);                                                   // Output Compare A Match Interrupt Enable

  sei();                                                                    //allow interrupts

  setupMPU();
  setupNRF();
}

void setupMPU() {
  Wire.beginTransmission(MPU_I2C); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(MPU_I2C); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(MPU_I2C); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
  //Set some filtering to improve the raw data.
  Wire.beginTransmission(MPU_I2C);                                     //Start communication with the address found during search
  Wire.write(0x1A);                                                         //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                         //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();  
}

void setupNRF() {
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.startListening();
}

void loop() {
  //Read NRF-Data if available
  if (radio.available()){                    //Wait for availabe data
    //Get and store the data
    int data[2] = {0};                      //If data is availabe create array with 5 slots to store data
    radio.read(&data, sizeof(data));        //Write the red data into the created array
    rollAngleNRF = -data[0];                //Store roll Angle and change the sign because of IMU positioning -> Move downwards should correspond to positive acceleration of the car
    pitchAngleNRF = data[1];                //Store pitch Angle
    cyclesNoDataReceived = 0; 
    #ifdef DEBUG
    Serial.print("NRF DATA-> Roll: ");
    Serial.print(rollAngleNRF);
    Serial.print("; Pitch: ");
    Serial.println(pitchAngleNRF);
    #endif          
  } else {
    cyclesNoDataReceived++;
  }

  if (cyclesNoDataReceived > 30) {
    pid_Setpoint = 0;
  }
  
  
  //Read Voltage
  battery_Voltage = (analogRead(battery_Voltage_Measurement) * 1.637) - 80;
  if(battery_Voltage < 1320 && battery_Voltage > 800) {
    batteryIsLow = true;
  }

  
  //Record Acceleration data
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accel_Raw.X = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accel_Raw.Y = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accel_Raw.Z = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ

  //Record Gyro data
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyro_Raw.X = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyro_Raw.Y = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyro_Raw.Z = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ

  //Apply the accelerometer offsets
  accel_Raw.X -= X_ACCEL_OFFSET;
  accel_Raw.Y -= Y_ACCEL_OFFSET;
  accel_Raw.Z -= Z_ACCEL_OFFSET;
  
  //Apply the gyroscope offsets
  gyro_Raw.X -= X_GYRO_OFFSET;
  gyro_Raw.Y -= Y_GYRO_OFFSET;
  gyro_Raw.Z -= Z_GYRO_OFFSET;

  //Convert raw accel data
  accel.X = accel_Raw.X / ACCEL_SENSITIVITY_CONVERSION ;
  accel.Y = accel_Raw.Y / ACCEL_SENSITIVITY_CONVERSION ; 
  accel.Z = accel_Raw.Z / ACCEL_SENSITIVITY_CONVERSION ;

  //Convert raw gyro data
  gyro.X = gyro_Raw.X / GYRO_SENSITIVITY_CONVERSION;
  gyro.Y = gyro_Raw.Y / GYRO_SENSITIVITY_CONVERSION; 
  gyro.Z = gyro_Raw.Z / GYRO_SENSITIVITY_CONVERSION;

  roll_Angle_Accel = atan2(accel_Raw.Y , accel_Raw.Z)/(2*M_PI)*360;
  pitch_Angle_Accel = atan2(-accel_Raw.X , sqrt((accel_Raw.Y*accel_Raw.Y)+(accel_Raw.Z*accel_Raw.Z)))/(2*M_PI)*360;
  
  long currentTime = millis();
  double dt = (currentTime - oldTime)/1000.0;
  //Calculate the estimated gyro angles
  roll_Angle_Gyro += gyro.X * dt;
  pitch_Angle_Gyro += gyro.Y * dt;

  //Estimated final compl. filter angles
  //roll_Angle = COMPL_FILTER_GYRO_WEIGHT * (roll_Angle + gyro.X * dt) + (1 - COMPL_FILTER_GYRO_WEIGHT) * roll_Angle_Accel;
  pitch_Angle = COMPL_FILTER_GYRO_WEIGHT * (pitch_Angle + gyro.Y * dt) + (1 - COMPL_FILTER_GYRO_WEIGHT) * pitch_Angle_Accel;

/**
 * PID-Controller:
 * pid_Error: Variable, which stores the Error between what the MPU measures and what the angle is supposed to be
 * pidI_Cum_Error: This is the integral which is done by adding all errors up
 * pidD_Last_Error: The Last Error is needed to calculate the differential of the pid-Controller
 * pid_Setpoint: The point we want to be stable at -> 0 degrees in our case
 * pid_Output: Final PID-Output value which has to be translated into a Motor speed and direction
 * float pid_P_Gain: Gain setting for the P-controller 
 * float pid_I_Gain: Gain setting for the I-controller 
 * float pid_D_Gain: Gain setting for the D-controller      
 */
 
  pid_Error = pitch_Angle - pid_Setpoint - self_balance_pid_setpoint;
  
  //if(pid_Output > 10 || pid_Output < -10)pid_Error += pid_Output * 0.015 ;

  pidI_Cum_Error += pid_I_Gain * pid_Error;

  if(pidI_Cum_Error > 400) {
    pidI_Cum_Error = 400;
  } else if (pidI_Cum_Error < -400) {
    pidI_Cum_Error = -400;
  }
  
  pid_Output = pid_P_Gain * pid_Error + pidI_Cum_Error + pid_D_Gain * (pid_Error - pidD_Last_Error);

  pidD_Last_Error = pid_Error;
  oldTime = currentTime;

  
  if(pid_Output < 5 && pid_Output > -5) {
    pid_Output = 0;
  }

  if(pid_Output > 400)pid_Output = 400;                                     //Limit the PID-controller to the maximum controller output: + and - 180!
  else if(pid_Output < -400)pid_Output = -400;

  
  if(pitch_Angle > 40 || pitch_Angle < -40 || batteryIsLow) {
     pid_Output = 0;
  }

  /**
   * Adapt the calculated output according to the rc-data:
   * Deadband between -4 and 4 degrees of transmitter.
   */
  pid_Output_Right = pid_Output;
  pid_Output_Left = pid_Output;

  if(rollAngleNRF >= 4) {
    pid_Output_Left += maxSpeedMotors;
    pid_Output_Right -= maxSpeedMotors;
  }

  if(rollAngleNRF < -4) {
    pid_Output_Left -= maxSpeedMotors;
    pid_Output_Right += maxSpeedMotors;
  }


  //Positive pitch angle -> Drive straight -> Lean foreward -> angle greater than 0 -> pid_Setpoint negative
  if (pitchAngleNRF >= 4) { 
    if(pid_Setpoint > -2.5) {
      pid_Setpoint -= 0.05;
    }
  }

  if(pitchAngleNRF < -4) {
    if(pid_Setpoint < 2.5) {
      pid_Setpoint += 0.05;
    }
  }

  if(pitchAngleNRF >= -4 && pitchAngleNRF < 4) {
    if(pid_Setpoint < 0) {
      pid_Setpoint += 0.05;
    } 
    if(pid_Setpoint > 0) {
      pid_Setpoint -= 0.05;
    }
  }

  
  //Allows the robot to adapt it's position to a neutral position in order to really get into a standstill.
  if(pid_Setpoint == 0){                                                    //If the setpoint is zero degrees
    if(pid_Output < 0) {
      self_balance_pid_setpoint += 0.0015;                  //Increase the self_balance_pid_setpoint if the robot is still moving forewards
    }
    if(pid_Output > 0) {
      self_balance_pid_setpoint -= 0.0015;                  //Decrease the self_balance_pid_setpoint if the robot is still moving backwards
    }
  }

  
  //Calculate the steps until which have to be taken untill the next motor pulse is allowed -> T = 1/(pid_Output * 20 * 10 ^-6) = 10 ^6 / (pid_Output * 20)  
  if(pid_Output_Right == 0) {
    throttle_Motor_Right = 0;
  } else {
    throttle_Motor_Right = 5000 / pid_Output_Right; 
  }

  if(pid_Output_Left == 0) {
    throttle_Motor_Left = 0;
  } else {
    throttle_Motor_Left = 5000 / pid_Output_Left; 
  }

  #ifdef DEBUG 
  Serial.print("PID_OUTPUT: ");
  Serial.print(pid_Output);
  Serial.print("-> Throttle_Right is therefore: ");
  Serial.print(throttle_Motor_Right);
  Serial.print("-> Throttle_Left is therefore: ");
  Serial.print(throttle_Motor_Left);
  Serial.print("->CUM Error ");
  Serial.print(pidI_Cum_Error);
  Serial.print("-> Pitch angle");
  Serial.print(pitch_Angle);
  Serial.print("Self balance");
  Serial.println(self_balance_pid_setpoint);
  #endif

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Interrupt routine  TIMER2_COMPA_vect
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER1_COMPA_vect){
  throttle_Counter_Motor_Right ++;
  if(throttle_Counter_Motor_Right > throttle_Motor_Memory_Right) {
    throttle_Counter_Motor_Right = 0;
    throttle_Motor_Memory_Right = throttle_Motor_Right;
    if(throttle_Motor_Memory_Right < 0) {                                         //Change spinning direction
      //PORTD &= ~(1 << PORTD5);
      PORTB &= ~(1 << PORTB0);
      throttle_Motor_Memory_Right *= -1;
    } else {                                                                //Change spinning direction
      //PORTD |= (1 << PORTD5);
      PORTB |= (1 << PORTB0);
    }
  } else if(throttle_Counter_Motor_Right == 1){
    PORTD |= 0b10000000;                                                     //(1 << PORTD7) | (1 << PORTD5);             
  }
  else if(throttle_Counter_Motor_Right == 2) {
    PORTD &= 0b01111111;                                                     //~(1 << PORTD7) & ~(1 << PORTD5);             
  }
  throttle_Counter_Motor_Left ++;
  if(throttle_Counter_Motor_Left > throttle_Motor_Memory_Left) {
    throttle_Counter_Motor_Left = 0;
    throttle_Motor_Memory_Left = throttle_Motor_Left;
    if(throttle_Motor_Memory_Left < 0) {                                         //Change spinning direction
      PORTD &= ~(1 << PORTD5);
      throttle_Motor_Memory_Left *= -1;
    } else {                                                                //Change spinning direction
      PORTD |= (1 << PORTD5);
    }
  } else if(throttle_Counter_Motor_Left == 1){
    PORTD |= 0b01000000;                                                     //(1 << PORTD7) | (1 << PORTD5);             
  }
  else if(throttle_Counter_Motor_Left == 2) {
    PORTD &= 0b10111111; 
  }
}
