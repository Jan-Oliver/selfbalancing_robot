#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <math.h>
#define MPU_I2C 0b1101000
/***************************************************
 * IMU Variables
 ***************************************************/
//Offsets for the IMU values
const float X_ACCEL_OFFSET = 312.58;
const float Y_ACCEL_OFFSET = -178.67;
const float Z_ACCEL_OFFSET = -157.19;
const float X_GYRO_OFFSET = -933.13;
const float Y_GYRO_OFFSET = -296.18;
const float Z_GYRO_OFFSET = -225.30;

//Constants which depend on the selected sensitivity of the IMU
const float ACCEL_SENSITIVITY_CONVERSION = 16384.0;
const float GYRO_SENSITIVITY_CONVERSION = 131.0;

//The Complementary filter needs a weight for the gyro data and the accel data to melt them
const float COMPL_FILTER_GYRO_WEIGHT = 0.95;

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

/***************************************************
 * Joystick Variables
 ***************************************************/
//Joystick Variables
const int ctrlSW = 2;
const int ctrlX = A1;
const int ctrlY = A0;
const int ADC_RESOLUTION = 9;
//Joystick Variables to store the data
bool ctrlSWData = false;
int ctrlXData = 0;
int ctrlYData = 0;

//Transmitter object
RF24 radio(7, 8); // CE, CSN
//Adress for communication
const byte address[6] = "00001";

//Array of all the data which should be send
int data[5];

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  //radio.setPALevel(RF24_PA_MIN);
  radio.stopListening();
  pinMode(ctrlX,INPUT);
  pinMode(ctrlY,INPUT);
  pinMode(ctrlSW,INPUT);
  digitalWrite(ctrlSW,HIGH);
  //IMU-communication
  Wire.begin();
  setupMPU();
}

/**
 * Configure the MPU
 */
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
}
/**
 * The main loop
 * First of all we read the Joystick Data which is pretty easy -> readJoystickData()
 * Afterwards it gets a bit more complicated. We have to first read the raw accel and gyro data from the Register -> recordAccelRegisters() and recordGyroRegisters()
 * Apply the needed offsets on this raw data to center it around 0 -> applyOffsets()
 * To get usefull data we have to convert the raw data with the corresponding selected sensitivity-values. -> convertFromRawData()
 * Now have to estimate the pitch and roll angles from the acceleration data. The yaw can't be obtained because the accel doesn't change along a rotation along the z-axis -> estimateAnglesAccel()
 * With these angles and the gyro data we can estimate the real angles by fusing both estimations. The accel data is bad at compensating vibrations, the gyro is drifting. By applying a
 * complementary filter we can get best of both worlds and obtain the final estimation -> complementaryFilter()
 * 
 * Pack all this data into one array called data (what a surprise :)) and send it to the driving car.
 * data[0] -> Joystick x-data: [0-1023]
 * data[1] -> Joystick y-data: [0-1023]
 * data[2] -> Joystick button data: 0 = pressed, 1 = not pressed
 * data[3] -> Estimated Roll-angle: Can be between -180 and 180 degrees but shouldn't come too close to that.
 * data[4] -> Estimated Pitch angle: Should be between -90 and 90 degrees and no more. Better not come close to -90 or +90
 */
void loop() {
  //Joystick
  readJoystickData();
  //IMU stuff
  recordAccelRegisters();
  recordGyroRegisters();
  applyOffsets();
  convertFromRawData();
  estimateAnglesAccel();
  complementaryFilter();
  
  data[0] = ctrlXData;
  data[1] = ctrlYData;
  data[2] = ctrlSWData;
  data[3] = roll_Angle;
  data[4] = pitch_Angle;
  radio.write(&data, sizeof(data));
  Serial.println(" sent data");
}

/***************************************************
 * Joystick Code
 ***************************************************/
void readJoystickData() {
  ctrlSWData = digitalRead(ctrlSW);
  ctrlXData = analogRead(ctrlX);
  ctrlYData = analogRead(ctrlY);
}

/***************************************************
 * IMU Code
 ***************************************************/
/**
 * Read raw accel data
 */
void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accel_Raw.X = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accel_Raw.Y = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accel_Raw.Z = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
}
/**
 * Read raw Gyro data
 */
void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyro_Raw.X = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyro_Raw.Y = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyro_Raw.Z = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
}
/**
 * Add the offsets to the Raw data. The offsets can be calculated by taking a number of samples (f.ex. 2000) and adding the all up. Then divide by tha amount of taken samples. 
 * This should be zero. So you have your offsets. Just be carful in the z-axis of the Accelerometer the value should be 1g in still position.
 */
void applyOffsets() {
  //Apply the accelerometer offsets
  accel_Raw.X -= X_ACCEL_OFFSET;
  accel_Raw.Y -= Y_ACCEL_OFFSET;
  accel_Raw.Z -= Z_ACCEL_OFFSET;
  
  //Apply the gyroscope offsets
  gyro_Raw.X -= X_GYRO_OFFSET;
  gyro_Raw.Y -= Y_GYRO_OFFSET;
  gyro_Raw.Z -= Z_GYRO_OFFSET;
}

/**
 * Convert the raw data of the IMU by dividing by the sensitivity specific conversion number. Can be found in datasheet
 */
void convertFromRawData() {
  //Convert raw accel data
  accel.X = accel_Raw.X / ACCEL_SENSITIVITY_CONVERSION ;
  accel.Y = accel_Raw.Y / ACCEL_SENSITIVITY_CONVERSION ; 
  accel.Z = accel_Raw.Z / ACCEL_SENSITIVITY_CONVERSION ;

  //Convert raw gyro data
  gyro.X = gyro_Raw.X / GYRO_SENSITIVITY_CONVERSION;
  gyro.Y = gyro_Raw.Y / GYRO_SENSITIVITY_CONVERSION; 
  gyro.Z = gyro_Raw.Z / GYRO_SENSITIVITY_CONVERSION;
}

/**
 * Estimate the Euler angles from the acceleration sensor values
 */
void estimateAnglesAccel() {
  roll_Angle_Accel = atan2(accel_Raw.Y , accel_Raw.Z)/(2*M_PI)*360;
  pitch_Angle_Accel = atan2(-accel_Raw.X , sqrt((accel_Raw.Y*accel_Raw.Y)+(accel_Raw.Z*accel_Raw.Z)))/(2*M_PI)*360;
}

/**
 * Combining both estimations to reduce the noise due to vibrations (accel) and the drift (gyro)
 */
void complementaryFilter() {
  long currentTime = millis();
  double dt = (currentTime - oldTime)/1000.0;
  oldTime = currentTime;
  //Calculate the estimated gyro angles
  roll_Angle_Gyro += gyro.X * dt;
  pitch_Angle_Gyro += gyro.Y * dt;

  //Estimated final compl. filter angles
  roll_Angle = COMPL_FILTER_GYRO_WEIGHT * (roll_Angle + gyro.X * dt) + (1 - COMPL_FILTER_GYRO_WEIGHT) * roll_Angle_Accel;
  pitch_Angle = COMPL_FILTER_GYRO_WEIGHT * (pitch_Angle + gyro.Y * dt) + (1 - COMPL_FILTER_GYRO_WEIGHT) * pitch_Angle_Accel;
}

/***************************************************
 * Debugging if needed
 ***************************************************/
void printData() {
  Serial.print("Switch: ");
  Serial.print(ctrlSWData);
  Serial.print("-> X-axis: ");
  Serial.print(ctrlXData);
  Serial.print("-> Y-axis: ");
  Serial.print(ctrlYData);
  Serial.print("-> Accel X: ");
  Serial.print(accel.X);
  Serial.print("-> Accel Y: ");
  Serial.print(accel.Y);
  Serial.print("-> Accel Z: ");
  Serial.print(accel.Z);
  Serial.print("-> Gyro X: ");
  Serial.print(gyro.X);
  Serial.print("-> Gyro Y: ");
  Serial.print(gyro.Y);
  Serial.print("-> Gyro Z: ");
  Serial.print(gyro.Z);
  Serial.print("-> Roll angle accel: ");
  Serial.print(roll_Angle_Accel);
  Serial.print("-> Pitch angle accel: ");
  Serial.print(pitch_Angle_Accel);
  Serial.print("-> Roll angle gyro: ");
  Serial.print(roll_Angle_Gyro);
  Serial.print("-> Pitch angle gyro: ");
  Serial.print(pitch_Angle_Gyro);
  Serial.print("-> Roll angle compl: ");
  Serial.print(roll_Angle);
  Serial.print("-> Pitch angle compl: ");
  Serial.println(pitch_Angle);
  delay(300);
}
