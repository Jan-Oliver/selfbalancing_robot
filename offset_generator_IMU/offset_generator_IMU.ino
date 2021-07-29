#include <Wire.h>
#define MPU_I2C 0b1101000

struct sensorVal {
  float X;
  float Y;
  float Z;
}; 

sensorVal gyro;
sensorVal accel;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  setupMPU();
  delay(20);
  calibrateMPU();
}

void setupMPU() {
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}

void calibrateMPU() {
  double X_ACCEL_ERROR = 0;
  double Y_ACCEL_ERROR = 0;
  double Z_ACCEL_ERROR = 0;
  double X_GYRO_ERROR = 0;
  double Y_GYRO_ERROR = 0;
  double Z_GYRO_ERROR = 0;

  for(int i = 0; i < 20000; i++) {
    //Read the Acceleration values
    recordAccelRegisters();
    recordGyroRegisters();
    //Calculate the errors each time
    X_ACCEL_ERROR += accel.X;
    Y_ACCEL_ERROR += accel.Y;
    Z_ACCEL_ERROR += accel.Z - 16384.0; //Z-Sensor misst ja immer 1 am Anfang
    X_GYRO_ERROR += gyro.X;
    Y_GYRO_ERROR += gyro.Y;
    Z_GYRO_ERROR += gyro.Z;
    if(i % 1000 == 0) {
      Serial.print(". ");
    }
  }
  double X_ACCEL_OFFSET = X_ACCEL_ERROR/20000.0;
  double Y_ACCEL_OFFSET = Y_ACCEL_ERROR/20000.0;
  double Z_ACCEL_OFFSET = Z_ACCEL_ERROR/20000.0;
  double X_GYRO_OFFSET = X_GYRO_ERROR /20000.0;
  double Y_GYRO_OFFSET = Y_GYRO_ERROR /20000.0;
  double Z_GYRO_OFFSET = Z_GYRO_ERROR /20000.0;
  Serial.println("Calibration done:");
  Serial.println("Offsets: ");
  Serial.print("const float X_ACCEL_OFFSET = ");
  Serial.print(X_ACCEL_OFFSET);
  Serial.println(";");
  Serial.print("const float Y_ACCEL_OFFSET = ");
  Serial.print(Y_ACCEL_OFFSET);
  Serial.println(";");
  Serial.print("const float Z_ACCEL_OFFSET = ");
  Serial.print(Z_ACCEL_OFFSET);
  Serial.println(";");
  Serial.print("const float X_GYRO_OFFSET = ");
  Serial.print(X_GYRO_OFFSET);
  Serial.println(";");
  Serial.print("const float Y_GYRO_OFFSET = ");
  Serial.print(Y_GYRO_OFFSET);
  Serial.println(";");
  Serial.print("const float Z_GYRO_OFFSET = ");
  Serial.print(Z_GYRO_OFFSET);
  Serial.println(";");
}

void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accel.X = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accel.Y = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accel.Z = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyro.X = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyro.Y = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyro.Z = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
}

void loop() {

}
