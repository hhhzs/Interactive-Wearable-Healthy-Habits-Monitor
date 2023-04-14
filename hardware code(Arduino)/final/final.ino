#include <Adafruit_MPR121.h>
#include <Wire.h> //library allows communication with I2C / TWI devices
#include <math.h> //library includes mathematical functions
#include <MPU6050_light.h>
#include <Wire.h>
#ifndef _BV
#define _BV(bit) (1 << (bit)) 
#endif
Adafruit_MPR121 cap = Adafruit_MPR121();
MPU6050 mpu(Wire);
int yellow = 2, blue = 10, green = 5, red = 7;+
int mf = A0, lf = A2, mm = A1, hl = A3;
const int MPU=0x68; //I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //16-bit integers
int AcXcal,AcYcal,AcZcal,GyXcal,GyYcal,GyZcal,tcal; //calibration variables
double t,tx,tf,pitch,roll;
uint16_t currtouched = 0;
int tp = 10; //change this if you aren't using them all, start at pin 0


//function to convert accelerometer values into pitch and roll
void getAngle(int Ax,int Ay,int Az) 
{
    double x = Ax;
    double y = Ay;
    double z = Az;

    pitch = atan(x/sqrt((y*y) + (z*z))); //pitch calculation
    roll = atan(y/sqrt((x*x) + (z*z))); //roll calculation

    //converting radians into degrees
    pitch = pitch * (180.0/3.14);
    roll = roll * (180.0/3.14) ;
}
#include <SparkFun_Bio_Sensor_Hub_Library.h>
#include <Wire.h>

// Reset pin, MFIO pin
int resPin = 4;
int mfioPin = 5;

// Takes address, reset pin, and MFIO pin.
SparkFun_Bio_Sensor_Hub bioHub(resPin, mfioPin); 

bioData body;  
// ^^^^^^^^^
// What's this!? This is a type (like int, byte, long) unique to the SparkFun
// Pulse Oximeter and Heart Rate Monitor. Unlike those other types it holds
// specific information on your heartrate and blood oxygen levels. BioData is
// actually a specific kind of type, known as a "struct". 
// You can choose another variable name other than "body", like "blood", or
// "readings", but I chose "body". Using this "body" varible in the 
// following way gives us access to the following data: 
// body.heartrate  - Heartrate
// body.confidence - Confidence in the heartrate value
// body.oxygen     - Blood oxygen level
// body.status     - Has a finger been sensed?


void setup(){
  Wire.begin(); //initiate wire library and I2C
    Wire.beginTransmission(MPU); //begin transmission to I2C slave device
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0); // set to zero (wakes up the MPU-6050)  
    Wire.endTransmission(true); //ends transmission to I2C slave device
    //Serial.begin(9600); //serial communication at 9600 bauds
    if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
    }
    Serial.println("MPR121 found!");

  Serial.begin(115200);

  Wire.begin();
  int result = bioHub.begin();
  if (result == 0) // Zero errors!
    Serial.println("Sensor started!");
  else
    Serial.println("Could not communicate with the sensor!");
 
  Serial.println("Configuring Sensor...."); 
  int error = bioHub.configBpm(MODE_ONE); // Configuring just the BPM settings. 
  if(error == 0){ // Zero errors!
    Serial.println("Sensor configured.");
  }
  else {
    Serial.println("Error configuring sensor.");
    Serial.print("Error: "); 
    Serial.println(error); 
  }

  // Data lags a bit behind the sensor, if you're finger is on the sensor when
  // it's being configured this delay will give some time for the data to catch
  // up. 
  Serial.println("Loading up the buffer with data....");
  delay(4000); 
  
}

void loop(){

    // Information from the readBpm function will be saved to our "body"
    // variable.  
    checkAllPins(tp);

    //make a new line to separate the message
    Serial.println();
    body = bioHub.readBpm();
    Serial.print("Heartrate: ");
    Serial.println(body.heartRate); 
    Serial.print("Confidence: ");
    Serial.println(body.confidence); 
    Serial.print("Oxygen: ");
    Serial.println(body.oxygen); 
    Serial.print("Status: ");
    Serial.println(body.status); 
    // Slow it down or your heart rate will go up trying to keep up
    // with the flow of numbers
    delay(250); 
    Wire.beginTransmission(MPU); //begin transmission to I2C slave device
    Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false); //restarts transmission to I2C slave device
    Wire.requestFrom(MPU,14,true); //request 14 registers in total  

    //Acceleration data correction
    AcXcal = -950;
    AcYcal = -300;
    AcZcal = 0;

    //Temperature correction
    tcal = -1600;

    //Gyro correction
    GyXcal = 480;
    GyYcal = 170;
    GyZcal = 210;


    //read accelerometer data
    AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) 0x3C (ACCEL_XOUT_L)  
    AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) 0x3E (ACCEL_YOUT_L) 
    AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) 0x40 (ACCEL_ZOUT_L)
  
    //read temperature data 
    Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) 0x42 (TEMP_OUT_L) 
  
    //read gyroscope data
    GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) 0x44 (GYRO_XOUT_L)
    GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) 0x46 (GYRO_YOUT_L)
    GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) 0x48 (GYRO_ZOUT_L) 

    //temperature calculation
    tx = Tmp + tcal;
    t = tx/340 + 36.53; //equation for temperature in degrees C from datasheet
    tf = (t * 9/5) + 32; //fahrenheit

    //get pitch/roll
    getAngle(AcX,AcY,AcZ);
 
    //Serial.print("Accelerometer: ");
    //Serial.print("X = "); 
    Serial.print(AcX + AcXcal);
    Serial.print(" ");
    //Serial.print(" Y = "); 
    Serial.print(AcY + AcYcal);
    Serial.print(" ");
    //Serial.print(" Z = "); 
    Serial.print(AcZ + AcZcal); 
    Serial.print(" ");
    //Serial.print(" Z = "); 
    Serial.print(t); 
    Serial.print(" ");
    Serial.print(roll);
    Serial.print(" ");
    Serial.println(pitch);
    Serial.print(" ");
    delay(1000);
}

void checkAllPins(int totalPins)
{
  // Get the currently touched pads
  currtouched = cap.touched();
  
  for (uint8_t i=0; i<totalPins; i++) 
  {
    // it if *is* touched set 1 if no set 0
    if ((currtouched & _BV(i)))
    {
      Serial.print(1); 
    }
    else
    {
      Serial.print(0);
    }

  ///adds a comma after every value but the last one
  if(i<totalPins-1)
  {
    Serial.print(",");
  }
}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
