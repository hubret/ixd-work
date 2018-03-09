//LightxLight
//based on:
//https://www.instructables.com/id/Multiplexing-with-Arduino-and-the-74HC595/

//uses 8 bit shift register and MPU9250.

#include <Wire.h>
#include <SPI.h>//this test uses SPI, only 4 wires connected to the board
#include <quaternionFilters.h>
#include <MPU9250.h>

#include <stdio.h>
#include <math.h>

#define PI 3.14159265

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling
int myLed2  = 11;

MPU9250 myIMU;

#define latchPin A1
#define clockPin A0
#define dataPin A2

//looping variables
byte i;
int j;
int n;
int t;
int c;
float a;

//storage variable
byte dataToSend;

//storage for led states, 4 bytes
byte ledData[] = {0, 0, 0, 0};

void setup() {
  //set pins as output
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);

  c = 0;
  j = 0;

  Wire.begin();
  Serial.begin(9600);

  // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);

  pinMode(myLed, OUTPUT);
  pinMode(myLed2, OUTPUT);
  
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2],1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3],1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4],1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);


    myIMU.initMPU9250();
    // Initialize device for act+ive mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);


    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");
    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
    }



  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while(1) ; // Loop forever if communication doesn't happen
  }
  
}

void loop() {

  j = (int)floor((a * (180/PI) + 180)/22.5);

  Serial.println(floor((a * (180/PI) + 180)/22.5));
  //Serial.println(a * (180/PI) + 180);

  t++;

  //framerate
  if(t%10==0){

    c++;

    switch(j){
      case 0:
        ledData[0] = 1;
        ledData[1] = 0;
        ledData[2] = 0;
        ledData[3] = 0;
        break;
      case 1:
        ledData[0] = 2;
        ledData[1] = 0;
        ledData[2] = 0;
        ledData[3] = 0;
        break;
      case 2:
        ledData[0] = 4;
        ledData[1] = 0;
        ledData[2] = 0;
        ledData[3] = 0;
        break;
      case 3:
        ledData[0] = 8;
        ledData[1] = 0;
        ledData[2] = 0;
        ledData[3] = 0;
        break;
      case 4:
        ledData[0] = 0;
        ledData[1] = 1;
        ledData[2] = 0;
        ledData[3] = 0;
        break;
      case 5:
        ledData[0] = 0;
        ledData[1] = 2;
        ledData[2] = 0;
        ledData[3] = 0;
        break;
      case 6:
        ledData[0] = 0;
        ledData[1] = 4;
        ledData[2] = 0;
        ledData[3] = 0;
        break;
      case 7:
        ledData[0] = 0;
        ledData[1] = 8;
        ledData[2] = 0;
        ledData[3] = 0;
        break;
      case 8:
        ledData[0] = 0;
        ledData[1] = 0;
        ledData[2] = 1;
        ledData[3] = 0;
        break;
      case 9:
        ledData[0] = 0;
        ledData[1] = 0;
        ledData[2] = 2;
        ledData[3] = 0;
        break;
      case 10:
        ledData[0] = 0;
        ledData[1] = 0;
        ledData[2] = 4;
        ledData[3] = 0;
        break;
      case 11:
        ledData[0] = 0;
        ledData[1] = 0;
        ledData[2] = 8;
        ledData[3] = 0;
        break;
      case 12:
        ledData[0] = 0;
        ledData[1] = 0;
        ledData[2] = 0;
        ledData[3] = 1;
        break;
      case 13:
        ledData[0] = 0;
        ledData[1] = 0;
        ledData[2] = 0;
        ledData[3] = 2;
        break;
      case 14:
        ledData[0] = 0;
        ledData[1] = 0;
        ledData[2] = 0;
        ledData[3] = 4;
        break;
      case 15:
        ledData[0] = 0;
        ledData[1] = 0;
        ledData[2] = 0;
        ledData[3] = 8;
        break;
    }

  }
  
  for (i=0;i<4;i++){
    
    //send data from ledData to each row, one at a time
    byte dataToSend = (1 << (i+4)) | (15 & ~ledData[i]);
      
    // setlatch pin low so the LEDs don't change while sending in bits
    digitalWrite(latchPin, LOW);
    // shift out the bits of dataToSend to the 74HC595
    shiftOut(dataPin, clockPin, LSBFIRST, dataToSend);
    //set latch pin high- this sends data to outputs so the LEDs will light up
    digitalWrite(latchPin, HIGH);

    delay(2);
  }  

  //breakout

  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {  
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0]*myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1]*myIMU.aRes; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2]*myIMU.aRes; // - accelBias[2];

    
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
//  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
                         myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: "); Serial.print(1000*myIMU.ax);
        Serial.print(" mg ");
        Serial.print("Y-acceleration: "); Serial.print(1000*myIMU.ay);
        Serial.print(" mg ");
        Serial.print("Z-acceleration: "); Serial.print(1000*myIMU.az);
        Serial.println(" mg ");

        

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;

      }

      myIMU.count = millis();
      digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
       //Serial.print("ax = "); Serial.print((int)1000*myIMU.ax);
         /*Serial.print(" ay = "); Serial.print((int)1000*myIMU.ay);
        Serial.print(" az = "); Serial.print((int)1000*myIMU.az);
        Serial.println(" mg");*/


        /********************** This is where we get the accelerometer x,y,z, values***********/
        //Serial.print((int)1000*myIMU.ax);Serial.print(",");Serial.println((int)1000*myIMU.ay);
        ////USE THESE TO DEFINE WHEN TO CHANGE THE LIGHTS
        
        //Serial.print("y "); Serial.println((int)1000*myIMU.ay);
        
        //Serial.print("z "); Serial.println((int)1000*myIMU.az);
        //Serial.print(",");
        /********************************* end of accelerometer numbers **********************/
      }

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
                    *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
                    *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
                    *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
                    - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //    8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw   -= 8.5;
      myIMU.roll  *= RAD_TO_DEG;

      if(SerialDebug)
      {
        /*Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(myIMU.yaw, 2);
        Serial.print(", ");
        Serial.print(myIMU.pitch, 2);
        Serial.print(", ");
        Serial.println(myIMU.roll, 2);

        Serial.print("rate = ");
        Serial.print((float)myIMU.sumCount/myIMU.sum, 2);
        Serial.println(" Hz");*/
      }

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
  /* boi
     Serial.print((int)1000*myIMU.ax);Serial.print(",");Serial.println((int)1000*myIMU.ay);
   */

    
    a = atan2(myIMU.ay,myIMU.ax);
    
     
}
