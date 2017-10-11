/*****************************************************************

Hardware setup: This library supports communicating with the
LSM9DS1 over either I2C or SPI. This example demonstrates how
to use I2C. The pin-out is as follows:
LSM9DS1 --------- Arduino
SCL ---------- SCL (A5 on older 'Duinos')
SDA ---------- SDA (A4 on older 'Duinos')
VDD ------------- 3.3V
GND ------------- GND
(CSG, CSXM, SDOG, and SDOXM should all be pulled high.
Jumpers on the breakout board will do this for you.)

The LSM9DS1 has a maximum voltage of 3.6V. Make sure you power it
off the 3.3V rail! I2C pins are open-drain, so you'll be
(mostly) safe connecting the LSM9DS1's SCL and SDA pins
directly to the Arduino.

Development environment specifics:
IDE: Arduino 1.6.3
Hardware Platform: SparkFun Redboard
LSM9DS1 Breakout Version: 1.0

*****************************************************************/
// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
//#include <Servo.h>

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
LSM9DS1 imu;
//Servo PWMout;

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define imu_M    0x1E // Would be 0x1C if SDO_M is LOW
#define imu_AG  0x6B // Would be 0x6A if SDO_AG is LOW


// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading.
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.
#define PWMpin 11

//Function Declarations
int magHeading();



void setup(){
  TCCR2B = TCCR2B & 0b11111000 | 0x05;
  //sets Arduino's pin 3 and 11 to frequency 122.55Hz.
  
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(11,OUTPUT);
  //Set up output USART
  Serial.begin(57600);
  Serial.setTimeout(0);

  //Set the device's communication mode and addresses:
  imu.settings.device.commInterface = IMU_MODE_I2C;
  imu.settings.device.mAddress  = imu_M;
  imu.settings.device.agAddress = imu_AG;

  //Set up Gyro:
  imu.settings.gyro.enabled = true;  // Enable the gyro
  imu.settings.gyro.scale = 2000; // +/-245deg/s (scale can be set to either 245, 500, or 2000)
  // [sampleRate] sets the output data rate (ODR) of the gyro
  // sampleRate can be set between 1-6
  // 1 = 14.9    4 = 238
  // 2 = 59.5    5 = 476
  // 3 = 119     6 = 952
  imu.settings.gyro.sampleRate = 4; //
  // [bandwidth] can set the cutoff frequency of the gyro.
  // Allowed values: 0-3. Actual value of cutoff frequency
  // depends on the sample rate. (Datasheet section 7.12)
  imu.settings.gyro.bandwidth = 3;
  // [lowPowerEnable] turns low-power mode on or off.
  imu.settings.gyro.lowPowerEnable = false; // LP mode off
  // [HPFEnable] enables or disables the high-pass filter
  imu.settings.gyro.HPFEnable = false; // HPF disabled
  // [HPFCutoff] sets the HPF cutoff frequency (if enabled)
  // Allowable values are 0-9. Value depends on ODR.
  // (Datasheet section 7.14)

  //Set up Magnitometer
  imu.settings.mag.enabled = true; // Enable magnetometer
  // [scale] sets the full-scale range of the magnetometer
  // mag scale can be 4, 8, 12, or 16
  imu.settings.mag.scale = 12; // Set mag scale to +/-12 Gs
  // [sampleRate] sets the output data rate (ODR) of the
  // magnetometer.
  // mag data rate can be 0-7:
  // 0 = 0.625 Hz  4 = 10 Hz
  // 1 = 1.25 Hz   5 = 20 Hz
  // 2 = 2.5 Hz    6 = 40 Hz
  // 3 = 5 Hz      7 = 80 Hz
  imu.settings.mag.sampleRate = 7; // Set OD rate to 80Hz
  // [tempCompensationEnable] enables or disables
  // temperature compensation of the magnetometer.
  imu.settings.mag.tempCompensationEnable = false;
  // [XYPerformance] sets the x and y-axis performance of the
  // magnetometer to either:
  // 0 = Low power mode      2 = high performance
  // 1 = medium performance  3 = ultra-high performance
  imu.settings.mag.XYPerformance = 3; // Ultra-high perform.
  imu.settings.mag.ZPerformance  = 3; // Ultra-high perform.
  // [operatingMode] sets the operating mode of the
  // magnetometer. operatingMode can be 0-2:
  // 0 = continuous conversion
  // 1 = single-conversion
  // 2 = power down
  imu.settings.mag.operatingMode = 0; // Continuous mode
  
  
  // The above lines will only take effect AFTER calling
  // imu.begin(), which verifies communication with the IMU
  // and turns it on.
  if (!imu.begin()){
    Serial.println("Error");
    while (1);
  }
  digitalWrite(13,HIGH); //ALIVE LED is GO
}

void loop()
{
  //Variables
  char readcount = 0;
  char CommandBuffer[10] = {0,0,0,0,0,0,0,0,0,0};
  char ind;
  int temp;
  char Mode = 0;   //1=transmit, 0=no transmit, always output 1500us pwm
  int heading = 0; //From Magnetometer data
  int gyro = 0;//[3] = {0,0,0};    //Gyro data (degrees per second, -2000 to 2000)
  signed int Speed = 0; //Percent throttle, from -100 to 100
  unsigned long time0 = 0; //Will be used to store time at which 'on' is transmitted
  unsigned long time1 = 0; //Used for current time
  unsigned long time  = 0; //Used to store the current time differential
  unsigned long transint = 6500;//microseconds - 250hz transmission
  unsigned int SensorBatt = 0;
  unsigned int MotorBatt  = 0;
  
  
  
  while(1){
    //Check for sensor availability
    //digitalWrite(13,HIGH);
    
    imu.readGyro();
    //gyro[0] = gyro[1];
    //gyro[1] = gyro[2];
    //gyro[2] = (int)imu.calcGyro(imu.gz); //degrees per second
    gyro = (int)imu.calcGyro(imu.gz); //degrees per second
    //digitalWrite(13,LOW);
      
    if(imu.magAvailable()){
      imu.readMag();
      heading = MagHeading(imu.mx, imu.my);
    }

    //Transmitting Data and timing  
    if(Mode==1){ 
      //Test if it has been long enough
      if((micros() - time1) > transint){ //If time since last trans. is below desired threshold
        digitalWrite(12,HIGH);
        
        //Send the timestamp
        time1 = micros(); //Save the current transmission time
        if(time1<time0)
          time = time1 + 4294967295-time0;
        else
          time = time1 - time0;
          
        //Find battery values
        SensorBatt = analogRead(A0)*200;//hundredths of volts (Divider = 
        MotorBatt  = analogRead(A1)*376; ;//hundredths of volts (accounts for voltage divider of 499/1324)
        
        Serial.print(String(time,HEX));
        Serial.print(',');
        //Send gyroscope data
        //Serial.print((gyro[0]+gyro[1]+gyro[2])/3,DEC);
        Serial.print(gyro,DEC);
        Serial.print(',');
        //Send Mag data
        Serial.print(heading,DEC);
        Serial.print(',');
        //Send Sensor Battery
        Serial.print(SensorBatt,HEX);
        Serial.print(',');
        //Send Motor Battery
        Serial.print(MotorBatt,HEX);
        Serial.print('\n');
        digitalWrite(12,LOW);
      }
    }
    
    

    //Read serial data: either throttle or configuration settings
    if (Serial.available()){ // If data comes in from serial monitor, send it out to XBee
      //Serial.readString().toCharArray(CommandBuffer,10,0);
      Serial.readBytes(CommandBuffer,10);
      
      
      if(CommandBuffer[0]==('o') && CommandBuffer[1]==('n')){
        time0 = micros();
        time1 = time0;
        Mode = 1;
      }

      if(CommandBuffer[0]==('o')&&CommandBuffer[1]==('f')&&CommandBuffer[2]==('f')){
        Mode = 0;
      }

      if(CommandBuffer[0]==('T')){
        //Find final number
        ind = 0;
        while(CommandBuffer[ind+1]!=0 && ind < 10){
          ind++;}
        
        Speed = 0;
        for(char i = ind;i>1;i--){
          temp = (int)CommandBuffer[i]-0x30;
          for(char j=1;j<(ind-i+1);j++)
            temp = temp*10;
            
          Speed = Speed + temp;
        }
        if(CommandBuffer[1] == '-')
          Speed *= -1;
          
        //Update pwm signal
        analogWrite(PWMpin,94+(float)(Speed)*0.1256);
        
      }
      CommandBuffer[0] = '/n';
    }
    
    
    }
}


// int MagHeading(float mx, float my)
// {
// 
//  float heading;
//  int result;
//  
//  if (my == 0){
//    heading = ((mx < 0) ? 180.0 : 0)*180/PI;
//  }else{
//    heading = atan2(mx, my)*180/PI;
//  }
//  
//  result = (int)(heading - DECLINATION) ;
//  
//  while (result > 360) result -= 360;
//  while (result < 0) result += 360;
// 
//  return result;
// }

int MagHeading(float mx, float my)
{
  float heading;
  if (my == 0)
  heading = (mx < 0) ? 180.0 : 0;
  else
  heading = atan2(mx, my);

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);
  else if (heading < 0) heading += 2 * PI;

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;

  return heading;
}

