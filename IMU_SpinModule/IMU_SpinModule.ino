// The SFE_LSM9DS1 library requires both Wire and SPI be
// included BEFORE including the 9DS1 library.
#include <Wire.h>
#include <SPI.h>
#include "SparkFunLSM9DS1.h"
#include <variant.h>
//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
LSM9DS1 imu;

///////////////////////
// LSM9DS1 I2C Setup //
///////////////////////

// SDO_XM and SDO_G are both pulled high, so our addresses are:
#define imu_M        0x1E // Would be 0x1C if SDO_M is LOW
#define imu_AG       0x6B // Would be 0x6A if SDO_AG is LOW 
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.(Earth's magnetic field varies by location. )

// Pin Definitions
#define Vbatt_in      A0    //Input pin for battery voltage / 4
#define ESCON_Current A1 // Analog out 1, from ESCON module 0V->0A, 3.3V->2.33A
#define ESCON_Speed   A2 // Analog out 2, from ESCON module 0V->-6200rpm, 3.3V->6200rpm

#define ESCON_PWM     10 // Din #1  10%->-2.3A, 90%->2.3A
#define ESCON_Enable  11 // Din #2
#define ESCON_Ready   12 // Dio #3
#define ESCON_DIO4    13 // Dio #4

#define Alive_LED	    2	// Pin for Alive LED
#define Comm_LED	    3	// Pin for communication confirmation LED
#define Motor_LED	    4	// Pin to confirm throttle commands are being sent to motor.
#define VBatt_LED	    5	// Pin to confirm throttle commands are being sent to motor.
#define XTRA          6 // DUE 6 is the XTRA Led
#define U3V_En		    7	// DUE 7 is the 2nd fire LED this if for the yoyo despinner
#define WakeUpLine	  8	// DUE 8 is going to be used as the DIO line to wake the DUE up from sleep
#define FIRE_0        9 // DUE 9 is the fire LED this is for the yoyo despinner 


//Function Declarations
void Configure_IMU();
void PrintData();


void setup(){
  //analogWriteResolution(12); // Use 12-bit pwm resolution on pwm-out pins THIS DOES NOTHING FOR DUE UNLESS VARIANT.H FILE IS CHANGED WHEN COMPILING FOR RELEASE
  analogReadResolution(12);  // 12-bit ADC used to full capacity

	  //sets Arduino's pin 3 and 11 to frequency 122.55Hz.
	  pinMode(Vbatt_in,INPUT);
	  pinMode(ESCON_Current,INPUT);
	  pinMode(ESCON_Speed,INPUT);
	  pinMode(ESCON_Ready,INPUT);
	  pinMode(ESCON_DIO4,INPUT);
	  pinMode(WakeUpLine,INPUT);

	// Output pins
	pinMode(ESCON_PWM,OUTPUT);
	pinMode(ESCON_Enable,OUTPUT);
	//analogWrite(ESCON_PWM,2047); // 50% duty cycle output to ESCON so that it will not move!
	analogWrite(ESCON_PWM,127);
	digitalWrite(ESCON_Enable,LOW);//Not Enabled
	
	//set all LEDs and Fire pins to outputs
	pinMode(Alive_LED,OUTPUT);
	pinMode(Comm_LED ,OUTPUT);
	pinMode(Motor_LED,OUTPUT);
	pinMode(VBatt_LED,OUTPUT);
	pinMode(FIRE_0,OUTPUT);
  pinMode(XTRA,OUTPUT);
	pinMode(U3V_En,OUTPUT);

	//write low to all LEDs and Fire pins
	digitalWrite(Alive_LED,LOW);
	digitalWrite(Comm_LED ,LOW);
	digitalWrite(Motor_LED,LOW);
	digitalWrite(VBatt_LED,LOW);
  digitalWrite(XTRA,LOW);
  
	digitalWrite(FIRE_0,HIGH);
	digitalWrite(U3V_En,HIGH);

  //Set up output USART
  Serial1.begin(57600);
  Serial1.setTimeout(1);
  Serial.begin(57600);
  Serial.setTimeout(2);
  Serial.println("Beginning Setup of Instruments...");

  digitalWrite(Alive_LED,HIGH);
  //IMU startup time?
  delay(1500);
  digitalWrite(Alive_LED,LOW);

  //Set up LSM9DS1 IMU for operation
  Configure_IMU();
  
  
  // Startup IMU and write desired settings to the IC.
  if (!imu.begin()){
    while (1){
		Serial.println("Error in Initialization of LSM9DS1 IMU");
		Serial1.println("ERROR");
		delay(1000);
	}
  }
  Serial.println("Spin Module is Go:");
  Serial1.println("GREEN");
  
  digitalWrite(Motor_LED,HIGH);
  delay(1000);
  digitalWrite(Comm_LED,HIGH);
  delay(1000);
  digitalWrite(Alive_LED,HIGH);
  delay(1000);
  digitalWrite(VBatt_LED,HIGH);
  delay(1000);
  digitalWrite(XTRA,HIGH);
  delay(1000);
  digitalWrite(Motor_LED,LOW);
  digitalWrite(Comm_LED,LOW);
  digitalWrite(Alive_LED,LOW);
  digitalWrite(VBatt_LED,LOW);
  digitalWrite(XTRA,LOW);
  delay(500);
  
  //Alive is go for launch
  digitalWrite(Alive_LED,HIGH); //ALIVE LED is GO
}


void Configure_IMU(){
	//Set the device's communication mode and addresses:
	imu.settings.device.commInterface = IMU_MODE_I2C;
	imu.settings.device.mAddress  = imu_M;
	imu.settings.device.agAddress = imu_AG;

	//Set up Gyro:
	imu.settings.gyro.enabled = true;  // Enable the gyro
	imu.settings.gyro.scale = 500; // +/-245deg/s (scale can be set to either 245, 500, or 2000)
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
	imu.settings.mag.enabled = false; // Disable magnetometer
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
}



void loop()
{
  //Variables
  //////////////////////////////////////////////////////////////
  //////////////////Transmission Packet setup///////////////////
  //////////////////////////////////////////////////////////////
  char  TXBuffer[14] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0};  //Transmit Buffer: "D"(1 byte), TransmitTime (4 bytes), gyrotrans (2 bytes), motor_rpm (2 bytes), motor_current (2 bytes), battery (1 byte), torque_echo (1 byte), "\n" (1 byte) = 14 total bytes.
  unsigned long TransmitTime         = 0; //Used for current time
  int gyrotrans = 0;  //gyro transmission value
  int motor_rpm = 0;  // motor speed transmission value
  int motor_current = 0;  // motor current transmission value
  char battery = 0;   // battery voltage transmission value
  char torque_echo = 0; // echo of the torque from the base (used to gauge packet loss/quality)
  TXBuffer[0] = 'D';
  TXBuffer[13] = 10;  // newline character
  //////////////////////////////////////////////////////////////
  //////////////////Receive Packet setup///////////////////
    //////////////////////////////////////////////////////////////
  char RXBuffer[5] = {0,0,0,0,0}; //Receive Buffer: "E" (1 byte), torque_cmd (1 byte), motor_enable_cmd (1 byte), fire_cmd (1 byte), "\n" (1 byte) = 5 total bytes.
  char torque_cmd = 0;      //Commanded torque from base
  char motor_enable_cmd = 0; // motor enable from base
  char fire_cmd = 0;  // yo-yo despinner command from base
  //////////////////////////////////////////////////////////////
  
//   char  ind; //Used in command processing
//   int   temp; //^

  
  char  Mode  = 0;   //1=transmit, 0=no transmit, always output 50% pwm
  float gyro  = 0; //Gyro data (degrees per second, -500 to 500)
  char  Speed = 0; //Percent throttle, from -100 to 100
  unsigned char Speed_PWM = 127;
  //byte Speed = 0;

  unsigned long transint             = 1000000/100;//microseconds - 100hz transmission
  unsigned long LastMotorCommandTime = 0; //Used to store the current time differential
  unsigned long MotorTimeout         = 2000000;//2 seconds for command timeout
  
  
  while(1)
  {
	//Get gyro data,
    imu.readGyro();
    gyro = imu.calcGyro(imu.gz); //raw->degrees per second
    TransmitTime = millis(); // use millis instead of micros for the xbee time because we don't need the timing resolution.

  //////////////////////////////////////////////////////////////
    //////////////////Things that run every loop/////////////////
  //////////////////////////////////////////////////////////////

///Build up the TXBuffer///

  //////////////////////////////////////////////////////////////
  // Time //////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////
  
  TXBuffer[1] = 51;//TransmitTime >> 24;
  TXBuffer[2] = 33 + (TransmitTime >> 16); // the first several several thousand milliseconds the (timer >> 16) will be very low (in the NULL to !) region of ASCII which are interpretted funny so adding a constant to get to a standard symbol
  TXBuffer[3] = TransmitTime >> 8;
  TXBuffer[4] = TransmitTime;

  //////////////////////////////////////////////////////////////
  // gyro //////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////
  
  TXBuffer[5] = 52;
  TXBuffer[6] = 53;

  //////////////////////////////////////////////////////////////
  // motor_rpm /////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////
    
  TXBuffer[7] = 54;
  TXBuffer[8] = 55;

  //////////////////////////////////////////////////////////////
  // motor_current /////////////////////////////////////////////
  //////////////////////////////////////////////////////////////
  
  TXBuffer[9] = 56;
  TXBuffer[10] = 57;
  //////////////////////////////////////////////////////////////
  // battery ///////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////

  TXBuffer[11] = 58;

  //////////////////////////////////////////////////////////////
  // torque doesn't need tended to this is handled in the RX routine
  //////////////////////////////////////////////////////////////
  
///////////////////////////////// COMMENTED OUT FOR TESTING ECHO //////////////////////////////////////////////////////////

//    if(Mode==0) //Stop motor do not transmit data
//    {
//      analogWrite(ESCON_PWM,127);
//      // Set led to indicate Motor Shutoff
//      digitalWrite(ESCON_Enable,LOW);
//      digitalWrite(Motor_LED,LOW);      
//    }
//
//    //Transmitting Data and timing  
//    if(Mode==1) //Mode 1 only transmit data(gyro, battery, etc.)
//    { //Transmit data
//		//Test if it has been long enough
//  		if((micros() - TransmitTime) > transint)
//    		{ //If time since last trans. is above desired threshold, send data to MyRIO
//    			//Find battery values
//    			float VBatt   = 17.49*analogRead(Vbatt_in)/4096.;//(accounts for voltage divider of 1/5.3 and max ADC in of 3.3V)
//    			float Current = float(analogRead(ESCON_Current)-2048.)*2.33/2048.; //Amps
//    			int MotorRPM  = (analogRead(ESCON_Speed)-2048.)*6200./2048.; //rpm
//    			
//    			TransmitTime = micros();
//    			PrintData(gyro, VBatt, Current, MotorRPM);
//    		}
//    }
//    if(Mode==2) // Mode 2 do not transmit only torque the motor.  When would we ever need this aside from testing?
//    {
//        digitalWrite(Motor_LED,HIGH);
//        digitalWrite(ESCON_Enable,HIGH);
//      //Torque command
//      if(CommandBuffer[0]==('T'))
//      {
//        LastMotorCommandTime = micros();
//             
//        Speed = CommandBuffer[1];// Comes in as signed integer!
//        //Update pwm signal
//        //analogWrite(ESCON_PWM,2047 + float(Speed)*16.384);//T+100 results in 90%,T-100 -> 10% PWM
//        Speed_PWM = Speed + 127;
//
//
//      }
//      analogWrite(ESCON_PWM,Speed_PWM);//moved this out of the if statement in case the IF doesn't execture
//      
//    }
//    if(Mode==3) // Mode 3 transmit data and torque the motor
//    {
//      digitalWrite(Motor_LED,HIGH);
//      digitalWrite(ESCON_Enable,HIGH);
//        
//      if((micros() - TransmitTime) > transint)
//        { //If time since last trans. is above desired threshold, send data to MyRIO
//        //Find battery values
//        float VBatt   = 17.49*analogRead(Vbatt_in)/4096.;//(accounts for voltage divider of 1/5.3 and max ADC in of 3.3V)
//        float Current = float(analogRead(ESCON_Current)-2048.)*2.33/2048.; //Amps
//        int MotorRPM  = (analogRead(ESCON_Speed)-2048.)*6200./2048.; //rpm
//        
//        TransmitTime = micros();
//        PrintData(gyro, VBatt, Current, MotorRPM);
//        //Torque command
//        if(CommandBuffer[0]==('T'))
//          {
//            LastMotorCommandTime = micros();
//                 
//            Speed = CommandBuffer[1];// Comes in as signed integer!
//            //Update pwm signal
//            //analogWrite(ESCON_PWM,2047 + float(Speed)*16.384);//T+100 results in 90%,T-100 -> 10% PWM
//            Speed_PWM = Speed + 127;
//
// 
//          }
//                      analogWrite(ESCON_PWM,Speed_PWM);//moved this out of the if statement in case the IF doesn't execture
//        }
//    }
    
    ///////////////////////////////// COMMENTED OUT FOR TESTING ECHO //////////////////////////////////////////////////////////

	//Check for motor timeout
/*	bobby commented this out for debugging 9/29
 *   if ( (micros() - LastMotorCommandTime)>MotorTimeout)
	{
		//analogWrite(ESCON_PWM,2047);//Shut off PWM commanded torque
		analogWrite(ESCON_PWM,127);
		// Set led to indicate Motor Shutoff
		digitalWrite(ESCON_Enable,LOW);
		digitalWrite(Motor_LED,LOW);

    }
*/
//  delay(1);// Probably don't need this delay any longer.

  /////Seems like checking serial data should be in an ISR?
    //Read serial data: either throttle or configuration settings
    if (Serial1.available())
      {       
  		  Serial1.readBytesUntil(10,RXBuffer,5); //Read until LF char (10) into commandBuffer, max=8
         if (RXBuffer[0]=='t')
         {
          //Serial.print(CommandBuffer);
          TXBuffer[12] = RXBuffer[1];
          Serial1.write(TXBuffer,14);  // Send the transmission buffer. // Without the length parameter sometimes garbage gets sent either before or after the desired packet.
         }
//      RXBuffer[0] = 10; //set first byte to a line feed (NL/LF - '/n') not sure why this is here?
      Serial1.flush();
  	  
///////////////////////////////// COMMENTED OUT FOR TESTING ECHO //////////////////////////////////////////////////////////

 /*     
//  		if(CommandBuffer[0]==('o') && CommandBuffer[1]==('n'))
//  		{ //10 is LF character
//  			Mode = 1;
//  			digitalWrite(Comm_LED,HIGH);
//  		}
//  
//  		//End motor communications
//  		if(CommandBuffer[0]==('o') && CommandBuffer[1]==('f') && CommandBuffer[2]==('f'))
//  		{
//  			Mode = 0; //Stop Transmitting
//  			digitalWrite(Comm_LED,LOW);
//  		}
//  
//  		//Fire (release the magnets) command
//  		if (CommandBuffer[0]==('f') && CommandBuffer[1]==('i') && CommandBuffer[2]==('r') && CommandBuffer[3]==('e'))
//  		{
//  
//  			digitalWrite(FIRE_0,LOW);
//  			digitalWrite(XTRA,HIGH);
//  			delay(1000);
//  			digitalWrite(FIRE_0,HIGH);
//  			digitalWrite(XTRA,LOW);
//  
//  		}
//  
//  		//Torque command
//  		if(CommandBuffer[0]==('T')){
//  			//digitalWrite(Motor_LED,HIGH);
//  			//digitalWrite(ESCON_Enable,HIGH);
//  			//LastMotorCommandTime = micros();
//               
//  			Speed = CommandBuffer[1];// Comes in as signed integer!
//  			//Update pwm signal
//  			//analogWrite(ESCON_PWM,2047 + float(Speed)*16.384);//T+100 results in 90%,T-100 -> 10% PWM
//       
//  			Speed_PWM = Speed + 127;
//  			//analogWrite(ESCON_PWM,Speed_PWM);
//  			Serial.println("T");
//  			Serial.write(lowByte(Speed));
//        Serial.print(lowByte(Speed));
//  		}
//  
//     // Turn motor on
//     if (CommandBuffer[0]==('m') && CommandBuffer[1]==('o') && CommandBuffer[2]==('e') && CommandBuffer[3]==('n'))
//     {
//        digitalWrite(Motor_LED,HIGH);
//        digitalWrite(ESCON_Enable,HIGH);
//        if (Mode == 1) // already transmitting
//          {
//            Mode = 3;
//          }
//        else if(Mode == 0) // not transmitting so don't start
//          {
//            Mode = 2;
//          }
//      }
//  
//      // Turn motor off
//     if (CommandBuffer[0]==('m') && CommandBuffer[1]==('o') && CommandBuffer[2]==('d') && CommandBuffer[3]==('s'))
//     {
//        digitalWrite(Motor_LED,LOW);
//        digitalWrite(ESCON_Enable,LOW);
//        analogWrite(ESCON_PWM,127);
//        
//        if (Mode == 3)
//          {
//            Mode = 1;
//          }
//        else if(Mode == 2)
//          {
//            Mode = 0;
//          }
//      }
//  
//      //Idle command
//      if(CommandBuffer[0]==('i') && CommandBuffer[1]==('d') && CommandBuffer[2]==('l') && CommandBuffer[3]==('e'))
//      {
//        // Drive the U3V EN pin into the LOW power state
//        digitalWrite(U3V_En,LOW);
//      }
//  
//      //Wake up command
//      if(CommandBuffer[0]==('w') && CommandBuffer[1]==('a') && CommandBuffer[2]==('k') && CommandBuffer[3]==('e'))
//      {
//        // Drive the U3V EN pin back into a high power state
//        digitalWrite(U3V_En,HIGH);
//      }

*/
///////////////////////////////// COMMENTED OUT FOR TESTING ECHO //////////////////////////////////////////////////////////
    /*
     if(CommandBuffer[0]==('e') && CommandBuffer[1]==('c') && CommandBuffer[2]==('h') && CommandBuffer[3]==('o'))
     {
      digitalWrite(Comm_LED,HIGH);
          float VBatt   = 17.49*analogRead(Vbatt_in)/4096.;//(accounts for voltage divider of 1/5.3 and max ADC in of 3.3V)
          float Current = float(analogRead(ESCON_Current)-2048.)*2.33/2048.; //Amps
          int MotorRPM  = (analogRead(ESCON_Speed)-2048.)*6200./2048.; //rpm
          
          TransmitTime = micros();
          //PrintData(gyro, VBatt, Current, CommandBuffer[5]); // For speed testing the last value of the XBEE packet is whatever what sent in CommandBuffer[4]
        PrintData(gyro, VBatt, CommandBuffer[4], CommandBuffer[5]);
      digitalWrite(Comm_LED,LOW);
     }
     */
      

    }
 }
}

//void PrintData(float gyro,float VBatt,float Current,int RPM) // LG's old function setup. 
// To send: time, gyro (2 bytes), motor_rpm, motor_current, battery*10 (char), echo of torque command
// to receive: torque (8 bits), fire (bit), motor on/off (bit)
/*
void PrintData(float gyro,float VBatt,char Current,char RPM)  //Bobby changed for development
{
	int gyrotrans  =  int(gyro   *100);
	Serial.println(gyrotrans);
	char ActualAmp = char(Current*50);

	//Send gyroscope data
	Serial1.print("D");
	Serial1.write(lowByte(gyrotrans));
	Serial1.write(highByte(gyrotrans));
  Serial.print(lowByte(gyrotrans));
	
	//Send Motor Battery
	//if (VBatt > 14.8)
	//	RPM = RPM + 32768;//Put this logic into highest bit of RPM
	

	//Send Speed Averaged
	//Serial1.write(lowByte(RPM));
	//Serial1.write(highByte(RPM));
  Serial1.write(Current);
  Serial1.write(RPM);

	//Send Current Averaged
	Serial1.write(ActualAmp);
	Serial1.print('\n');

}
*/

//Old Torque Command Interpretation
// Speed = 0; //Reset desired torque

// for(char i = ind-1;i>1;i--){
// 	temp = (int)CommandBuffer[i]-0x30;
// 	CommandBuffer[i] = 0;
// 
// 	for(char j=1;j<(ind-i);j++)
// 	temp = temp*10;
// 	
// 	Speed = Speed + temp;
// }
// if(CommandBuffer[1] == '-')
// Speed *= -1;
