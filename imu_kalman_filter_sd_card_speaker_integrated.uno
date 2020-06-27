/*
*      File Name: imu_kalman_filter_sd_card_speaker_integrated.uno
*      Date: 21/01/2016 
*
*      Author: Dhanunjaya Singh
*              Varun Tandon
*
*      Email Id: dhanunjay.201889@gmail.com
*
*      Description:
*
*       1. IMU sensor Integrated with Arduino
*       2. IMU Sensor data enhancement using kalman Filter
*       3. SD Card Integrated with Arduino
*       4. Speaker Inerfaced with Arduino for playing Audio Files Stored in SD Card
*
*       The author's acknowldege the contribution of the library developers included in this software for this software development.
*
*       The Code below is granted free of charge, to any person obtaining a copy of this software 
*       and associated documentation files (the "Software"), to deal in the Software without restriction, 
*       including without limitation on the rights to use, copy, modify, merge, publish and distribute.   
*
*/

#include <Wire.h>
#include "Kalman.h" 
#include <FreeSixIMU.h>

/*SD card Integration for storing audio output files*/
#include <SdFat.h>
#define SD_ChipSelectPin 10  

/*Audio Speaker Integration*/
#include <TMRpcm.h>

/*Create an object for SD and Audio TMR*/
SdFat SD;
TMRpcm tmrpcm;

/*For DigitalWrite*/
int digitalwritepin=4;

float angles[3]; // yaw pitch roll
FreeSixIMU sixDOF = FreeSixIMU();
Kalman kalmanX;
Kalman kalmanY;

/*for cube face detection*/
static byte levelselect=1;
static int previous_face=0;
static int mod_count=1;

/*For alphabets*/
static int alphabets=0;
static int numbers=0;
static int days=0;
static int weeks=0;
static int months=0;
static int multiple=0;
//static byte screenincr=0;

/*Initial IMU parameters*/
#define gyroAddress 0x68
#define adxlAddress 0x53

/*Optimized values identified through experimentation*/
double zeroValue[5] = { -200, 44, 660, 52.3, -18.5};

/*All the angles start at 180 degrees*/
double gyroXangle = 180;
double gyroYangle = 180;
double compAngleX = 180;
double compAngleY = 180;

/*Cube Face detection parameters*/
static int face;
static int faceprev=0;

unsigned long timer;

/*I2C buffer*/
uint8_t buff[2];

void setup()
{

        /*Initialize Serial Communication*/
	Serial.begin(4800);
	Wire.begin();
        
	/*Set Speaker Pin*/
	tmrpcm.speakerPin = 9; 

	pinMode(digitalwritepin, OUTPUT);

	Serial.println("Serial channels Initialized");
        
	/*Initializing IMU*/
	sixDOF.init(); 
	delay(5);
	Serial.println("IMU Initialized");
        
	/*Full resolution Mode*/
	i2cWrite(adxlAddress, 0x31, 0x09); 

	/*Setup ADXL345 for constant measurement mode*/
	i2cWrite(adxlAddress, 0x2D, 0x08); 

	i2cWrite(gyroAddress, 0x16, 0x1A); // this puts your gyro at +-2000deg/sec  and 98Hz Low pass filter
	i2cWrite(gyroAddress, 0x15, 0x09); // this sets your gyro at 100Hz sample rate
        
	/*Set Starting angle*/
	kalmanX.setAngle(180); 
	kalmanY.setAngle(180);
	timer = micros(); 


	if (!SD.begin(SD_ChipSelectPin,SPI_FULL_SPEED)) {  // see if the card is present and can      be initialized:
		Serial.println("SD fail");  
		return;   // don't do anything more if not
	}          

	Serial.println("!!! All Modules Initialized Successfully !!!"); 
}

void loop()
{

	double gyroXrate = -(((double)readGyroX() - zeroValue[3]) / 14.375);
	gyroXangle += gyroXrate * ((double)(micros() - timer) / 1000000); // Without any filter

	double gyroYrate = (((double)readGyroY() - zeroValue[4]) / 14.375);
	gyroYangle += gyroYrate * ((double)(micros() - timer) / 1000000); // Without any filter

	double accXangle = getXangle();
	double accYangle = getYangle();

	timer = micros();

	int measurement1 = accXangle;
	int measurement2 = accYangle;
	delay(50);

	Serial.println("measurement1 value is");
	Serial.println(measurement1);
	Serial.print("\t");
	Serial.println("measurement2 value is");
	Serial.println(measurement2);
	Serial.print("\t");

	if((accXangle> 343 && accXangle<345) && (accYangle> 2.5 && accYangle<4))//surface 1 detection
	{  
		face=1; 
	}
	if((accXangle> 339.5 && accXangle<342.5) && (accYangle>24.5 && accYangle<25.5 ))//surface 2 detection
	{
		face=2; 
	} 
	if((accXangle> 342.5 && accXangle<343.7) && (accYangle>341 && accYangle<343 ))//surface 3 detection
	{
		face=3;
	}
	if((accXangle> 2.8 && accXangle<4.3) && (accYangle>3.5 && accYangle<5.5)) //surface 4 detection
	{
		face=4;
	}
	if((accXangle>322.5 && accXangle<323.4) && (accYangle>2.8 && accYangle<4 ))//surface 5 detection
	{
		face=5;
	}

	Serial.println("Current Face is");
	Serial.println(face);

	int temp=face;

	switch(levelselect)
	{
		case 1:Serial.println("alphabets");
		       play_audioalphabets();
		       break;

		case 2:Serial.println("numbers");
		       numbers++;
		       break;

		case  3:Serial.println("Months");
			months++;
			break;

		case 4:Serial.println("Days");
		       days++;
		       break;

		case 5:Serial.println("Multiples");
		       multiple++;
		       break;
	}
	previous_face=face;
}  

void i2cWrite(uint8_t address, uint8_t registerAddress, uint8_t data) 
{
	Wire.beginTransmission(address);
	Wire.write(registerAddress);
	Wire.write(data);
	Wire.endTransmission();
}

uint8_t* i2cRead(uint8_t address, uint8_t registerAddress, uint8_t nbytes) 
{
	Wire.beginTransmission(address);
	Wire.write(registerAddress);
	Wire.endTransmission();
	Wire.beginTransmission(address);
	Wire.requestFrom(address, nbytes);
	for (uint8_t i = 0; i < nbytes; i++)
		buff[i] = Wire.read();
	Wire.endTransmission();
	return buff;
}

/*This function measures the y-axis of the gyro*/
int readGyroX() 
{ 
	uint8_t* data = i2cRead(gyroAddress, 0x1F, 2);
	return ((data[0] << 8) | data[1]);
}

/*This funtion measures the x-axis of the gyro*/
int readGyroY() 
{ 
	uint8_t* data = i2cRead(gyroAddress, 0x1D, 2);
	return ((data[0] << 8) | data[1]);
}

double getXangle() 
{
	double accXval = (double)readAccX() - zeroValue[0];
	double accZval = (double)readAccZ() - zeroValue[2];
	double angle = (atan2(accXval, accZval) + PI) * RAD_TO_DEG;
	return angle;
}

double getYangle() 
{
	double accYval = (double)readAccY() - zeroValue[1];
	double accZval = (double)readAccZ() - zeroValue[2];
	double angle = (atan2(accYval, accZval) + PI) * RAD_TO_DEG;
	return angle;
}

int readAccX() 
{
	uint8_t* data = i2cRead(adxlAddress, 0x32, 2);
	return (data[0] | (data[1] << 8));
}

int readAccY() 
{
	uint8_t* data = i2cRead(adxlAddress, 0x34, 2);
	return (data[0] | (data[1] << 8));
}

int readAccZ() 
{
	uint8_t* data = i2cRead(adxlAddress, 0x36, 2);
	return (data[0] | (data[1] << 8));
}

void Digital_write()
{
	Serial.println("Digital Write Successfull");
	delay(100);
	if((mod_count%2)!=0)
	{
		digitalWrite(digitalwritepin,LOW);
		delay(100);
		digitalWrite(digitalwritepin,HIGH);
		delay(100);
	}
	else
	{
		digitalWrite(digitalwritepin,HIGH);
		delay(100);
		digitalWrite(digitalwritepin,LOW);
		delay(100);
	}

}

/*This function plays the alphabet letters based on the cubes current face*/
void play_audioalphabets()
{  

	int next=face-previous_face;
	if(face==1 && next==1)
	{
		alphabets++;
	}
	if(face==2 && next==1)
	{
		alphabets++;
	}
	if(face==3 && next==1)
	{
		alphabets++;
	}
	if(face==4 && next==1)
	{
		alphabets++;
	}
	if(face==5 && next==1)
	{
		alphabets++;

		Digital_write();  
		mod_count++;
		delay(100);
		face=0;
	} 
	if(next>1 || next<0)
	{
		face=previous_face;
		delay(4000);
		tmrpcm.play("try.wav");
		/*play audion try again on wrongly placing the cube surface*/
	}
	if (next==1)
	{
		switch(alphabets)
		{
			case 1: delay(1500);
				tmrpcm.play("a.wav");
				break;
			case 2:delay(1500);
			       tmrpcm.play("b.wav");
			       break;
			case 3:delay(1500);
			       tmrpcm.play("c.wav");
			       break;
			case 4:delay(1500);
			       tmrpcm.play("d.wav");
			       break;
			case 5:delay(1500);
			       tmrpcm.play("e.wav");
			       break;
			case 6:delay(1500);
			       tmrpcm.play("f.wav");
			       break;
			case 7:delay(1500);
			       tmrpcm.play("g.wav");
			       break;
			case 8:delay(1500);
			       tmrpcm.play("h.wav");
			       break;
			case 9:delay(1500);
			       tmrpcm.play("i.wav");
			       break;
			case 10:delay(1500);
				tmrpcm.play("j.wav");
				break;
			case 11:delay(1500);
				tmrpcm.play("k.wav");
				break;
			case 12:delay(1500);
				tmrpcm.play("l.wav");
				break;
			case 13:delay(1500);
				tmrpcm.play("m.wav");
				break;
			case 14:delay(1500);
				tmrpcm.play("n.wav");
				break;
			case 15:delay(1500);
				tmrpcm.play("o.wav");
				break;
			case 16:delay(1500);
				tmrpcm.play("p.wav");
				break;
			case 17:delay(1500);
				tmrpcm.play("q.wav");
				break;
			case 18:delay(1500);
				tmrpcm.play("r.wav");
				break;
			case 19:delay(1500);
				tmrpcm.play("s.wav");
				break;
			case 20:delay(1500);
				tmrpcm.play("t.wav");
				break;
			case 21:delay(1500);
				tmrpcm.play("u.wav");
				break;
			case 22:delay(1500);
				tmrpcm.play("v.wav");
				break;
			case 23:delay(1500);
				tmrpcm.play("w.wav");
				break;
			case 24:delay(1500);
				tmrpcm.play("x.wav");
				break;
			case 25:delay(1500);
				tmrpcm.play("y.wav");
				break;
			case 26:delay(1500);
				tmrpcm.play("z.wav");
				break;

		}
	}
	if(alphabets==26)
	{
		alphabets=0;
		Serial.println("Successfully completed test !!!System Reset");
		face=0;
	}
}

