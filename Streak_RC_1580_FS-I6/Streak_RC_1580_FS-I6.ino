#include <EEPROM.h>

//----------------Extra variables for serial input/output
byte ByteIn = 0, ReadFlag = 0, VariableCounter =0, loopswarn=0, loopscutoff =0;

//Future use
int Battery_warn = 105, Battery_cutoff = 100,adcFactor=35;
int Battery_Volts = 0;
byte Battery_Pin = A3;
const int numReadings = 100;
int readings[numReadings];
int readIndex = 0;
long total = 0;
int average = 0;
int current = 0, previous = 0;
const int interval = 300; 
byte buzzerState = LOW;
byte buzzerPin = A0;
byte state =1;
byte cutOffState = 1;


//Motor pins declaration
#define RightMotor 9     // pin maping according to schematics
#define LeftMotor 10
#define R_dir 8
#define L_dir 12
#define R_brk 13
#define L_brk 11


//RC receiver declarations
#define elePin 7    // Receiver channel 1 pin
#define thrPin 6    // Receiver channel 3 pin
#define rudPin 5    // Receiver channel 4 pin

int ele = 0;    // Receiver channel 1 pwm value
int thr = 0;    // Receiver channel 3 pwm value
int rud = 0;    // Receiver channel 4 pwm value
byte thrAdjust = 1;
byte LeftRightAdjust = 0;

byte temp, temp1;
//Variables to store values
byte ppm1 = 0;
byte ppm2 = 0;
byte ppm3 = 0;

//EEPROM Storage variables
byte maxEle=0,minEle=0,midEle=0,revEle=0;  //Elevation limits
byte maxThr=0,minThr=0,midThr=0,revThr=0;  //Throttle limits
byte maxRud=0,minRud=0,midRud=0,revRud=0;  //Rudder limits
byte temp_serial,dataFlowing,dataStream;

void setup()
{
	TCCR1B = 0B00001010;
	TCCR1A = 0B01010011;
	pinMode(RightMotor,OUTPUT);
	pinMode(LeftMotor,OUTPUT);
	pinMode(R_dir,OUTPUT);
	pinMode(L_dir,OUTPUT);
	pinMode(R_brk,OUTPUT);
	pinMode(L_brk,OUTPUT);
	pinMode(buzzerPin,OUTPUT);
	eepromRead();
	digitalWrite(R_brk,0);
	digitalWrite(L_brk,0);
	Serial.begin(115200);
}

void loop() 
{
	ppm1 = pulseIn(elePin, HIGH,100000)/10;      // Note the duration PWM pin is HIGH  //With timeout of 100ms
	ppm2 = pulseIn(thrPin, HIGH,100000)/10;      // Note the duration PWM pin is HIGH  //With timeout of 100ms
	ppm3 = pulseIn(rudPin, HIGH,100000)/10;      // Note the duration PWM pin is HIGH  //With timeout of 100ms

	if(ppm1<midEle) ele = map(ppm1,midEle,minEle,0,-100);     //Mapping elevation stick to -100 to +100
	else ele = map(ppm1,midEle,maxEle,0,100);

	thr = map(ppm2,minThr,maxThr,0,100);                      //Mapping throttle value to -100 to +100
	if(ppm2==0) thr = 0;

	if(ppm3<midRud) rud = map(ppm3,midRud,minRud,0,-100);     //Mapping rudder value to -100 to +100
	else rud = map(ppm3,midRud,maxRud,0,100);
	serialReceive();    //Software calibration


	//---------------Reverse channel
	if(revEle == 0) ele = ele * 1;
	else if(revEle == 1) ele = ele * -1; //Reverse elevator channel as its reversed for copters in Robokits Remotes.

	if(revThr == 0) thr = thr*1;
	else if(revThr == 1) thr = thr * -1;
	
	if(revRud == 0) rud = rud*1;
	else if(revRud == 1) rud = rud * -1;

	if(cutOffState == 0)
	{
		ele =0;
		rud=0;
		thr=0;
	}   

	if(ele>15)							//If elevator stick is upwards (Deadband 5 points)
	{
		if (rud>15)						//If rudder stick is right
		{
			LMF(ele*thr*0.01);
			temp=(ele*thr*0.01);
			temp1=(rud*thr*0.01);
			if (temp>=temp1) {temp=temp-temp1;} else {temp=0;}
			RMS();
		}
		else if (rud<-15)				//If rudder stick is towards left
		{
			RMF(ele*thr*0.01);
			temp = (ele*thr*0.01);
			temp1=(rud*thr*-0.01);
			if (temp>=temp1) {temp=temp-temp1;} else {temp=0;}
			LMS();
		}
		else							//If rudder stick is neutral
		{
			LMF(ele*thr*0.01);
			RMF(ele*thr*0.01);
		}
	}
	else if(ele<-15)					//If elevator stick is downwards  (Deadband 5 points)
	{
		if (rud>15)						//If rudder stick is right
		{
			LMB(ele*thr*-0.01);
			temp=(ele*thr*-0.01);
			temp1=(rud*thr*0.01);
			if (temp>=temp1) {temp=temp-temp1;} else {temp=0;}
			RMS();
		}
		else if (rud<-15)				//If rudder stick is towards left
		{
			RMB(ele*thr*-0.01);
			temp = (ele*thr*-0.01);
			temp1=(rud*thr*-0.01);
			if (temp>=temp1) {temp=temp-temp1;} else {temp=0;}
			LMS();
		}
		else							//If rudder stick is neutral
		{
			LMB(ele*thr*-0.01);
			RMB(ele*thr*-0.01);
		}
	}
	else								//If Elevator stick is neutral
	{
		if(rud>15)						//If rudder stick is right
		{
			if(thr >= 5 && thr <=25)
				{ thr = 25;}			//Minimun 25% throttle  
			LMF(rud*thr*0.01);
			RMB(rud*thr*0.01);
		}
		else if(rud<-15)				//If rudder stick is towards left
		{
			if(thr >= 5 && thr <=25)
			{thr = 25;}					//Minimun 25% throttle    
			LMB(rud*thr*-0.01);
			RMF(rud*thr*-0.01);
		}
		else							//Stop if no ele or rud stick movement        
		{
			LMS();
			RMS();
		}
	}
}

void Default_Parameters()
{
	maxEle=200;
	minEle=100;
	midEle=150;
	revEle=0;
	maxThr=200;
	minThr=100;
	midThr=150;
	revThr=0;
	maxRud=200;
	minRud=100;
	midRud=150;
	revRud=0;
	Battery_warn = 105;
	Battery_cutoff = 100;
	adcFactor=35;
	thrAdjust = 50;
	LeftRightAdjust = 0;
	eepromWrite();
}

//Eeprom write functions
void eepromWrite()
{
	EEPROM.update(0,minThr);
	EEPROM.update(1,maxThr);
	EEPROM.update(2,midThr);
	EEPROM.update(3,revThr);

	EEPROM.update(4,minEle);
	EEPROM.update(5,maxEle);
	EEPROM.update(6,midEle);
	EEPROM.update(7,revEle);

	EEPROM.update(8,minRud);
	EEPROM.update(9,maxRud);
	EEPROM.update(10,midRud);
	EEPROM.update(11,revRud);

	EEPROM.update(12,Battery_warn);
	EEPROM.update(13,Battery_cutoff);
	EEPROM.update(14,adcFactor);
	EEPROM.update(15,thrAdjust);
	EEPROM.update(16,LeftRightAdjust);
}

void eepromRead()
{
	minThr = EEPROM.read(0);
	maxThr = EEPROM.read(1);
	midThr = EEPROM.read(2);
	revThr = EEPROM.read(3);

	minEle = EEPROM.read(4);
	maxEle = EEPROM.read(5);
	midEle = EEPROM.read(6);
	revEle = EEPROM.read(7);

	minRud = EEPROM.read(8);
	maxRud = EEPROM.read(9);
	midRud = EEPROM.read(10);
	revRud = EEPROM.read(11);

	Battery_warn = EEPROM.read(12);
	Battery_cutoff = EEPROM.read(13);
	adcFactor = EEPROM.read(14);
	thrAdjust = EEPROM.read(15);
	LeftRightAdjust = EEPROM.read(16);
}

void serialReceive()
{
	if(Serial.available()>0)
	{
		ByteIn = Serial.read(); 
		if(ByteIn==240)    //Send RC data
		{
			Serial.write(251);       //Start Byte 
			Serial.write(ppm1); 
			Serial.write(ppm2);  
			Serial.write(ppm3);
			Serial.write(Battery_Volts);
			Serial.write(252);       //End Byte  
		}
		else if (ByteIn == 241)          //Start byte incoming data for RC min max, center, reverse (Startbyte,THRmin, THRmax, THRCenter,THRreverse, ELEmin, ELEmax, ELEcenter,ELEreverse, Rudmin, RudMax, RudCenter,RudReverse,EndByte)
		{
			ReadFlag=241;
			VariableCounter=0;
		}
		else if (ByteIn == 242)          //End byte incoming data for RC min max, center, reverse
		{
			ReadFlag=0; 
		}
		else if (ByteIn == 243)          //Request configuration from board
		{
			Serial.write(253);
			Serial.write(minThr);
			Serial.write(maxThr);
			Serial.write(midThr);
			Serial.write(revThr);
			Serial.write(minEle);
			Serial.write(maxEle);
			Serial.write(midEle);
			Serial.write(revEle);
			Serial.write(minRud);
			Serial.write(maxRud);
			Serial.write(midRud);
			Serial.write(revRud);
			Serial.write(Battery_warn);
			Serial.write(Battery_cutoff);
			Serial.write(adcFactor);
			Serial.write(thrAdjust);
			Serial.write(LeftRightAdjust);
			Serial.write(254);
			}    
		else if (ByteIn == 239)          //Reset EEprom Parameters
		{
			Default_Parameters();
			//--------Sent updated parameters back after resetting
			Serial.write(253);
			Serial.write(minThr);
			Serial.write(maxThr);
			Serial.write(midThr);
			Serial.write(revThr);
			Serial.write(minEle);
			Serial.write(maxEle);
			Serial.write(midEle);
			Serial.write(revEle);
			Serial.write(minRud);
			Serial.write(maxRud);
			Serial.write(midRud);
			Serial.write(revRud);
			Serial.write(Battery_warn);
			Serial.write(Battery_cutoff);
			Serial.write(adcFactor);
			Serial.write(thrAdjust);
			Serial.write(LeftRightAdjust);
			Serial.write(254);
		}
			
		if (ReadFlag==241)     //If rc min, max, center and reverse data is being transmitted
		{
			switch (VariableCounter) 
			{
			case 1:
				minThr = ByteIn;
				break;
			case 2:
				maxThr = ByteIn;
				break;
			case 3:
				midThr = ByteIn;
				break;
			case 4:
				revThr = ByteIn;  //0 if normal, 1 if reverse
				break;
			case 5:
				minEle = ByteIn;
				break;
			case 6:
				maxEle = ByteIn;
				break;
			case 7:
				midEle = ByteIn;
				break;
			case 8:
				revEle = ByteIn;  //0 if normal, 1 if reverse
				break;
			case 9:
				minRud = ByteIn;
				break;
			case 10:
				maxRud = ByteIn;
				break;
			case 11:
				midRud = ByteIn;
				break;
			case 12:
				revRud = ByteIn;  //0 if normal, 1 if reverse
				break;
			case 13:
				Battery_warn  = ByteIn; 
				break;
			case 14:
				Battery_cutoff = ByteIn;
				break;
			case 15:
				adcFactor = ByteIn;
				break;
			case 16:
				thrAdjust = ByteIn;
				break;
			case 17:
				LeftRightAdjust = ByteIn;
				break;
			default: 
				
				break;
			}
			VariableCounter++;
			eepromWrite();
		}
	}
}

//Motor Movement functions
    
void LMF(int Pwm)
{
	if (Pwm<0) {Pwm=Pwm*-1;}
	Pwm=100-Pwm;                  //Because brushless drive runs at lowest speed on full duty cycle.
	Pwm = map(Pwm, 0,100,0,1023);
	digitalWrite(L_brk,0);
	digitalWrite(L_dir,1);
	analogWrite(LeftMotor,Pwm);
}

void LMB(int Pwm)
{
	if (Pwm<0) {Pwm=Pwm*-1;}
	Pwm=100-Pwm;                  //Because brushless drive runs at lowest speed on full duty cycle.
	Pwm = map(Pwm, 0,100,0,1023);
	digitalWrite(L_brk,0);
	digitalWrite(L_dir,0);
	analogWrite(LeftMotor,Pwm);
}

void LMS()
{
	digitalWrite(LeftMotor,0);
	digitalWrite(L_brk,1);
}

void RMF(int Pwm)
{
	if (Pwm<0) {Pwm=Pwm*-1;}
	Pwm=100-Pwm;                  //Because brushless drive runs at lowest speed on full duty cycle.
	Pwm = map(Pwm, 0,100,0,1023);
	digitalWrite(R_brk,0);
	digitalWrite(R_dir,0);
	analogWrite(RightMotor,Pwm);
}

void RMB(int Pwm)
{
	if (Pwm<0) {Pwm=Pwm*-1;}
	Pwm=100-Pwm;                  //Because brushless drive runs at lowest speed on full duty cycle.
	Pwm = map(Pwm, 0,100,0,1023);
	digitalWrite(R_brk,0);
	digitalWrite(R_dir,1);
	analogWrite(RightMotor,Pwm);
}

void RMS()
{
	digitalWrite(RightMotor,0);
	digitalWrite(R_brk,1);  
}
