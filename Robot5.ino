/*
   This sketch uses the calibration saved in the EEPROM (don't forget to calibrate your RC by using the
   calibration sketch) to control four Servos by using an RC
*/

#include <FastRCReader.h>
#include <EEPROM.h>
#include <Servo.h>

//defines for the controller.

#define MinValueLeft 712
#define MaxValueLeft 2000
#define CenterValueLeftHigh 1500
#define CenterValueLeftLow  1400

#define MinValueRight 960
#define MaxValueRight 2020
#define CenterValueRightHigh 1300
#define CenterValueRightLow  1280


//Motors Definitions

#define DirectionMotorA 12
#define BreakMotorA 9
#define PWMMotorA 3

#define DirectionMotorB 13
#define BreakMotorB 8
#define PWMMotorB 11


//0 and 2 are left and right Y axes
//Port definitions
#define CHANNELAMOUNT 6
const uint8_t channelPins[CHANNELAMOUNT] = { 4, 5, 6, 7, 8, 9};
uint16_t ch1;
uint16_t ch2;
//The adress the calibration/mapping is saved in the EEPROM
#define EEPROMADRESS 0

RCChannelMapper RC;

void setup() {
	Serial.begin(115200);
	Serial.println("Start.");
	//Setup Channel A
	pinMode(DirectionMotorA, OUTPUT); //Initiates Motor Channel A pin
	pinMode(BreakMotorA, OUTPUT); //Initiates Brake Channel A pin

	//Setup Channel B
	pinMode(DirectionMotorB, OUTPUT); //Initiates Motor Channel A pin
	pinMode(BreakMotorB, OUTPUT);  //Initiates Brake Channel A pin

	Serial.println("Motors initialised.");

	RC.begin();

	for (uint8_t i = 0; i < CHANNELAMOUNT; i++) {
		RC.addChannel(channelPins[i]);
		Serial.print("Added Channel "); Serial.print(i + 1); Serial.print(" on Port "); Serial.print(channelPins[i]);
	}

	Serial.println("\nAdded/Attached all Ports\n");

	EEPROM.get(EEPROMADRESS, RC);

	Serial.println("Red from EEPROM, starting loop");

	/*
	  unsigned long currentTime = millis();
	  while((millis() - currentTime) < 3000);
	*/
}

void loop() {

/*	for (uint8_t i = 0; i < CHANNELAMOUNT; i++) {
		Serial.print("[");
		Serial.print(i);
		Serial.print("] ");
		Serial.print(RC.getFreq(channelPins[i]));
		Serial.print(" - ");
	}
	Serial.println(".");*/

	ch1 = RC.getFreq(channelPins[0]);
	ch2 = RC.getFreq(channelPins[2]);

	//Motor 1 
	if (ch1 > CenterValueLeftHigh)
	{
	//	Serial.println("CH1 Forward");
		digitalWrite(DirectionMotorA, HIGH); //Establishes forward direction of Channel A
		digitalWrite(BreakMotorA, LOW);   //Disengage the Brake for Channel A
		analogWrite(PWMMotorA, map(ch1, CenterValueLeftHigh, MaxValueLeft, 0, 254));   //Spins the motor on Channel A at full speed		

	}
	else if (ch1 < CenterValueLeftLow)
	{	

		digitalWrite(DirectionMotorA, LOW); //Establishes forward direction of Channel A
		digitalWrite(BreakMotorA, LOW);   //Disengage the Brake for Channel A
		analogWrite(PWMMotorA, map(ch1,CenterValueLeftLow, MinValueLeft , 0, 254));   //Spins the motor on Channel A at full speed		

	}
	else{
		//Serial.println("CH1 Stop");
		digitalWrite(BreakMotorA, HIGH);   //Disengage the Brake for Channel A
	}

	///Motor 2 
	if (ch2 > CenterValueRightHigh)
	{
	//	Serial.println("CH2 Forward");
		digitalWrite(DirectionMotorB, HIGH); //Establishes forward direction of Channel A
		digitalWrite(BreakMotorB, LOW);   //Disengage the Brake for Channel A
		analogWrite(PWMMotorB, map(ch2, CenterValueRightHigh, MaxValueRight, 0, 254));   //Spins the motor on Channel A at full speed
	}
	else if (ch2 < CenterValueRightLow)
	{			
		digitalWrite(DirectionMotorB, LOW); //Establishes forward direction of Channel A
		digitalWrite(BreakMotorB, LOW);   //Disengage the Brake for Channel A
		analogWrite(PWMMotorB, map(ch2, CenterValueRightLow, MinValueRight, 0, 254));   //Spins the motor on Channel A at full speed
	}
	else{
	//	Serial.println("CH2 Stop");		
		digitalWrite(BreakMotorB, HIGH);   //Disengage the Brake for Channel A
	}

    //delay(200);
}
