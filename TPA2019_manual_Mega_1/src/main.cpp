#include <Arduino.h>

#include <PS4BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
// You will need to hold down the PS and Share button at the same time,
//PS4BT PS4(&Btd, PAIR);
// After that you can simply create the instance like so and then press the PS button on the device
PS4BT PS4(&Btd);

#define D2R PI / 180
#define R2D 180 / PI

uint32_t sendDataTime;
uint32_t readJoyTime;
int16_t sumSpeed;
float radian;
int8_t yaw;
int16_t vSpeed[4];

void initPS4();
void sendSpeed(int16_t speed[4]);
int16_t *getVector(int16_t velocity, float Ceta, int16_t rotate);
void readJoyPS4();

void setup()
{
	Serial.begin(9600);
	Serial1.begin(38400);

	initPS4();

	delay(1000);
	yaw = 0;
	radian = 0;
	sumSpeed = 0;
	readJoyTime = 0;
	sendDataTime = 0;
}

void loop()
{
	Usb.Task();
	if (millis() - readJoyTime > 5)
	{
		readJoyPS4();
		readJoyTime = millis();
	}
	//readJoyPS4();
	if (millis() - sendDataTime > 10) //20
	{
		getVector(sumSpeed, radian, yaw);
		sendSpeed(vSpeed);
		// Serial.print(" Sp :");
		// Serial.print(vSpeed[0]);
		// Serial.print(" Sp :");
		// Serial.print(vSpeed[1]);
		// Serial.print(" Sp :");
		// Serial.print(vSpeed[2]);
		// Serial.print(" Sp :");
		// Serial.print(vSpeed[3]);
		// Serial.println();
		sendDataTime = millis();
	}
}
void initPS4()
{
#if !defined(__MIPSEL__)
	while (!Serial)
		; // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
	if (Usb.Init() == -1)
	{
		Serial.print(F("\r\nOSC did not start"));
		while (1)
			; // Halt
	}
	Serial.print(F("\r\nPS4 Bluetooth Library Started"));
}

void sendSpeed(int16_t speed[4])
{
	// --------- communication ----------
	byte data1_L, data2_L, data3_L, data4_L;
	byte data1_M, data2_M, data3_M, data4_M;

	data1_L = vSpeed[0] & 0x00ff;
	data1_M = (vSpeed[0] >> 8) & 0x00ff;

	data2_L = vSpeed[1] & 0x00ff;
	data2_M = (vSpeed[1] >> 8) & 0x00ff;

	data3_L = vSpeed[2] & 0x00ff;
	data3_M = (vSpeed[2] >> 8) & 0x00ff;

	data4_L = vSpeed[3] & 0x00ff;
	data4_M = (vSpeed[3] >> 8) & 0x00ff;

	char cmd[12] = {'#', 's', data1_M, data1_L, data2_M, data2_L, data3_M, data3_L, data4_M, data4_L, '\r', '\n'};
	for (int i = 0; i < 12; i++)
	{
		Serial1.write(cmd[i]);
	}
	/*
	// --------- communication ----------
	uint8_t dataset[4][2];
	//Serial.println(speed[0]);
	for (uint8_t i = 0; i < 4; i++)
	{
		dataset[i][0] = speed[i] & 0x00ff;
		dataset[i][1] = (speed[i] >> 8) & 0x00ff;
	}
	uint8_t firstByte = 0x55; // 0101 0101
	uint8_t lastByte = 0xc9;  // 1100 1001
	Serial1.write(firstByte);

	for (uint8_t i = 0; i < 4; i++)
	{

		Serial1.write(dataset[i][0]);
		Serial1.write(dataset[i][1]);
		delay(1);
	}
	Serial1.write(lastByte);
	*/
}

int16_t *getVector(int16_t velocity, float Ceta, int16_t rotate)
{
	float cCeta = cos(Ceta);
	float sCeta = sin(Ceta);

	// ----------------------- Auto -------------------------------
	float vFL = (velocity * ((cCeta * -0.707106) + (sCeta * -0.707106)) + rotate);
	float vFR = (velocity * ((cCeta * -0.707106) + (sCeta * 0.707106)) + rotate);
	float vBL = (velocity * ((cCeta * 0.707106) + (sCeta * -0.707106)) + rotate);
	float vBR = (velocity * ((cCeta * 0.707106) + (sCeta * 0.707106)) + rotate);
	/*
	float vFL = (velocity * ((cCeta * 0.707106) + (sCeta * 0.707106)) - rotate);
	float vFR = (velocity * ((cCeta * -0.707106) + (sCeta * 0.707106)) + rotate);
	float vBL = (velocity * ((cCeta * -0.707106) + (sCeta * 0.707106)) - rotate);
	float vBR = (velocity * ((cCeta * 0.707106) + (sCeta * 0.707106)) + rotate);
	*/
	vSpeed[0] = (int16_t)vFL;
	vSpeed[1] = (int16_t)vFR;
	vSpeed[2] = (int16_t)vBL;
	vSpeed[3] = (int16_t)vBR;
	return vSpeed;
}

void readJoyPS4()
{
	
	if (PS4.connected())
	{
		int16_t maxSpeed = 150;
		sumSpeed = 0;

		uint8_t analog_Lx = PS4.getAnalogHat(LeftHatX);
		uint8_t analog_Ly = PS4.getAnalogHat(LeftHatY);
		uint8_t analog_Rx = PS4.getAnalogHat(RightHatX);

		bool _up = PS4.getButtonPress(UP);
		bool _right = PS4.getButtonPress(RIGHT);
		bool _down = PS4.getButtonPress(DOWN);
		bool _left = PS4.getButtonPress(LEFT);
		bool arrowPress = _up || _right || _down || _left;

		//Serial.println(sumSpeed);
		//
		// ------------------------------------------
		bool center_Lx = (116 < analog_Lx && analog_Lx < 160);
		bool center_Ly = (116 < analog_Ly && analog_Ly < 140);
		bool center_Rx = (116 < analog_Rx && analog_Rx < 140);

		radian = 0;
		int16_t sp_Lx = map(analog_Lx, 255, 0, -maxSpeed, maxSpeed) * -1;
		int16_t sp_Ly = map(analog_Ly, 255, 0, -maxSpeed, maxSpeed);

		sumSpeed = maxSpeed;
		if (center_Lx && center_Ly)
		{
			radian = 0;
			sumSpeed = 0;
		}
		else if ((sp_Lx > 0 && center_Ly) || _right)
		{
			radian = 0.0f; // 0
		}
		else if ((center_Lx && sp_Ly > 0) || _up)
		{
			radian = 1.5708f; // 90
		}
		else if ((sp_Lx < 0 && center_Ly) || _left)
		{
			radian = 3.141593f; // 180
		}
		else if ((center_Lx && sp_Ly < 0) || _down)
		{
			radian = -1.5708f; // 270 //4.712389f;
		}
		else
		{
			radian = atan2(sp_Ly, sp_Lx);
			//double sum = (sp_Lx * sp_Lx) + (sp_Ly * sp_Ly);
			//sumSpeed = max(abs(sp_Lx), abs(sp_Ly));
			//Serial.println(sumSpeed);
		}

		yaw = 0;
		if (!center_Rx)
		{
			yaw = map(analog_Rx, 0, 255, -150, 150);
		}
	}
	else
	{
		//Serial.println("Joy not connect");
	}
}
