#include <Arduino.h>
#include <PS4BT.h>
#include <usbhub.h>
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
int16_t sumSpeed;
float radian;
int8_t yaw;
void setup()
{
	Serial.begin(9600);
	Serial1.begin(38400);

	delay(200);
	yaw = 0;
	radian = 0;
	sumSpeed = 0;
	sendDataTime = 0;
}

void loop()
{
	if (millis() - sendDataTime > 20)
	{
		int16_t *_vSpeed[4];
		*_vSpeed = getVector(sumSpeed, radian, yaw);
		sendSpeed(*_vSpeed);

		sendDataTime = millis();
	}
}

void initPS4()
{
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
	uint8_t dataset[4][2];

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
}

int16_t *getVector(int16_t velocity, float Ceta, int16_t rotate)
{
	float cCeta = cos(Ceta);
	float sCeta = sin(Ceta);

	/* ----------------------- Auto -------------------------------
	float vFL = (velocity * ((cCeta * -0.707106) + (sCeta * -0.707106)) + rotate);
	float vFR = (velocity * ((cCeta * -0.707106) + (sCeta * 0.707106)) + rotate);
	float vBL = (velocity * ((cCeta * 0.707106) + (sCeta * -0.707106)) + rotate);
	float vBR = (velocity * ((cCeta * 0.707106) + (sCeta * 0.707106)) + rotate);
	*/
	float vFL = (velocity * ((cCeta * 0.707106) + (sCeta * 0.707106)) - rotate);
	float vFR = (velocity * ((cCeta * -0.707106) + (sCeta * 0.707106)) + rotate);
	float vBL = (velocity * ((cCeta * -0.707106) + (sCeta * 0.707106)) - rotate);
	float vBR = (velocity * ((cCeta * 0.707106) + (sCeta * 0.707106)) + rotate);
	int16_t vSpeed[4] = {(int)vFL, (int)vFR, (int)vBL, (int)vBR};
	return vSpeed;
}

void readJoyPS4()
{
	if (PS4.connected())
	{
		int16_t maxSpeed = 200;
		sumSpeed = 0;

		uint8_t analog_Lx = PS4.getAnalogHat(LeftHatX);
		uint8_t analog_Ly = PS4.getAnalogHat(LeftHatY);
		uint8_t analog_Rx = PS4.getAnalogHat(RightHatX);
		// ------------------------------------------
		bool center_Lx = (116 < analog_Lx & analog_Lx < 160);
		bool center_Ly = (116 < analog_Ly & analog_Ly < 140);
		bool center_Rx = (116 < analog_Rx & analog_Rx < 140);

		radian = 0;
		int16_t sp_Lx = map(analog_Lx, 255, 0, -maxSpeed, maxSpeed) * -1;
		int16_t sp_Ly = map(analog_Ly, 255, 0, -maxSpeed, maxSpeed);
		sumSpeed = sqrt((sp_Lx * sp_Lx) + (sp_Ly * sp_Ly));

		if (center_Lx && center_Ly)
		{
			radian = 0;
			sumSpeed = 0;
		}
		else if (sp_Lx > 0 && center_Ly)
			radian = 0.0f; // 0
		else if (center_Lx && sp_Ly > 0)
			radian = 1.5708f; // 90
		else if (sp_Lx < 0 && center_Ly)
			radian = 3.141593f; // 180
		else if (center_Lx && sp_Ly < 0)
			radian = -1.5708f; // 270 //4.712389f;
		else
			radian = atan2(sp_Ly, sp_Lx);

		yaw = 0;
		if (!center_Rx)
		{
			yaw = map(analog_Rx, 0, 255, -150, 150) * -1;
		}
	}
}