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

uint32_t sendDataTime;
uint32_t readJoyTime;
uint32_t missionTime;
uint32_t _tempTime;
int16_t sumSpeed;
float radian;
int8_t yaw;
int16_t vSpeed[4];

uint8_t linear[3] = {46, 48, 50}; // pwm in1 in2
uint8_t griper[3] = {53, 45, 47}; // grip slide shoot
uint8_t plate[2] = {49, 51};	  //grip slide
bool gripped, gripping, shooted, shooting;

void initPS4();
void sendSpeed(int16_t speed[4]);
int16_t *getVector(int16_t velocity, float Ceta, int16_t rotate);
void readJoyPS4();
void getAnalog(uint8_t _Lx, uint8_t _Ly, uint8_t _Rx);
void drive(uint8_t pin[3], int16_t speed);
void inGame();

void setup()
{
	Serial.begin(9600);
	Serial1.begin(38400);

	for (uint8_t i = 0; i < 3; i++)
	{
		pinMode(linear[i], OUTPUT);
	}
	drive(linear, -200);
	initPS4();

	delay(2000);
	drive(linear, 0);
	yaw = 0;
	radian = 0;
	sumSpeed = 0;
	readJoyTime = 0;
	sendDataTime = 0;
}

void loop()
{
	Usb.Task();
	if (millis() - readJoyTime > 2)
	{
		readJoyPS4();
		readJoyTime = millis();
	}
	//readJoyPS4();
	if (millis() - sendDataTime > 10) //10 20
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
	inGame();
	if (millis() - missionTime > 20)
	{
		//
		//drive(linear, 200);
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
	float vFL = (velocity * ((cCeta * -0.707106) + (sCeta * -0.707106)) - rotate);
	float vFR = (velocity * ((cCeta * -0.707106) + (sCeta * 0.707106)) - rotate);
	float vBL = (velocity * ((cCeta * 0.707106) + (sCeta * -0.707106)) - rotate);
	float vBR = (velocity * ((cCeta * 0.707106) + (sCeta * 0.707106)) - rotate);
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

		sumSpeed = 0;

		uint8_t analog_Lx = PS4.getAnalogHat(LeftHatX);
		uint8_t analog_Ly = PS4.getAnalogHat(LeftHatY);
		uint8_t analog_Rx = PS4.getAnalogHat(RightHatX);

		getAnalog(analog_Lx, analog_Ly, analog_Rx);

		//Serial.println(sumSpeed);
		//
	}
	else
	{
		sumSpeed = 0;
		//Serial.println("Joy not connect");
	}
}

void getAnalog(uint8_t _Lx, uint8_t _Ly, uint8_t _Rx)
{
	int16_t maxSpeed = 150;
	// ------------------------------------------
	bool center_Lx = (116 < _Lx && _Lx < 160);
	bool center_Ly = (116 < _Ly && _Ly < 140);
	bool center_Rx = (116 < _Rx && _Rx < 140);

	radian = 0;
	int16_t sp_Lx = map(_Lx, 255, 0, -maxSpeed, maxSpeed) * -1;
	int16_t sp_Ly = map(_Ly, 255, 0, -maxSpeed, maxSpeed);

	sumSpeed = maxSpeed;
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
	{
		radian = atan2(sp_Ly, sp_Lx);
		//double sum = (sp_Lx * sp_Lx) + (sp_Ly * sp_Ly);
		//sumSpeed = max(abs(sp_Lx), abs(sp_Ly));
		//Serial.println(sumSpeed);
	}

	yaw = 0;
	if (!center_Rx)
	{
		yaw = map(_Rx, 0, 255, -150, 150);
		// Serial.println(analog_Rx);
	}
}

void drive(uint8_t pin[3], int16_t speed)
{
	speed = constrain(speed, -255, 255);
	if (speed > 0)
	{
		digitalWrite(pin[1], HIGH);
		digitalWrite(pin[2], LOW);
		digitalWrite(pin[0], HIGH));
	}
	else if (speed < 0)
	{
		digitalWrite(pin[1], LOW);
		digitalWrite(pin[2], HIGH);
		digitalWrite(pin[0], HIGH));
	}
	else
	{
		digitalWrite(pin[1], LOW);
		digitalWrite(pin[2], LOW);
	}
	
	//analogWrite(pin[0], abs(speed));
}

void grip()
{
	digitalWrite(griper[0], HIGH);
	gripping = true;
	// if (millis() - _tempTime > 3000)
	// {
	// 	gripping = false;
	// 	gripped = true;
	// 	Serial.println("Grip");
	// }
	if (millis() - _tempTime > 500)
	{
		drive(linear, 220);
		//Serial.println("Lifting");
		gripping = false;
	 	gripped = true;
	}
}

void reGrip()
{
	digitalWrite(griper[0], LOW);
	drive(linear, -200);
	gripped = false;
}

void shoot()
{
	digitalWrite(griper[0], LOW); // grip open
	shooting = true;
	if (millis() - _tempTime > 3000)
	{
		drive(linear, -200);
		gripped = false;
		shooting = false;
		shooted = true;
		//Serial.println("Shoot3");
	}
	else if (millis() - _tempTime > 2500)
	{
		digitalWrite(griper[3], LOW);
		//Serial.println("Shoot2");
	}
	else if (millis() - _tempTime > 50)
	{
		digitalWrite(griper[3], HIGH); // shoot
		//Serial.println("Shoot1");
	}
}

void slidePlate(bool state)
{
	digitalWrite(plate[1], state);
}

void gripPlate(bool state)
{
	digitalWrite(plate[0], state);
}

void inGame()
{
	bool _right = PS4.getButtonClick(RIGHT);
	bool _left = PS4.getButtonClick(LEFT);
	bool x = PS4.getButtonClick(CROSS);
	bool sq = PS4.getButtonClick(SQUARE);
	bool tri = PS4.getButtonClick(TRIANGLE);
	bool cir = PS4.getButtonClick(CIRCLE);

	if (!gripped && x)
	{
		_tempTime = millis();
		grip();
	}
	else if (gripped && tri)
	{
		reGrip();
		//Serial.println("Free");
	}
	else if (gripped && sq)
	{
		_tempTime = millis();
		shoot();
	}

	if (gripping)
	{
		grip();
		//Serial.println("gripping");
	}
	else if (shooting)
	{
		shoot();
		//Serial.println("shooting");
	}
}