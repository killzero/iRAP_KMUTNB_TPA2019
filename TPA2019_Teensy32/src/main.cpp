#include <Arduino.h>

uint8_t motor_pin[4][3] = {{5, 6, 7}, {5, 6, 7}, {5, 6, 7}, {5, 6, 7}};
uint8_t enco_pin[4] = {6, 7, 8, 9};

uint8_t buff_index = 0;
uint8_t buffer[10];

int16_t setpoint[4];

uint32_t encoPulse[4];

void setup()
{
	// ------ set buadrate ----
	Serial.begin(9600);
	Serial1.begin(38400);

	for (uint8_t i = 0; i < 4; i++)
	{
		for (uint8_t j = 0; j < 3; j++)
		{
			pinMode(motor_pin[i][j], OUTPUT);
		}
		pinMode(enco_pin[i], INPUT);
		setpoint[i] = 0;
		encoPulse[i] = 0;
	}

	for (uint8_t i = 0; i < 10; i++)
	{
		buffer[i] = 0;
	}
	//--------------- interupt -------------//
	// attachInzterrupt(digitalPinToInterrupt(EncoPinA), encoderA, FALLING);
	// attachInterrupt(digitalPinToInterrupt(EncoPinB), encoderB, FALLING);
	// attachInterrupt(digitalPinToInterrupt(EncoPinC), encoderC, FALLING);
	// attachInterrupt(digitalPinToInterrupt(EncoPinD), encoderD, FALLING);
}

void loop()
{
	// put your main code here, to run repeatedly:
}

void drive_pwm(uint8_t motor, int16_t speed)
{
	speed = constrain(speed, -255, 255);
	// ---- forward direction ---- //
	if (speed > 0)
	{
		digitalWrite(motor_pin[motor][1], HIGH);
		digitalWrite(motor_pin[motor][2], LOW);
	}
	else if (speed < 0)
	{
		digitalWrite(motor_pin[motor][1], LOW);
		digitalWrite(motor_pin[motor][2], HIGH);
	}
	else
	{
		digitalWrite(motor_pin[motor][1], LOW);
		digitalWrite(motor_pin[motor][2], LOW);
	}
	analogWrite(motor_pin[motor][0], abs(speed));
}

void serialEvent1()
{
	while (Serial1.available())
	{
		buffer[buff_index] = (uint8_t)Serial1.read(); // get data
		buff_index = (buff_index + 1) % 10;			  // limit index
		if (buff_index == 1 && buffer[0] != 0x55)
		{
			buff_index = 0;
			return; // break when get data fail
		}

		if (buff_index == 0 && buffer[9] == 0xc9)
		{
			setpoint[0] = (int16_t)((buffer[1] << 8 | buffer[2]));
			setpoint[1] = (int16_t)((buffer[3] << 8 | buffer[4]));
			setpoint[2] = (int16_t)((buffer[5] << 8 | buffer[6]));
			setpoint[3] = (int16_t)((buffer[7] << 8 | buffer[8]));

			//temporary fix decode negative value (max -600)
			if (setpoint[0] >= 64936)
				setpoint[0] -= 65536;
			if (setpoint[1] >= 64936)
				setpoint[1] -= 65536;
			if (setpoint[2] >= 64936)
				setpoint[2] -= 65536;
			if (setpoint[3] >= 64936)
				setpoint[3] -= 65536;
		}
	}
}

void countEnco0(){
	encoPulse[0]++;
}
