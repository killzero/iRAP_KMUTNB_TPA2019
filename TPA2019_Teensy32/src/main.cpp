#include <Arduino.h>
#include <Encoder.h>
// ------------------------ PWM INA INB ------
uint8_t motor_pin[4][3] = {{20, 11, 12}, {22, 17, 16}, {21, 15, 14}, {23, 18, 19}};
Encoder encoPulse[4] = {Encoder(3, 2), Encoder(4, 5), Encoder(6, 7), Encoder(8, 9)};

uint8_t buff_index = 0;
uint8_t buffer[12];

float kp[4] = {1.0f, 1.0f, 1.0f, 1.0f};
float ki[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float kd[4] = {0.0f, 0.0f, 0.0f, 0.0f};

int16_t sumError[4];
int16_t lastError[4];
int16_t setpoint[4][2]; // currentsetPoint, lastsetPoint
int16_t currentSpeed[4];

//uint32_t encoPulse[4];
uint32_t encoTime[4];
int32_t lastPulse[4];

uint32_t _calTime;
uint32_t _printTime;
// declare befor use
void drive_pwm(uint8_t motor, int16_t speed);
int16_t getRPM(uint8_t enco);
int16_t getOutput(uint8_t motor, int16_t setpoint);

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
		sumError[i] = 0;
		lastError[i] = 0;
		setpoint[i][0] = 0;
		setpoint[i][1] = 0;
		encoTime[i] = 0;
		currentSpeed[i] = 0;
		lastPulse[i] = 0;
	}
	_calTime = 0;
	_printTime = 0;
	for (uint8_t i = 0; i < 10; i++)
	{
		buffer[i] = 0;
	}
}

void loop()
{
	uint32_t _diffTime = micros() - _calTime;
	if (_diffTime > 10) //10
	{
		//Serial.println(encoPulse[0].read());
		//drive_pwm(3, 120);
		for (uint8_t i = 0; i < 4; i++)
		{
			drive_pwm(i, setpoint[i][0]);
			//Serial.println(setpoint[0]);
		}
		_calTime = micros();
	}
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

		buff_index = buff_index % 12;			   // limit index
		buffer[buff_index] = (char)Serial1.read(); // get data

		if (buff_index == 0 && buffer[0] != '#')
		{
			return; // break when get data fail
		}

		buff_index++;

		if (buff_index == 11 && buffer[10] == '\r')
		{
			if (buffer[1] == 's')
			{
				setpoint[0][0] = (int)((buffer[2] << 8 | buffer[3]));
				setpoint[1][0] = (int)((buffer[4] << 8 | buffer[5]));
				setpoint[2][0] = (int)((buffer[6] << 8 | buffer[7]));
				setpoint[3][0] = (int)((buffer[8] << 8 | buffer[9]));

				//temporary fix decode negative value (max -600)
				if (setpoint[0][0] >= 64936)
					setpoint[0][0] -= 65536;
				if (setpoint[1][0] >= 64936)
					setpoint[1][0] -= 65536;
				if (setpoint[2][0] >= 64936)
					setpoint[2][0] -= 65536;
				if (setpoint[3][0] >= 64936)
					setpoint[3][0] -= 65536;
			}
			buff_index = 0;
		}
	}
}

int16_t getRPM(uint8_t enco)
{
	/* --------------------------------------------------------
   * ---- * 6000 = 100 * 60 from 10 millisec to 1 minute ----
   * ---- / 200 pulse to RPM --------------------------------
   * ---- pulse * 1000 * 60 / 800 / encoTime0.01(microsec)
   * ---- RPM from pulse/micro
   * ----------------------------------------------------- */
	currentSpeed[enco] = encoPulse[enco].read() * 75 / encoTime[enco];
	encoTime[enco] = 0;
	encoPulse[enco].write(0);
	//return (int16_t)currentSpeed[enco];
	return currentSpeed[enco];
}

int16_t getOutput(uint8_t motor, int16_t _setpoint)
{
	/*  ----------------------------------------- 
	*   -------------   PID calculate pseudocode ---------------
   	*   error = setpoint - actual_position
   	*   integral = integral + (error*dt)
   	*   derivative = (error - previous_error)/dt
   	*   output = (Kp*error) + (Ki*integral) + (Kd*derivative)
   	*  --------------------------------------------------------*/
	int16_t sat = 150; // saturation limit of sum_error(integral)

	int16_t error = _setpoint - getRPM(motor); // current error

	int8_t _direction = 0;
	if (error > 0)
		_direction = 1;
	else if (error < 0)
		_direction = -1;

	if (_setpoint != setpoint[motor][1])
		sumError[motor] = 0;

	sumError[motor] = sumError[motor] + abs(error); // *dt (error * 0.01)
	int diff_error = (error - lastError[motor]);	// / 0.01 different of error
	if (sumError[motor] > sat)
	{
		sumError[motor] = sat;
	}
	lastError[motor] = error;

	setpoint[motor][1] = _setpoint;

	float output = kp[motor] * error + (ki[motor] * sumError[motor]) + (kd[motor] * diff_error);
	output = constrain(output, 0, 255) * _direction; // limit PWM

	return error; //(int16_t)output;
}

void TunePID()
{
	int16_t error = getOutput(0, setpoint[0][0]);
	if (millis() - _printTime > 100)
	{
		Serial.print("S: "); //setpoint
		Serial.print(setpoint[0][0]);
		Serial.print("RPM: ");
		Serial.print(getRPM(0));

		
		_printTime = millis();
	}
}