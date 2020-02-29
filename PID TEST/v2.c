#pragma config(Sensor, port2,  color2,         sensorVexIQ_LED)
#pragma config(Sensor, port3,  color3,         sensorVexIQ_LED)
#pragma config(Sensor, port4,  color5,         sensorVexIQ_LED)
#pragma config(Sensor, port7,  color4,         sensorVexIQ_LED)
#pragma config(Sensor, port8,  gyro,           sensorVexIQ_Gyro)
#pragma config(Sensor, port9,  color1,         sensorVexIQ_LED)
#pragma config(Motor,  motor1,          dropper,       tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor5,          frontarm,      tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor6,          spintake,      tmotorVexIQ, PIDControl, reversed, encoder)
#pragma config(Motor,  motor10,         leftdrive,     tmotorVexIQ, PIDControl, reversed, driveLeft, encoder)
#pragma config(Motor,  motor11,         backarm,       tmotorVexIQ, PIDControl, encoder)
#pragma config(Motor,  motor12,         rightdrive,    tmotorVexIQ, PIDControl, driveRight, encoder)

//GLOBALS
int TIMEOUT = 0;
bool TIMED_OUT = false;

const float TURN_RATE_EXPONENT = PI / 2;
const float LEFT_TURN_RATE_COEFFICIENT = 0.8;
const float RIGHT_TURN_RATE_COEFFICIENT = 1.01;

int avg(int a, int b) {
	return (a + b) / 2;
}

int sq(int a) {
	return a * a;
}

void resetEncoders(int type) {
	switch (type) {
		case 0:
			resetMotorEncoder(dropper);
			resetMotorEncoder(frontarm);
			resetMotorEncoder(spintake);
			resetMotorEncoder(backarm);
			resetMotorEncoder(leftdrive);
			resetMotorEncoder(rightdrive);
			resetGyro(gyro);
			break;
		case 1:
			resetMotorEncoder(dropper);
			resetMotorEncoder(frontarm);
			resetMotorEncoder(spintake);
			resetMotorEncoder(backarm);
			break;
		case 2:
			resetMotorEncoder(leftdrive);
			resetMotorEncoder(rightdrive);
			resetGyro(gyro);
			break;
		case 3:
			resetGyro(gyro);
			break;
		case 4:
			resetMotorEncoder(leftdrive);
			resetMotorEncoder(rightdrive);
			break;
		default:
			resetMotorEncoder(leftdrive);
			resetMotorEncoder(rightdrive);
			resetGyro(gyro);
			break;
	}
}

task runTimeout() {
	TIMED_OUT = false;
	if (TIMEOUT != 0) {
		int counter = 0;
		while (counter < TIMEOUT) {
			counter++;
			sleep(10);
		}
		if (counter >= TIMEOUT) {
			TIMED_OUT = true;
		}
	}
}

int calculateMotorSpeed(int dist, int pos, int max) { //quartic ellipse
	int scaled_input = pos * 100 / dist;
	return max * sqrt(100000000 - sq(sq(scaled_input))) / 10000;
}

int calculateMotorSpeedTurns(int dist, int pos, int max, int exponent) { //ellipse
	int scaled_input = abs(pos) * 100 / dist;
	return pow(abs(scaled_input - 100), exponent) / (pow(100, exponent) / max);
}

void forward(int dist, int timeout, int max_speed) {
	int left_encoder = getMotorEncoder(leftdrive);
	int right_encoder = getMotorEncoder(rightdrive);

	int p_left;
	int d_left;
	int error_previous = 0;
	int integral = 0;
	int base_speed;
	int left_speed;

	TIMEOUT = timeout;
	startTask(runTimeout);

	while (abs(avg(left_encoder, right_encoder)) < dist && !TIMED_OUT) {
		left_encoder = getMotorEncoder(leftdrive);
		right_encoder = getMotorEncoder(rightdrive);

		p_left = right_encoder - left_encoder;
		d_left = error_previous - p_left;
		integral += p_left;
		
		base_speed = calculateMotorSpeed(dist, abs(avg(left_encoder, right_encoder)), max_speed);
		left_speed = base_speed + p_left * 3 + d_left * 0.2 + integral * 0.01;
		
		setMotorSpeed(leftdrive, left_speed);
		setMotorSpeed(rightdrive, base_speed);
		
		error_previous = p_left;
	}

	stopTask(runTimeout);
}

void backward(int dist, int timeout, int max_speed) {
	int left_encoder = getMotorEncoder(leftdrive);
	int right_encoder = getMotorEncoder(rightdrive);

	int p_left;
	int d_left;
	int error_previous = 0;
	int integral = 0;
	int base_speed;
	int left_speed;

	TIMEOUT = timeout;
	startTask(runTimeout);

	while (abs(avg(left_encoder, right_encoder)) < dist && !TIMED_OUT) {
		left_encoder = getMotorEncoder(leftdrive);
		right_encoder = getMotorEncoder(rightdrive);

		p_left = right_encoder - left_encoder;
		d_left = error_previous - p_left;
		integral += p_left;
		
		base_speed = -calculateMotorSpeed(dist, -abs(avg(left_encoder, right_encoder)), max_speed);
		left_speed = base_speed + p_left * 3 + d_left * 0.2+ integral * 0.01;

		setMotorSpeed(leftdrive, left_speed);
		setMotorSpeed(rightdrive, base_speed);
		
		error_previous = p_left;
	}

	stopTask(runTimeout);
}

void turnLeft(int angle, int timeout, int max_speed) {
	resetEncoders(2);
	int left_encoder = getMotorEncoder(leftdrive);
	int right_encoder = getMotorEncoder(rightdrive);

	int p_left;
	int base_speed;
	int left_speed;

	TIMEOUT = timeout;
	startTask(runTimeout);

	while (abs(getGyroDegrees(gyro)) < angle * LEFT_TURN_RATE_COEFFICIENT && !TIMED_OUT) {
		left_encoder = getMotorEncoder(leftdrive);
		right_encoder = getMotorEncoder(rightdrive);

		p_left = abs(right_encoder) - abs(left_encoder);
		base_speed = calculateMotorSpeedTurns(angle * LEFT_TURN_RATE_COEFFICIENT, getGyroDegrees(gyro), max_speed, TURN_RATE_EXPONENT);
		left_speed = -base_speed - p_left * 0.9;

		datalogAddValue(0, base_speed);
		
		setMotorSpeed(leftdrive, left_speed);
		setMotorSpeed(rightdrive, base_speed);
	}

	stopTask(runTimeout);
}

void turnRight(int angle, int timeout, int max_speed) {
	resetEncoders(2);
	int left_encoder = getMotorEncoder(leftdrive);
	int right_encoder = getMotorEncoder(rightdrive);

	int p_left;
	int base_speed;
	int left_speed;

	TIMEOUT = timeout;
	startTask(runTimeout);

	while (abs(getGyroDegrees(gyro)) < angle * RIGHT_TURN_RATE_COEFFICIENT && !TIMED_OUT) {
		left_encoder = getMotorEncoder(leftdrive);
		right_encoder = getMotorEncoder(rightdrive);

		p_left = abs(right_encoder) - abs(left_encoder);
		base_speed = -calculateMotorSpeedTurns(angle * RIGHT_TURN_RATE_COEFFICIENT, getGyroDegrees(gyro), max_speed, TURN_RATE_EXPONENT);
		left_speed = -base_speed + p_left * 0.9;

		datalogAddValue(0, base_speed);
		
		setMotorSpeed(leftdrive, left_speed);
		setMotorSpeed(rightdrive, base_speed);
	}

	stopTask(runTimeout);
}

task main() {
	datalogClear();
	resetEncoders(0);
}
