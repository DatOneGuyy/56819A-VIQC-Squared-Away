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
			break;
		default:
			resetMotorEncoder(leftdrive);
			resetMotorEncoder(rightdrive);
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

int calculateMotorSpeed(int dist, int pos) {
	int scaled_input = pos * 100 / dist;
	return sqrt(100000000 - sq(sq(scaled_input))) / 100;
}

void forward(int dist, int timeout) {
	int left_encoder = getMotorEncoder(leftdrive);
	int right_encoder = getMotorEncoder(rightdrive);
	
	int p_left;
	int base_speed;
	int left_speed;
	
	TIMEOUT = timeout;
	startTask(runTimeout);
	
	while (abs(avg(left_encoder, right_encoder)) < dist && !TIMED_OUT) {
		left_encoder = getMotorEncoder(leftdrive);
		right_encoder = getMotorEncoder(rightdrive);
		
		p_left = right_encoder - left_encoder;
		base_speed = calculateMotorSpeed(dist, abs(avg(left_encoder, right_encoder)));
		left_speed = base_speed + p_left * 0.9;
		
		setMotorSpeed(leftdrive, left_speed);
		setMotorSpeed(rightdrive, base_speed);
	}
	
	stopTask(runTimeout);
}

task main() {
	resetEncoders(0);
	forward(500, 300);
}
