// Main Loop for Balance Bot Black
/* Encoders */
	#include "eqep.h"
	#include "util.h"
	#include <cstdlib>
/* IMU */
	#include "MPU9250.h"
	#include <iostream>
	#include <unistd.h>
	#include <math.h>
	#include <stdio.h>
	#include <iomanip>
/* PWM */
	#include "PWM.h"
	//#include "util.h"
	#include <cstdlib>
	//#include <iostream>
/* Motor Driver */
	#include "L298N.h"
	#include "GPIO.h"
	//#include "PWM.h"
	//#include <cstdlib>
	#include <string>
	//#include <iostream>

using namespace ZJ;
using namespace std;

	

int main(){
	cout << "Start\n";
	/* Initialize the IMU */
	MPU9250 imu(2, 0x68);
    imu.setAccRange(MPU9250::PLUSMINUS_16_G);
    imu.setGyroRange(MPU9250::PLUSMINUS_2000_DPS);
	
	
	/* Initialize variables outside the loop */
	int killSwitch = 1;
	
	EQEP encoder1(0);	//P8_34, P8_33
	int pos1 = 0;
	EQEP encoder2(1);	//P8_12, P8_11
		// GPIO: write failed to open file : Permission denied
	int pos2 = 0;
	
	
	L298N motor1("pwm0",4,5); //P9_18, P9_17
	float volt1 = 0.0;
    L298N motor2("pwm1",30,31); //P9_11, P9_13
	float volt2 = 0.0;
	
	while (killSwitch){
	/* Read Encoders */
		pos1 = encoder1.getPosition();
		pos2 = encoder2.getPosition();
		
		
	/* Read IMU */
		imu.readSensor();
		cout << imu.getAccPitch() << "\n";
		// imu.displayRollPitchOmegaXYZ(1);
		cout << "\n";
		// use Felipe's kalman filter here
		
		
	/* Calculate Control Effort */
		volt2 = 12.0*(pos2 - 1632)/1632; // stall voltage is 1V
		
	
	/* Output Pwm */
		motor1.runByVoltage(volt1);	//P9_14, 
		motor2.runByVoltage(volt2); //P9_16,             P8_19, P8_13

	

	}
	
	return 0;
}

