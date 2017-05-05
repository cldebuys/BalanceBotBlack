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
	#include "MotorController.h"
	#include "GPIO.h"
	//#include "PWM.h"
	//#include <cstdlib>
	#include <string>
	//#include <iostream>
/* Timing */
	#include <chrono>
	#include <fstream>
	#include <sys/time.h>

using namespace ZJ;
using namespace std;

// typedef std::chrono::high_resolution_clock Clock;
// typedef std::chrono::nanoseconds ns;
// typedef std::chrono::duration<float> fsec;
    typedef std::chrono::high_resolution_clock Time;
    typedef std::chrono::nanoseconds ns;
    typedef std::chrono::duration<float> fsec;
	
int main(){
	cout << "Start\n";
	/* Initialize the IMU */
	MPU9250 imu(2, 0x68);
    imu.setAccRange(MPU9250::PLUSMINUS_16_G);
    imu.setGyroRange(MPU9250::PLUSMINUS_2000_DPS);
	
	
	/* Initialize variables outside the loop */
	int killSwitch = 1;
	
	EQEP encoder1(0);	//P8_35, P8_33
	int pos1 = 0;
	EQEP encoder2(1);	//P8_12, P8_11
		// GPIO: write failed to open file : Permission denied
	int pos2 = 0;
	
	
	MotorController motor(4,5); //P9_18, P9_17
	float volt1 = 0.0;
    // L298N motor2("pwm1",30,31); //P9_11, P9_13
	float volt2 = 0.0;
	
	// while (killSwitch){
	// /* Read Encoders */
		// pos1 = encoder1.getPosition();
		// pos2 = encoder2.getPosition();
		
		
	// /* Read IMU */
		// imu.readSensor();
		// cout << imu.getAccPitch() << "\n";
		// cout << "\n";
		// use Felipe's kalman filter here
		
		
	// /* Calculate Control Effort */
		// volt2 = 12.0*(pos2 - 1632)/1632; // stall voltage is 1V
		
	
	// /* Output Pwm */
		// motor.runByVoltage(volt1);	//P9_14, 
	//	// motor2.runByVoltage(volt2); //P9_16,             P8_19, P8_13

	

	// }

    	
    // auto t0 = Time::now();
    // auto t1 = Time::now();
    // fsec fs = t1 - t0;
    // ns d = std::chrono::duration_cast<ns>(fs);
    // std::cout << fs.count() << " s\n";
    // std::cout << d.count() << " ms\n";
    // std::cout << MotorEnCnt/d.count() << " ms\n";
///////////////////////////////////////////////////////////////

	/* Getting Motor Parameters */
	ofstream out_data;
	out_data.open("encoder_rate.dat");
	motor.runByVoltage(12.0);
	int iter = 0;
    auto t1 = Time::now();
	int pos_now = 0;
	double encoder_rate = 0;
	usleep(10000000);
	motor.runByVoltage(0.0);
	int pos_prev = encoder2.getPosition();
    auto t0 = Time::now();
	auto current_time = Time::now();
	auto start_time = Time::now();
	
	while (iter < 1000){
		
		pos_now = encoder2.getPosition();
		
		t1 = Time::now();
		fsec fs = t1 - t0;
		ns d = std::chrono::duration_cast<ns>(fs);
		
	// std::cout << fs.count() << " s\n";
    // std::cout << d.count() << " ms\n";
    // std::cout << MotorEnCnt/d.count() << " ms\n";
	
	// /*	encoder_rate = (float)(pos_now - pos_prev) / (float)(chrono::duration_cast<chrono::nanoseconds>(t2 - t1).count());*/

		encoder_rate = (double)1000*(pos_now - pos_prev)/d.count(); // counts per second
		
		out_data << fs.count() <<"	" << encoder_rate << "\n";
		cout << encoder_rate << "	" << pos_now << "	" << pos_prev <<"\n";
		t0 = t1;
		if (pos_prev == pos_now){
			++iter;
		}
		else{
			iter = 0;
		}
		pos_prev = pos_now;
		
	}
	out_data.close();
	return 0;
}

