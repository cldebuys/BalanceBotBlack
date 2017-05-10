// Main Loop for Balance Bot Black
/* Encoders */
	#include "eqep.h"
	#include "util.h"
	#include <cstdlib>
/* IMU */
	#include "MPU9250.h"
	#include <iostream>
	#include <unistd.h>
	//#include <math.h>
	#include <cmath>
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
	bool killSwitch = 1;
	unsigned long i = 0;
	
	EQEP encoder1(0);	//P8_35, P8_33
	int pos1 = 0;
	EQEP encoder2(1);	//P8_12, P8_11
		// GPIO: write failed to open file : Permission denied
	int pos2 = 0;
	
	PWM motor1("pwm1a"); //P9_14
	PWM motor2("pwm1b"); //P9_16
	GPIO gip1(30); //P9_11
	GPIO gip2(31); //P9_13
	float volt1 = 0.0;
	float volt2 = 0.0;
	float e1 = 0.0;
	float e2 = 0.0;
	float control1 = 0.0;
	float control2 = 0.0;
	
	while (killSwitch){
	/* Read Encoders */
		pos1 = encoder1.getPosition();
		pos2 = encoder2.getPosition();
		
		
	/* Read IMU */
		imu.readSensor();
		// cout << imu.getAccPitch() << "\n";
		// cout << "\n";
		// use Felipe's kalman filter here
		
		
	/* Calculate Control Effort */
		e1 = imu.getAccPitch(); // stall voltage is 1V
		volt1 = 12.0*(e1); // stall voltage is 1V
		volt2 = 12.0*(e1); // stall voltage is 1V
		control1 = 92*(1/65.0)*e1 + 8; //percent duty cycle
		control2 = 92*(1/65.0)*e1 + 8;
			
		if (abs(e1) >= 60){
			killSwitch = 0;
			control1 = 0;
			control2 = 0;
		}
	
	/* Output Pwm */
		if (e1 >= 0){
			gip1.setValue(GPIO::LOW);
			gip2.setValue(GPIO::LOW);		
			
			// if((control1 or control2) < 1/12/100)
			// {
				// control1 = 1/12*100;
				// control2 = 1/12*100;
			// }
			motor1.setDutyCyclePercentage(control1);
			motor2.setDutyCyclePercentage(control2);
		}
		else {
			gip1.setValue(GPIO::HIGH);
			gip2.setValue(GPIO::HIGH);
			
			// if(( abs(control1) or abs(control2)) < 1/12*100) //this is the minimal control effort dude
			// {
				// control1 = 1/12*100;
				// control2 = 1/12*100;
			// }
			
			motor1.setDutyCyclePercentage(100 - abs(control1));
			motor2.setDutyCyclePercentage(100 - abs(control2));
		}
		
		cout << imu.getAccPitch() << "	" << control1 << "	" << control2 << "	" << e1<<"\n";
		// gip1.setValue(GPIO::LOW);
		// gip2.setValue(GPIO::LOW);
		// motor1.setDutyCyclePercentage(control1);
		// motor2.setDutyCyclePercentage(control2);
		// gip1.setValue(GPIO::HIGH);
		// gip2.setValue(GPIO::HIGH);
		// usleep(5000000);
		// gip1.setValue(GPIO::LOW);
		// gip2.setValue(GPIO::LOW);
	
		
	 }

///////////////////////////////////////////////////////////////	
    // auto t0 = Time::now();
    // auto t1 = Time::now();
    // fsec fs = t1 - t0;
    // ns d = std::chrono::duration_cast<ns>(fs);
    // std::cout << fs.count() << " s\n";
    // std::cout << d.count() << " ms\n";
    // std::cout << MotorEnCnt/d.count() << " ms\n";
///////////////////////////////////////////////////////////////

	/* Getting Motor Parameters */
	// ofstream out_data;
	// out_data.open("encoder_rate.dat");
	// motor.runByVoltage(12.0);
	// int iter = 0;
    // auto t1 = Time::now();
	// int pos_now = 0;
	// double encoder_rate = 0;
	// usleep(10000000);
	// motor.runByVoltage(0.0);
	// int pos_prev = encoder2.getPosition();
    // auto t0 = Time::now();
	// auto current_time = Time::now();
	// auto start_time = Time::now();
	
	// while (iter < 1000){
		
		// pos_now = encoder2.getPosition();
		
		// t1 = Time::now();
		// fsec fs = t1 - t0;
		// ns d = std::chrono::duration_cast<ns>(fs);
		
	/* std::cout << fs.count() << " s\n"; */
    /* std::cout << d.count() << " ms\n"; */
    /* std::cout << MotorEnCnt/d.count() << " ms\n"; */
	
	/*	encoder_rate = (float)(pos_now - pos_prev) / (float)(chrono::duration_cast<chrono::nanoseconds>(t2 - t1).count());*/

		// encoder_rate = (double)1000*(pos_now - pos_prev)/d.count(); // counts per second
		
		// out_data << fs.count() <<"	" << encoder_rate << "\n";
		// cout << encoder_rate << "	" << pos_now << "	" << pos_prev <<"\n";
		// t0 = t1;
		// if (pos_prev == pos_now){
			// ++iter;
		// }
		// else{
			// iter = 0;
		// }
		// pos_prev = pos_now;
		
	// }
	// out_data.close();
	//////////////////////////////////////////////////////////
		// cout << "Start\n";
	
	// PWM motor1("pwm1a"); //P9_14
	// PWM motor2("pwm1b"); //P9_16
	// GPIO gip1(30); //P9_11
	// GPIO gip2(31); //P9_13

	// gip1.setValue(GPIO::LOW);
	// gip2.setValue(GPIO::LOW);
	// motor1.setDutyCyclePercentage(50);
	// motor2.setDutyCyclePercentage(50);
	// gip1.setValue(GPIO::HIGH);
	// gip2.setValue(GPIO::HIGH);
	// usleep(5000000);
	// gip1.setValue(GPIO::LOW);
	// gip2.setValue(GPIO::LOW);
		
	return 0;
}