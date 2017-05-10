/* Main Loop for BalanceBotBlack */
/* Encoders */
	#include "eqep.h"
	#include "util.h"
	#include <cstdlib>
/* IMU */
	#include "MPU9250.h"
	#include <iostream>
	#include <unistd.h>
	#include <cmath>
	#include <stdio.h>
	#include <iomanip>
/* PWM */
	#include "PWM.h"
/* Motor Driver */
	#include "MotorController.h"
	#include "GPIO.h"
	#include <string>
/* Timing */
	#include <chrono>
	#include <fstream>
	#include <sys/time.h>
/* Matrix Math */
	#include <Eigen/Dense>
	
	
	using namespace ZJ;
	using namespace std;

/* Timing */
    typedef std::chrono::high_resolution_clock Time;
    typedef std::chrono::nanoseconds ns;
    typedef std::chrono::duration<float> fsec;
	
int main(){
// Initialize everything outside the loop
	cout << "Start\n";
	
/* Initialize the IMU */
	MPU9250 imu(2, 0x68);
    imu.setAccRange(MPU9250::PLUSMINUS_16_G);
    imu.setGyroRange(MPU9250::PLUSMINUS_2000_DPS);
	
/* Loop Variables */
	bool killSwitch = 1;
	unsigned long i = 0;
	
/* Encoders */
	EQEP encoder1(0);	//P8_35, P8_33
	int pos1 = 0;
	EQEP encoder2(1);	//P8_12, P8_11
	int pos2 = 0;
	
/* PWM */
	PWM motor1("pwm1a"); //P9_14
	PWM motor2("pwm1b"); //P9_16
	GPIO gip1(30); //P9_11
	GPIO gip2(31); //P9_13
	
/* Control Variables */
	float volt1 = 0.0;
	float volt2 = 0.0;
	float e1 = 0.0;
	float pos_d = 0.0;
	float e_pos = 0.0;
	float torque_to_percent = 0.0;
	float count_to_radians = 2*PI/1632;
	float R = 0.02; // meters
	float control1 = 0.0;
	float control2 = 0.0;
	
/* H-Control Matrices */
	MatrixXd A(3,3); // use MatrixXf for arbitrary size
	A << 0, 0, 0,
		 0, 0, 0,
		 0, 0, 0;
		 
	MatrixXf B(3,2);
	B << 0, 0,
		 0, 0,
		 0, 0;
		 
	MatrixXf C(1,3);
	C << 0, 0, 0;
	
	MatrixXf D(1,2);
	D << 0, 0;
	
	Vector2d q(0,0); // column vectors by default
	Vector2d u(0,0);	
	Vector2d y(0,0);
	
///* THE LOOP OF ETERNAL BALANCE *///
	while (killSwitch){
		
	/* Read Encoders */
		pos1 = encoder1.getPosition();
		pos2 = encoder2.getPosition();
		
		
	/* Read IMU */
		imu.readSensor();
		/* use Felipe's kalman filter here */
		
		
	/* Calculate Control Effort */
		e1 = imu.getAccPitch(); // stall voltage is 1V
		control1 = 92*(1/65.0)*e1 + 8; //percent duty cycle
		control2 = 92*(1/65.0)*e1 + 8;
	
	
	/* Calculate H-Control Effort */
		e1 = imu.getAccPitch()*PI/180;
		e_pos = (pos1 - pos_d)*count_to_radians*R;
		u(0) = e1;
		u(1) = e_pos;
		
		y = C*q + D*u;	//   y[k] = C*q[k] + D*u
		q = A*q + B*u;	// q[k+1] = A*q[k] + B*u
		
		control1 = y(0)*torque_to_percent + 8;
		control2 = y(1)*torque_to_percent + 8;
	
		/* Prevent Saturation */
			if (abs(e1) >= 60){
				killSwitch = 0;
				control1 = 0;
				control2 = 0;
			}
		
		
	/* Output Pwm */
		if (e1 >= 0){
			gip1.setValue(GPIO::LOW);
			gip2.setValue(GPIO::LOW);		
			
			motor1.setDutyCyclePercentage(control1);
			motor2.setDutyCyclePercentage(control2);
		}
		else {
			gip1.setValue(GPIO::HIGH);
			gip2.setValue(GPIO::HIGH);
			
			motor1.setDutyCyclePercentage(100 - abs(control1));
			motor2.setDutyCyclePercentage(100 - abs(control2));
		}
		
		// cout << imu.getAccPitch() << "	" << control1 << "	" << control2 << "	" << e1<<"\n";
	
	 }

////////////////////* Chrono Sample Code *////////////////////
    // auto t0 = Time::now();
    // auto t1 = Time::now();
    // fsec fs = t1 - t0;
    // ns d = std::chrono::duration_cast<ns>(fs);
    // std::cout << fs.count() << " s\n";
    // std::cout << d.count() << " ms\n";
    // std::cout << MotorEnCnt/d.count() << " ms\n";
//////////////////////////////////////////////////////////////


////////////////* Getting Motor Parameters *//////////////////
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
//////////////////////////////////////////////////////////////
	
	return 0;
}
