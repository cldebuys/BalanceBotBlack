// Created by: Zongyao Jin on 4/24/2017

#include "MotorController.h"
#include "VfsPWM.h"
#include <cstdlib>
#include <string>
#include <iostream>

MotorController::MotorController() {

	pwmControl = new VfsPWM();
	// std::cout << "Period of PWM: " << pwmControl->getPeriod()  << " ns" << std::endl;
	// std::cout << "after set direction" << std::endl;

	this->release();
}

void MotorController::release() {
	pwmControl->toggle_pwms(FORWARD_PWM,TURN_OFF);
	pwmControl->toggle_pwms(BACKWARD_PWM,TURN_OFF);
	// std::cout << "after set in1 value" << std::endl;
}

void MotorController::setCW() {
//not used
}


void MotorController::setCCW() {
//not used
}

void MotorController::runByVoltage(float voltage) {
	if (voltage >= 0.0f) //move forward
	{
		pwmControl->toggle_pwms(BACKWARD_PWM, TURN_OFF);
		
		pwmControl->set_duty_fraction(FORWARD_PWM, voltage/MAX_VOLTAGE);
		pwmControl->toggle_pwms(FORWARD_PWM, TURN_ON);
	} 
	else //move backward
	{
		pwmControl->toggle_pwms(FORWARD_PWM, TURN_OFF);
		
		pwmControl->set_duty_fraction(BACKWARD_PWM, voltage/MAX_VOLTAGE);
		pwmControl->toggle_pwms(BACKWARD_PWM, TURN_ON);
	}
}

MotorController::~MotorController() 
{
	delete pwmControl;
}

