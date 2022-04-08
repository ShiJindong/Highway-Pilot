#include "Controller.h"

void Controller::setDesrValueLimit(double desrValueMin, double desrValueMax) {
	this->desrValueMin = desrValueMin;
	this->desrValueMax = desrValueMax;
}

void Controller::setOutputLimit(double outputMin, double outputMax) {
	this->outputMin = outputMin;
	this->outputMax = outputMax;
}

Controller::Controller(double dt, double Kp, double Ki, double Kd):error_pre(0), error_pre2(0), integral(0), output(0), desrValueMin(0), desrValueMax(1e6), outputMin(0), outputMax(1e6){
	this->dt = dt;
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;
}

Controller::Controller(double dt):Kp(5), Ki(0), Kd(0), error_pre(0), error_pre2(0), integral(0), output(0), desrValueMin(0), desrValueMax(1e6), outputMin(0), outputMax(1e6) {
	this->dt = dt;
}

SpeedController::SpeedController(double dt, double Kp, double Ki, double Kd) :Controller(dt, Kp, Ki, Kd){
}

SpeedController::SpeedController(double dt) : Controller(dt) {
}

void SpeedController::control_calc(double desrSpd, double actualSpd) {
	desrSpd = (desrSpd > desrValueMin ? desrSpd : desrValueMin);                                                                // limit desired value of speed
	desrSpd = (desrSpd < desrValueMax ? desrSpd : desrValueMax);
	double error = desrSpd - actualSpd;
	integral = error * dt;
	double deltaOutput = Kp * (error - error_pre) + Ki * integral + Kd * (error - 2 * error_pre + error_pre2) / dt;             // deltaOutput: incremental output of controller
	error_pre2 = error_pre;
	error_pre = error;
	output += deltaOutput;                                                                                   

	if (output > 0) {
		VehicleControl.Gas = output;
		VehicleControl.Brake = 0;
	}
	else {
		VehicleControl.Gas = 0;
		VehicleControl.Brake = -output * 1e-3;                                                                                  //We give the shrunken output to the brake, so that a too fast brakin can be avoided
	}
	
}

LaneKeepingController::LaneKeepingController(double dt, double Kp, double Ki, double Kd) :Controller(dt, Kp, Ki, Kd) {
}

LaneKeepingController::LaneKeepingController(double dt): Controller(dt, 1, 1, 30) {             
}

void LaneKeepingController::control_calc(double desrRouteDevDist, double routeDevDist, double routeDevAng, double actualSpd) {
	double error = desrRouteDevDist - routeDevDist;
	integral += error * actualSpd * dt;                                                  
	VehicleControl.Steering.Ang = Kp * error + Ki * integral + Kd * (0 - routeDevAng);                                           // (0 - routeDevAng): because desired deviation angle along route is ZERO.   
}


DistanceController::DistanceController(double dt, double Kp, double Ki, double Kd) :Controller(dt, Kp, Ki, Kd) {
}

DistanceController::DistanceController(double dt) : Controller(dt, 3, 0, 0) {     
}

double DistanceController::control_calc(double desrDistance, double actualDistance) {
	double error = actualDistance - desrDistance;                                                                                // Because: double error = (-desrDistance) - (-actualDistance);
	integral = error * dt;
	double deltaOutput = Kp * (error - error_pre) + Ki * integral + Kd * (error - 2 * error_pre + error_pre2) / dt;              // deltaOutput: incremental output of controller
	error_pre2 = error_pre;
	error_pre = error;
	output += deltaOutput;
	output = (output < outputMax ? output : outputMax);
	output = (output > outputMin ? output : outputMin);                                                                          // limited output (desired value of speed)
	return output;                                                                                                               // return desired speed for speed controller
}