#pragma once
#include "VehicleControl.h"


using namespace std;

class Controller {
protected: 
	double dt;            // Sample time
	double Kp;            // Parameter of controller
	double Ki;
	double Kd;
	double error_pre;     // error of previous step
	double error_pre2;    // error of previous 2 step
	double integral;      // integral part
	double output;        // output of controller
	double desrValueMin;  // minimal desired value
	double desrValueMax;  // maximal desired value
	double outputMin;     // minimal output value
	double outputMax;     // maximal output value
public:
	Controller(double, double, double, double);
	Controller(double);
	void setDesrValueLimit(double, double);
	void setOutputLimit(double, double);
};


class SpeedController:public Controller{
public:
	SpeedController(double, double, double, double);
	SpeedController(double);
	void control_calc(double, double);
};


class LaneKeepingController:public Controller{
public:
	LaneKeepingController(double, double, double, double);
	LaneKeepingController(double);
	void control_calc(double, double, double, double);

};

class DistanceController:public Controller{
public:
	DistanceController(double, double, double, double);
	DistanceController(double);
	double control_calc(double,double);     
};   

