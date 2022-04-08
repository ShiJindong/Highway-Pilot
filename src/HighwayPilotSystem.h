/*
**  Praktikum Software Engineering (SS 2021) - Highway Pilot
**  Gruppe 8: Yiwei Miao, Mingqian Ji, Fan Xiao, Jindong Shi
*/

#pragma once

#include <cmath>
#include "User.h"
#include "Vehicle\Sensor_Inertial.h"
#include "Vehicle\Sensor_Road.h"
#include "Vehicle\Sensor_Object.h"
#include "Vehicle\Sensor_Radar.h"
#include "Vehicle\Tire.h"
#include "Controller.h"

class HighwayPilotSystem{
private:
	double dt;                                              // Timestep of simulation
	double desrSpd;                                         // Desired speed in km/h
	double SpdMin;                                          // Speed limit in km/h
	double SpdMax; 
	double Timegap;                                         // Time gap between ego vehicle and target vehicle ahead for ACC mode
	double SafeDistance;                                    // If the Distance to the vehicle ahead or behind on the left/right lane is shorter than the SafeDistance, the ego vehicle is not permited to change the lane left/right. 
	SpeedController spdcontroller;
	DistanceController distance_controller;
	LaneKeepingController LKcontroller;

public:
	HighwayPilotSystem(double);
	void setDesrSpdAndLimit(double, double, double);        // Assignment 8: Set desired speed and speed limit
	void curveDriveSpdLimit();                              // Assignment 9: Speed limit at curvature drive
	void adaptiveCruiseControl();                           // Assignment 15: Adaptive Cruise Control (ACC mode);    Assignment 7: longitudinal speed control (free drive mode)
	void obstacleAvoid();                                   // For Emergency: The ego vehicle should change lane left/right or brake quickly, if there is any vehicle ahead too close to the ego vehicle. (obstacle avoid mode)
	void laneChangeTrigger();                               // Assignment 18, 19, 21: Lane change triggered with Direct Variable Access (DVA)
	void laneKeeping();                                     // Assignment 13: Lane keeping
	bool IsLaneChangeAllowed(int);                          // Assignment 23: Lane change left/right is permited, only if there is a corresponding gap in left/right lane                                       
	void overtake();                                        // Assignment 25: Entire overtake process consists of lane change left, overtake beside the vehicle and lane change right
	
};

void myTest(HighwayPilotSystem &);