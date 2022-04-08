#include "HighwayPilotSystem.h"

HighwayPilotSystem::HighwayPilotSystem(double dt) : desrSpd(130), SpdMin(60), SpdMax(130), Timegap(2.5), SafeDistance(40), spdcontroller(dt), distance_controller(dt), LKcontroller(dt) { // Default: desrSpd = 130 km/h, SpdMin = 60 km/h, SpdMax = 130 km/h, Timegap = 2.5s
    this->dt = dt;
    spdcontroller.setDesrValueLimit(SpdMin, SpdMax);
    distance_controller.setOutputLimit(SpdMin, SpdMax);
}


void HighwayPilotSystem::setDesrSpdAndLimit(double desrSpd, double SpdMin, double SpdMax) {
    this->desrSpd = desrSpd;
    this->SpdMin = SpdMin;
    this->SpdMax = SpdMax;
}



void HighwayPilotSystem::curveDriveSpdLimit(){
    // Assignment 9: Speed limit at curvature drive

    RoadSensor->PreviewDist = 0;                                                         // Set the preview distance to a bigger value if needed:  RoadSensor->PreviewDist = 25;
                                    
    double frictionCoeffi = 1;                                                           //Friction coefficent between 0~1, equal to 1 if the road is dry:  double frictionCoeffi = Car.Tire[0].muRoad;   
    double roadCurvatureXY = RoadSensor->Path.CurveXY;
    curvatureRadius = abs(1 / roadCurvatureXY);
    double decelerateFactor = 0.5;                                                       // We decelerate the desired Speed slower than the theoretical calculated maximal speed at curvature drive in order to ensure that the maximal value of lateral acceleration -4 ~ 4 m/s^2 is not exceeded at curvature drive
    double curveDriveSpdMax = sqrt(frictionCoeffi * curvatureRadius * gravityAccel) * 3600 / 1000 * decelerateFactor;                // Maximal speed in km/h by curvature drive
    
    if (curveDriveSpdMax < SpdMax) {
        spdcontroller.setDesrValueLimit(SpdMin, curveDriveSpdMax);
        distance_controller.setOutputLimit(SpdMin, curveDriveSpdMax);
    }
    else {
        spdcontroller.setDesrValueLimit(SpdMin, SpdMax);
        distance_controller.setOutputLimit(SpdMin, SpdMax);
    }


}



void HighwayPilotSystem::adaptiveCruiseControl() {     
    // Assignment 15: Adaptive Cruise Control (ACC)
    actualSpd = InertialSensor->Vel_B[0] * 3600 / 1000;                                  //Actual speed of ego vehicle in km/h measured by inertial sensor
    double TargetCatchDistance = 150;                                                    //Distance, that the ego vehicle treat the vehicle in front of it as a target to follow                                       
    ObjectSensor->trtype = LaneTracking;                                                 //For Object Sensor we use "LaneTracking" mode

    if (ObjectSensor->relvTarget.dtct == 1 && ObjectSensor->relvTarget.InLane == 1) {
        DistanceToTarget = ObjectSensor->relvTarget.NearPnt.ds[0];                       //We use "LaneTracking" mode, so that the selected relevant target "relvTarget" is the nearest object in path, see "Reference Manual" page 638.
        SpdDeviationToTarget = ObjectSensor->relvTarget.NearPnt.dv[0] * 3600 / 1000;     //The current selected relevant target is visualized with yellow box in IPGMovie
    }

    //Desired distance (default: Timegap = 2.5s) to the target vehicle that the ego vehicle needs to keep.
    //The faster the target vehicle ahead drives, the longer distance to the target vehicle that the ego vehicle should keep, because the tiemgap is constant. 
    double desrDistance = Timegap * (actualSpd + SpdDeviationToTarget) * 1000 / 3600;    

    if (DistanceToTarget > 0 && DistanceToTarget < TargetCatchDistance && ObjectSensor->relvTarget.InLane == 1 && (actualSpd + SpdDeviationToTarget)<SpdMax && (actualSpd + SpdDeviationToTarget) > SpdMin) {                                                               //ACC mode
        // ACC mode
        desrSpdACC = distance_controller.control_calc(desrDistance, DistanceToTarget);
        spdcontroller.control_calc(desrSpdACC, actualSpd);
    }
    else {
        //Assignment 7: longitudinal speed control (free drive mode)
        spdcontroller.control_calc(desrSpd, actualSpd);
    }
}


void HighwayPilotSystem::obstacleAvoid() {
    //This function is for Emergency (obstacle avoid mode): The ego vehicle should change lane left / right or brake quickly, if there is any vehicle ahead too close to the ego vehicle. 
    int nLaneL = RadarSensor->nLanesL;                                                // Observe the number of lanes left/right (integer) using radar sensor, alternaltiv using road sensor or object by lane sensor
    int nLaneR = RadarSensor->nLanesR;
    actualSpd = InertialSensor->Vel_B[0] * 3600 / 1000;                               //Actual speed of ego vehicle in km/h measured by inertial sensor

    ObjectSensor->trtype = LaneTracking;

    if (ObjectSensor->relvTarget.dtct == 1 && ObjectSensor->relvTarget.InLane == 1) {
        if (abs(ObjectSensor->relvTarget.NearPnt.ds[0]) < SafeDistance/2) {
            if (IsLaneChangeAllowed(0)) {                                             
                turnLeft = 1;
            }
            else if (!IsLaneChangeAllowed(0) && IsLaneChangeAllowed(1)) {
                turnRight = 1;
            }
            else {
            }
        }
    }
}


bool HighwayPilotSystem::IsLaneChangeAllowed(int TernLeftRightRequest) {              // The parameter TernLeftRightRequest (0 or 1) is given by function user: 0 for turn left, 1 for turn right, so that the function gives the bool parameter back and tells whether tern left or rigth is permited 
    // Assignment 23: Lane change left/right is permited, only if there is a corresponding gap in left/right lane
    int nLaneL = RadarSensor->nLanesL;                                                // Observe the number of lanes left/right (integer) using radar sensor, alternaltiv using road sensor or object by lane sensor
    int nLaneR = RadarSensor->nLanesR;

    actualSpd = InertialSensor->Vel_B[0] * 3600 / 1000;
    ObjectSensor->trtype = LaneTracking;
   
    if (TernLeftRightRequest == 0 && nLaneL > 0) {                                    // observe the traffic situation on the left lane
        for (int i = 0; i < Traffic.nObjs; i++) {
            tObjectSensorObj* obj = ObjectSensor_GetObject(0, i);
            if (obj != NULL) {
                if (obj->InLane == 0 && obj->RefPnt.ds[1] > 0 && abs(obj->RefPnt.ds[1]) < 1.5* RoadSensor->Act.Width && abs(obj->RefPnt.ds[0]) < SafeDistance) {
                // Condition: target vehicle is not in Lane of ego vehicle && target vehicle is at left site of current lane && target vehicle is in neighbor lane && distance to target vehicle < SafeDistance
                    return false;
                }
            }
        }
        return true;
    }
    else if(TernLeftRightRequest == 1 && nLaneR > 0){                                // observe the traffic situation on the right lane
        for (int i = 0; i < Traffic.nObjs; i++) {
            tObjectSensorObj* obj = ObjectSensor_GetObject(0, i);
            if (obj != NULL) {
                if (obj->InLane == 0 && obj->RefPnt.ds[1] < 0 && abs(obj->RefPnt.ds[1]) < 1.5 * RoadSensor->Act.Width && abs(obj->RefPnt.ds[0]) < SafeDistance) {
                    return false;
                }
            }
        }
        return true;
    }
    else if(TernLeftRightRequest == 0 &&  nLaneL == 0){
        return false;
    }
    else if (TernLeftRightRequest == 1 && nLaneR == 0){
        return false;
    }
    else {
        return true;
    }

}                      

void HighwayPilotSystem::overtake() {
    // Assignment 25: Entire overtake process consists of lane change left, overtake beside the target vehicle and lane change right
    ObjectSensor->trtype = LaneTracking;
    
    if (ObjectSensor->relvTarget.dtct == 1 && ObjectSensor->relvTarget.InLane == 1 && overtakeRequest == 1 && IsLaneChangeAllowed(0)) {
        //If the Object Sensor observes a target vehicle in current Lane and the driver gives a overtake request (overtakeRequest = 1), and lanechange left is permited
        //Then the ego vehicle changes lane left and accelerates to the desired speed 130 km/h to overtake beside the target vehicle.
        turnLeft = 1;
        overtakeRequest = 2;                                            //Set overtakeRequest = 2 after Lanechange left
    }

    if (overtakeRequest == 2) {                                         //If overtakeRequest = 2, that means that the ego vehicle overtakes beside the target vehicle currently and prepares a lanechange right
        for (int i = 0; i < Traffic.nObjs; i++) {
            tObjectSensorObj* obj = ObjectSensor_GetObject(0, i);
            if (obj != NULL) {
                if (obj->RefPnt.ds[1] < -0.5 * RoadSensor->Act.Width && obj->RefPnt.ds[1] > -1.5 * RoadSensor->Act.Width && obj->RefPnt.ds[0] < 0 && obj->RefPnt.ds[0] > -1.5 * SafeDistance && IsLaneChangeAllowed(1)) {
                    // If there is a target vehicle on the neighbor right lane and the ego vehicle drives to the front of the target vehicle, and lanechange right is allowed, then the ego vehicle changes lane right
                    turnRight = 1;
                    overtakeRequest = 0;                               // Set overtakeRequest = 0 after entire overtake process
                }
            }
        }
    }

    if (overtakeRequest == 1 && (ObjectSensor->relvTarget.dtct == 0 || RadarSensor->nLanesL == 0)) {
        // If the driver gives a overtake request (overtakeRequest = 1), while there is no target vehicle ahead to overtake or there is no lane left
        // Then set overtakeRequest = 0, that means that the overtake request is impossible to be executed.
        overtakeRequest = 0;
    }


}


void HighwayPilotSystem::laneChangeTrigger() {
    if (desrRouteDeviationDistance == 0) {                       // get the lateral distance of actual lane mid to the route center line at beginning of simulation
        desrRouteDeviationDistance = RoadSensor->Act.tMidLane;
    }

    if (turnLeft == 1 && IsLaneChangeAllowed(0)) { 
        laneChangeStepsCounter[0] = 0;
    }
    if (laneChangeStepsCounter[0] < laneChangeSteps) {           // Instead of rapid change of desired route deviation distance at lane change, we change the desired route deviation deistance slower and smoother, which will be increased or decreased to the reference value within a number ("laneChangeSteps") of Timesteps 
        desrRouteDeviationDistance = desrRouteDeviationDistance + RoadSensor->Act.Width / laneChangeSteps;
        laneChangeStepsCounter[0]++;
    }

    if (turnRight == 1 && IsLaneChangeAllowed(1)) { 
        laneChangeStepsCounter[1] = 0;
    }
    if (laneChangeStepsCounter[1] < laneChangeSteps) {
        desrRouteDeviationDistance = desrRouteDeviationDistance - RoadSensor->Act.Width / laneChangeSteps;
        laneChangeStepsCounter[1]++;
    }


    turnLeft = 0;                                                // Reset the turnLeft/Right signal after the lane change trigger at once
    turnRight = 0;
}



void HighwayPilotSystem::laneKeeping() {
    routeDeviationDistance = RoadSensor->Route.Deviation.Dist;
    routeDeviationAngle = RoadSensor->Route.Deviation.Ang;
    LKcontroller.control_calc(desrRouteDeviationDistance, routeDeviationDistance, routeDeviationAngle, actualSpd);
}




void myTest(HighwayPilotSystem & myHighwayPilot) {
    myHighwayPilot.curveDriveSpdLimit();
    myHighwayPilot.adaptiveCruiseControl();
    myHighwayPilot.obstacleAvoid();
    myHighwayPilot.overtake();
    myHighwayPilot.laneChangeTrigger();
    myHighwayPilot.laneKeeping();
    
    // Assignment 20: lateral acceleration limit between -4 and 4 m/s^2
    lateralAccel = InertialSensor->Acc_B[1];                           // Output the lateral acceleration of ego vehicle in IPGControl for monitoring
}