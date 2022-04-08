# Highway-Pilot
---
This is a easy Highway-Pilot system (highway driver-assistance system) I finished in Laboratory of Software Engineering. (For more information, please visit https://www.itiv.kit.edu/60_6075.php)

In this project, the sensordate (inclusive IMU, Radar, Object-Sensor, Road-Sensor) are processed und used for lateral and longitudinal control of vehicles. The following functionalities are realized:
- Set desired speed and speed limit of vehicles
- Speed limit at curvature drive
- Adaptive Cruise Control (ACC mode)
- longitudinal speed control (free drive mode)
- For Emergency: The ego vehicle should change lane left/right or brake quickly, if there is any vehicle ahead too close to the ego vehicle. (obstacle avoid mode)
- Lane change triggered with Direct Variable Access (DVA)
- Lane keeping
- Lane change left/right is permited, only if there is a corresponding gap in left/right lane                                  
- Overtake: entire overtake process consists of lane change left, overtake beside the vehicle and lane change right

CarMaker, as a advanced vehicle simulation environment (please see https://ipg-automotive.com/) would be used to implement virtual test scenarios and to verify the functionalities of this Highway-Pilot system.
