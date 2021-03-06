# Highway-Pilot
---
This is a Highway-Pilot driver-assistance system built with C++ language in Laboratory of Software Engineering in year 2021. (For more information, please visit https://www.itiv.kit.edu/)

In this project, the sensordate (including IMU, Radar, Object-Sensor, Road-Sensor, Tire-Sensor) are used for lateral and longitudinal control of the autonomous vehicle. The structure of the main system is illustrated in /Highway-Pilot/Classdisgram.PNG. 

The following functions are implemented:
- Automated speed limit at curvature drive w.r.t. the road curvature
- Adaptive Cruise Control (ACC mode)
- Longitudinal speed control (free drive mode)
- For Emergency: The ego vehicle should change lane left/right or brake immediately, if there are vehicles, humans or any other obstacles ahead too close to the ego vehicle. (obstacle avoid mode)
- Lane keeping by following the road line
- Lane change triggered with Direct Variable Access (DVA) of CarMaker
- Lane change left/right is permited, only if there is a corresponding gap in left/right lane
- Overtake: Entire overtake process consists of lane change left, overtake beside another vehicle and lane change right

CarMaker, as an advanced vehicle simulation, integration and test platform  (please visit https://ipg-automotive.com/), was used to implement the virtual test scenarios (stored in filepath /Highway-Pilot/Data/TestRun/) and to verify the functionalities of this Highway-Pilot system.
