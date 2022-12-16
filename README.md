<h1>
  WAAM Tormach 6-DoF Robot Subtractive Manufacturing with Complience Control (on going)
</h1>

Input:
- cartesian coordinates of the 3D object organized in layers, with corresponding normal vectors (optional)

My Work:
- Convert points [x,y,z,roll,pitch,yall] to corresponding robot joint command [q1,q2,q3,q4,q5,q6]
- Examine executability (Singularities, joint limit, workspace limit)
- Implementations and Debugging 

Deliverable 
- Robot Execution demo
- Execution files and documentations of usage

## Software Requirements:
* [Robot Raconteur](https://github.com/robotraconteur/robotraconteur/blob/master/docs/common/installation.md)
* Python3
* [tormach RR Driver (on the Host Computer)](https://github.com/hehonglu123/Tormach_RR_Driver)
* ATI FT Sensor Driver (included)

## Python Packages:
* general-robotics-toolbox
* qpsolvers
* BeatifulSoup4
* Numpy

## Demos
### Workspace Calibration
![](demos/01_ws_calib_manualCompliance_and_touchOff.gif)
- Work part can be mounted arbitrarily below the robot end effector
- Three point is need [origin, x-axis boundary, y-axis boundary]
- run `workspace_calibration.py`

### Trajectory Tracking
#### cosine wave created using numpy
![](demos/02_path_execution_with_a_marker.gif)
- edit cosine wave parameters and run `curve_gen_wave.py`
- generate trajectory using `traj_gen_wave.py`
- edit and run `tormach_machining.py`

#### arbitrary groups of curves created else where
![](demos/03_path_execution_with_a_marker.gif)
- create a new folder in `./data/`
- have each curve in a seperate .csv file
- edit the `data_path` and `file_list` in `__main__` of `tormach_maching.py`

### Milling Application
![](demos/04_engrave_milling.gif)
- cutting depth can be applied separately
- spindle speed and feedrate can both be adjusted based on the force / torque sensor readings
- safe stop system when larger than threshold force detected

## Detailed Operation Procedures
### On Tormach Host Computer:
Open a Terminal and start Tormach PathPilot by entering:
```
launch-pathpilot-launcher -t docker.pathpilot.com/ros_public:noetic-dist-focal-172.7a5bb919
```

while the application is starting, start another Terminal and enter:
```
docker exec -it ros-None-ui bash
```
to run Docker for ROS

Once inside the Docker environment `(ros-dist@172+7a5bb919)root@ros-dist-ui:`, source ROS directory:
```
source /opt/ros/neotic/setup.bash
```
start Tormach Driver:
```
cd Desktop/Tormach_RR_Driver/
python tormach_driver2.py
```
wait for about a minute for the dependencies installation to finish.

### PathPilot and Tormach Robot
- Release the E-stop button
- In PathPilot, click the blinking `RESET` button

### On your computer:
- Connect to the lab wifi `CII1044`, ask a lab member for the password
- Make sure that you meet the software requirment. 
- Enter the project directory `\tormach_za6_subtractive_mftg\`

In a new Command Prompt / Terminal and start the ATI force torque sensor service:
```
python robotraconteur_ati_driver.py
```
(try multiple time if necessary until the servor start successfully)

Mount the part on your workbench, label the origin, x-axis, and y-axis on your part.

Start another Command Prompt / Terminal, start **workspace calibration** by entering:
```
python workspace_calibration.py
```
Follow the instruction displayed.




