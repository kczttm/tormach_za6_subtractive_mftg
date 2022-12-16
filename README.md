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
- try multiple time if necessary until the servor start successfully
- avoid touching the tool or the sensor during starting

Mount the part on your workbench, label the origin, x-axis, and y-axis on your part.

(pic)

### Milling Example
our current part has a width of about 50 mm, we will remove a 50 mm x 50 mm x 3 mm volume from the top of the part.

#### Generate milling tool path in the part frame using python
edit parameters in `curve_gen_wave.py`:
- spindle diameter
- xlen = 50 mm
- ylen = 50 mm
Start a Command Prompt / Terminal, then run the file:
```
python curve_gen_wave.py
```

Then we can calibrate the workspace geometry by:
```
python workspace_calibration.py
```
Follow the instruction displayed. During the calibration, the origin, an arbitrary points on x and on y axis will be collected. The calibration data will only be saved when all three points are recorded. 
At the end of the calibration, the robot path file will be generated automatically as `traj_gen_wave.py` is called in `workspace_calibration.py`

Next, prepare the spindle by turning on the air inlet and adjusting the pressure to between 0.3 and 0.4 MPa. 
(pic)

Turn on the spindle controller. If using the spindle in manual mode, then turn on the spindle now. 
** It is very important to have the air coming out of the spindle during the operation **

Edit the `tormach_machining.py` file to run the following command in `__main__`:
```
proj = tormach_machining()
curve_bf, t_traj = proj.load_traj()
proj.eval_traj(curve_bf, t_traj)
proj.jog_traj(curve_bf, t_traj, depth = 3, spindle = False)
proj.jog_home()
```









