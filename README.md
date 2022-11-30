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
* Robot Raconteur
* Python3
* tormach RR Driver
* ATI FT Sensor Driver (included)

## Python Packages:
* general-robotics-toolbox
* qpsolvers
* BeatifulSoup4
* Numpy

# Workspace Calibration
![](demos/01_ws_calib_manualCompliance_and_touchOff.gif)
