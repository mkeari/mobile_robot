# 2D Mobile Robot: Goal following with obstacle avoidance
<p align="center">
<img src="/misc/mobile_robot_gif3.gif" width="400" height="350"/>
</p>

## Installation
In order to run the code, 
1) Install the required packages:
* matplotlib
* numpy
2) Run the main script file
```
python main.py
```

## Configuration
In the beginning of the main.py, the user can modify some of the environment settings:
```
p_initial = [12, 4]         #Initial position of the robot
theta_initial = 3           #Initial orientation of the robot

R_initial = 0.2             #Radius of robot wheels
L_initial = 1.5             #Length of robot base
dT_initial = 0.05           #delta Time (the framing frequency for plotting)

robot_speed = 5             #Base speed of the robot
prox_sensor_radius = 0.5    #Radius of proximity sensor domain

p_light = [18, 18]          #Position of light source

obstacle_radius = 1.5       #Radius of the obstacles
p_obstacles = [[6, 7],      # Centre coordinates of the obstacles
               [5, 16],
               [14, 8],
               [18, 3]]
```

## Additional example animations:
<p align="center">
<img src="/misc/mobile_robot_gif1.gif" width="400" height="350"/>
<img src="/misc/mobile_robot_gif2.gif" width="400" height="350"/>
</p>
