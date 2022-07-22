# move_r1mini 

move_r1mini is very simple position controller for R1mini mobile platform. 

This node will do followings
- Open a service to receive a service call to "/set_goal"
- Diff wheel controller will try to reach new goal position and angle requested
- Publish Twist message via "/cmd_vel" topic

This package is tested for R1mini platform and may also work for R1D2 as well.

# Installation

```
cd to catkin_ws/src
git clone https://github.com/kyuhyong/move_r1mini.git -b melodic
cd ..
catkin_make
```

# Usage

Request a servcice call to set goal position and angle by entering below command
- 'x', 'y' : Position relative to odom.position x, y
- 'theta' : Target orientation yaw angle when reached to the target.

```
$ rosservice call /set_goal "{'x':1.0, 'y':1.0, 'theta':0.0}"
```

# To do

Add command to set maximum Vx, Vw
