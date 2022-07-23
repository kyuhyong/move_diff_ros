# move_r1mini for ROS-foxy

For **ROS-melodic** user, checkout to melodic branch.  

**move_r1mini** is very simple position controller for R1mini mobile platform.

This node will do followings

- Open a service to receive a service call to "/set_goal"
- Open an action to "/go_to" to set target odom and gives feedback
- Diff wheel controller will try to reach new goal position and angle requested
- Publish Twist message via "/cmd_vel" topic

This package is tested for R1mini platform.


# Installation

**tf-transformations** package is required and can be installed by
```
$ sudo apt install ros-<distro>-tf-transformations
```

Clone this packae to /src under a ros2 workspace
```
cd to ros2_ws/src
git clone https://github.com/kyuhyong/move_r1mini.git
cd ..
colcon build --symlink-install
```

# Usage : Service call method

To move R1mini robot to X=1.0, Y=1.0 and goal yaw_z = 0.0
Request a servcice call to set goal position and angle by entering below command.
```
$ ros2 service call /set_goal move_commander_interface/srv/SetGoal "{'x':1.0,'y':1.0,'theta':0.0}"`
```
# Usage : Action call method

Action in ros 2 gives feedback to the client.
Request an action to set goal position and angle by entering below command.

```
$ ros2 action send_goal --feedback go_to move_commander_interface/action/GoTargetOdom "{'x':0.5,'y':0.0,'theta':0.0}"
```


# To do

Move controller is based on pure pursuit algorithm and is quite jittery. 
May be improved by applying PID controllers

