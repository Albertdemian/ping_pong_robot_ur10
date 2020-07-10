# ping pong robot UR10
This is the project dedicated to the ping-pong playing robot manipulator UR10. In the repository you could find the computer aided design, computer vision and control parts.

### The machine configuration the code was tested on:
* OS: Ubuntu 18.04.4
* ROS: Melodic


### Clone the repository into your ROS catkin workspace:
```bash
cd catkin_ws/src
```

```bash
git clone https://github.com/Albertdemian/ping_pong_robot_ur10.git
```

```bash
cd .. && catkin_make
```

### Before you start working with the code, you should at first download dependencies:

```bash
./src/ping_pong_robot_ur10/download.sh
```
###### It needs sudo privileges 

### To play a little with the Robot and ROS, you can start our simple interface. 
This interface controls the robot in both task and cartesian spaces. 

![Alt text](images/interface.png?raw=True?scale=0.5 "Interface")

#### To launch interface: 
* source your terminal 
* launch interface: 
```bash
roslaunch interface control.launch
```

* wait for 3 seconds for interface to synchronize with real robot configuration 

* Make sure that only one "activate" botton is checked
* You can play with sliders and then check "execute" to see the robot moving to desired position 
### How to start the project?

* You need to source all terminals
* Setup wired connection with robot through network settings 
* Connect RealSense camera with usb and setup field of view
#### #1 To launch roscore and feedback node: 
```bash
roslaunch  ur10_control control_w_ball.launch 
```

#### #2 To launch vision node: 
```bash
rosrun computer_vision main.py
```
#### #3 To launch robot control node: 
```bash
rosrun ur10_control catch_ball.py
```

