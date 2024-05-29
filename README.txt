					RUNNING THE CODE

=================== Notes to read before running ===================
- run the flipper_controller code on the RPi first as that launches the ROS Master which is used by both the laptop and RPi ROS nodes
- build both workspaces before sourcing and running the code (I used catkin build)
- must set (export) the  ROS_MASTER_URI and export ROS_IP in the ~/.bashrc on both the command laptop and RPi
	- ROS_MASTER_URI: use the ip of the RPi
	- ROS_IP: use the ip of the device 

=================== wos24_rpi_ws ===================

This ROS workspace contains code to be run on the Rpi. It contains the locomotion_controller_node, flipper_conroller_node and robot_controller_node.

- locomotion_controller_node: subscribes to the locomotion keyboard command from the laptop and calculates and publishes the desired duty cycle
	- if you want to reduce the minimum speed of the robot, reduce MIN_DUTY_CYCLE (line 14 of locomotion_controller.cpp); MIN_DUTY_CYCLE <= BASE_DUTY_CYCLE

- flipper_controller_node:subscribes to the flipper keyboard command (flipper movement and mode change) from the laptop and publishes the desired flipper movement 

- robot_controller_node: subscribes to the desired duty cycle and flipper movement command and outputs the correct signals on the respective RPi pins to move the robot


[do only once after turning on the RPi]
> sudo systemctl enable pigpiod
> sudo pigpiod

[terminal 1]
> cd ~/WOS-ROS/wos24_rpi_ws
> source devel/setup.bash
> roslaunch flipper_controller flipper_controller.launch

[terminal 2]
> cd ~/WOS-ROS/wos24_rpi_ws 
> source devel/setup.bash
> roslaunch locomotion_controller locomotion_controller.launch

[terminal 3]
> cd ~/WOS-ROS/wos24_rpi_ws/src/robot_controller
> python3 robot_controller.py


=================== wos24_laptop_ws ===================

This ROS workspace contains code to be run on the command laptop. It contains the teleop_twist_keyboard pkg and flipper_keyboard_node. Take note: you need to click in the correct terminal window when giving the commands.

- teleop_twist_keyboard: reads the keyboard command for direction and speed and deduces the velocity to be sent to the locomotion controller

- flipper_keyboard_node: reads the keyboard command for the flipper mode and movement and forwards that to the flipper controller

[terminal 1]
> cd ~/WOS-ROS/wos24_laptop_ws
> source devel/setup.bash
> roslaunch teleop_twist_keyboard teleop_twist_keyboard.launch

[Keyboard commands for teleop]
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

t : up (+z)
b : down (-z)

anything else : stop

q/z : increase/decrease max speeds by 0.1
w/x : increase/decrease only linear speed by 0.1
e/c : increase/decrease only angular speed by 0.1

CTRL-C to quit



[terminal 2]
> cd ~/WOS-ROS/wos24_laptop_ws
> source devel/setup.bash
> roslaunch flipper_keyboard flipper_keyboard.launch

[Keyboard commands for flipper]
Please choose flipper command:
1. open flipper
2. close flipper
3. manual mode
4. autonomous mode

- press the corresponding number for the command on the keyboard and then press ENTER
