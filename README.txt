					RUNNING THE CODE

=================== Notes to read before running ===================

- run the shooter_controller code on the RPi first as that launches the ROS Master which is used by both the laptop and RPi ROS nodes
- build both workspaces before sourcing and running the code (I used catkin build)
- must set the ROS_MASTER_URI and ROS_IP in the ~/.bashrc on both the command laptop and RPi:
	- open the bashrc file using sudo nano ~/.bashrc
	- add the following 2 lines to the bottom of the file
		export ROS_MASTER_URI=http://<ip of the RPi>:11311
		export ROS_IP=<ip of the device>

=================== wos25_rpi_ws ===================

This ROS workspace contains code to be run on the Rpi. It contains the locomotion_controller_node, shooter_conroller_node and robot_controller_node.

- locomotion_controller_node: subscribes to the locomotion keyboard command from the laptop and calculates and publishes the desired duty cycle
	- if you want to reduce the minimum speed of the robot, reduce MIN_DUTY_CYCLE (line 14 of locomotion_controller.cpp); MIN_DUTY_CYCLE <= BASE_DUTY_CYCLE

- shooter_controller_node: subscribes to the shooter keyboard command (shooter fire and pitch movement) from the laptop and publishes the desired shooter movement on 2 separate topics (1 for fire and 1 for pitch)

- robot_controller_node: subscribes to the desired duty cycle and shooter fire and pitch command and outputs the correct signals on the respective RPi pins to move the robot and shooter


[do only once after turning on the RPi]
> sudo systemctl enable pigpiod
> sudo pigpiod

[terminal 1]
> cd ~/WOS-ROS/wos25_rpi_ws
> source devel/setup.bash
> roslaunch shooter_controller shooter_controller.launch

[terminal 2]
> cd ~/WOS-ROS/wos25_rpi_ws 
> source devel/setup.bash
> roslaunch locomotion_controller locomotion_controller.launch

[terminal 3]
> cd ~/WOS-ROS/wos25_rpi_ws/src/robot_controller
> python3 robot_controller.py


=================== wos25_laptop_ws ===================

This ROS workspace contains code to be run on the command laptop. It contains the teleop_twist_keyboard pkg and shooter_keyboard_node. Take note: you need to click in the correct terminal window when giving the commands.

- teleop_twist_keyboard: reads the keyboard command for direction and speed and deduces the velocity to be sent to the locomotion controller

- shooter_keyboard_node: reads the keyboard command for the shooter fire and pitch movement and forwards that to the shooter controller node on the rpi

[terminal 1]
> cd ~/WOS-ROS/wos25_laptop_ws
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
> cd ~/WOS-ROS/wos25_laptop_ws
> source devel/setup.bash
> roslaunch shooter_keyboard shooter_keyboard.launch

[Keyboard commands for shooter]
Please choose shooter command:
Shoot -------[ 1 ]
Pitch down --[ 9 ]
Pitch up ----[ 0 ]

- press the corresponding number for the command on the keyboard and then press ENTER
