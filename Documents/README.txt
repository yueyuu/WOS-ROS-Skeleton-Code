					RUNNING THE CODE

=================== Notes to read before running ===================
- run the robot_controller code on the RPi first as that launches the ROS Master which is used by both the laptop and RPi ROS nodes
- build both workspaces before sourcing and running the code

=================== wos23_rpi_ws ===================

This ROS workspace contains code to be run on the Rpi. It contains the robot_controller_node, motor_driver code and rplidar_ros pkg.

robot_controller_node: 
	subscribes to the keyboard command from the laptop and publishes the desired duty cycle
motor_driver: 
	subscribes to the desired duty cycle and outputs PWM signal on the respective RPi pins
rplidar_ros: 
	default rplidar ROS package that reads the raw lidar data and publishes the data out on a ROS topic


[terminal 1]
> cd ~/WOS23-ROS/wos23_rpi_ws 
> source devel/setup.bash
> roslaunch robot_controller robot_controller.launch

[terminal 2]
> cd ~/WOS23-ROS/wos23_rpi_ws/src/robot_controller/src
> python3 motor_driver.py

[terminal 3]
> cd ~/WOS23-ROS/wos23_rpi_ws
> source devel/setup.bash
> roslaunch rplidar_ros rplidar.launch

=================== wos23_laptop_ws ===================

This ROS workspace contains code to be run on the command laptop. It contains the teleop_twist_keyboard pkg and Rviz display config file.

[terminal 1]
> cd ~/WOS23-ROS/wos23_laptop_ws
> source devel/setup.bash
> roslaunch teleop_rviz teleop_rviz.launch

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
