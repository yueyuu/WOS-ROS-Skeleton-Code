#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo
import RPi.GPIO as GPIO

import time

from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board


# use command 'pinout' in terminal to see the pinout diagram for the rpi
FLIPPER_PIN = 17
IR_PIN = 18
FLIPPER_OPEN = 0
FLIPPER_CLOSE = 1

# set up flipper
# gpiozero uses BCM pinout; cannot be changed 
factory = PiGPIOFactory() # to prevent servo jitter
flipper = Servo(FLIPPER_PIN, pin_factory=factory)

# set up motor driver hat
motor_driver_hat = Board(1, 0x10) # RaspberryPi select bus 1, set address to 0x10

# setup GPIO stuff
GPIO.setwarnings(False)	# disable warnings
GPIO.setmode(GPIO.BCM) # just to make sure the pinout is BCM
GPIO.setup(IR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # to set the default state of the pin to be LOW

#========================================================================================

def pwmCallback(msg):
    # read ros msg
    left,right = msg.data.split(",")
    left_dc = float(left)*100
    right_dc = float(right)*100
    rospy.loginfo('Received duty cycle: %f, %f', left_dc, right_dc)

    # set left wheel duty cycle 
    #left_dir = motor_driver_hat.CW if left_dc >= 0 else motor_driver_hat.CCW
    if left_dc >= 0:
        left_dir = motor_driver_hat.CCW
    else:
        left_dir = motor_driver_hat.CW
    motor_driver_hat.motor_movement([motor_driver_hat.M1], left_dir, abs(left_dc))   
    
    # set right wheel duty cycle 
    #right_dir = motor_driver_hat.CW if right_dc >= 0 else motor_driver_hat.CCW
    if right_dc >= 0:
        right_dir = motor_driver_hat.CCW
    else:
        right_dir = motor_driver_hat.CW
    motor_driver_hat.motor_movement([motor_driver_hat.M2], right_dir, abs(right_dc))   


def flipperCommandCallback(msg):
	flipper_command = msg.data
	if flipper_command == FLIPPER_OPEN:
		# turn servo anti-clockwise
		flipper.max() # TODO: depends on how it is mounted **
		rospy.loginfo("Opening flipper!")
	elif flipper_command == FLIPPER_CLOSE:
		# turn servo clockwise
		flipper.min() # TODO: depends on how it is mounted **
		rospy.loginfo("Closing flipper!")
	else:
		rospy.loginfo("Invalid flipper command! Not updating flipper position...")
	

# print last operate status, users can use this variable to determine the result of a function call
def printBoardStatus():
	if motor_driver_hat.last_operate_status == motor_driver_hat.STA_OK:
		rospy.loginfo("board status: everything ok")
	elif motor_driver_hat.last_operate_status == motor_driver_hat.STA_ERR:
		rospy.loginfo("board status: unexpected error")
	elif motor_driver_hat.last_operate_status == motor_driver_hat.STA_ERR_DEVICE_NOT_DETECTED:
		rospy.loginfo("board status: device not detected")
	elif motor_driver_hat.last_operate_status == motor_driver_hat.STA_ERR_PARAMETER:
		rospy.loginfo("board status: parameter error, last operate no effective")
	elif motor_driver_hat.last_operate_status == motor_driver_hat.STA_ERR_SOFT_VERSION:
		rospy.loginfo("board status: unsupport board framware version")


def stopMotors():
	motor_driver_hat.motor_stop(motor_driver_hat.ALL)   # stop all DC motor
	flipper.mid() # need to check again ************
	print("Stopping motors and closing flipper on shutdown!")
    

def main():
	rospy.init_node('robot_controller_node', anonymous=True)
	rospy.on_shutdown(stopMotors)

	# for motor driver hat 
	while motor_driver_hat.begin() != motor_driver_hat.STA_OK:    # Board begin and check board status
		printBoardStatus()
		rospy.loginfo("Motor driver hat: board begin failed")
		time.sleep(2)
	rospy.loginfo("Motor driver hat: board begin success")
	flipper.mid() # set defualt command as close
	motor_driver_hat.set_motor_pwm_frequency(1000)   # Set DC motor pwm frequency to 1kHz

	rospy.Subscriber('duty_cycle', String, pwmCallback)
	rospy.Subscriber('flipper_driver_command', UInt8, flipperCommandCallback)
	IR_pub = rospy.Publisher('IR_state', UInt8, 10)
	rospy.loginfo("Started robot controller node!")

	while not rospy.is_shutdown():
		# repeatedly poll the pin 
		IR_state = GPIO.input(IR_PIN)
		IR_pub.publish(IR_state)
		if IR_state == GPIO.HIGH:
			rospy.loginfo("IR is [ ON ]")
		# do i need a rate for this loop *******************************
	
		

if __name__ == '__main__':
	main()
