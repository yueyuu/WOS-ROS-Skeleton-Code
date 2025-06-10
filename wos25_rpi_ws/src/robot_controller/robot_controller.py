#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import UInt8
from std_msgs.msg import Int8

from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo
from gpiozero import AngularServo
import RPi.GPIO as GPIO

import time

from DFRobot_RaspberryPi_DC_Motor import DFRobot_DC_Motor_IIC as Board


# use command 'pinout' in terminal to see the pinout diagram for the rpi
# pins
SHOOTER_FIRE_PIN = 18
SHOOTER_PITCH_PIN = 17

# constants
FIRE = 1
MIN_ANGLE = -90
MAX_ANGLE = 90
FIRE_ROTATE_SPEED = -1 
FIRE_REVERSE_SPEED = 1 

# set up shooter
# gpiozero uses BCM pinout; cannot be changed
factory = PiGPIOFactory() # to prevent servo jitter
fire_servo = Servo(SHOOTER_FIRE_PIN, pin_factory=factory)
pitch_servo = AngularServo(SHOOTER_PITCH_PIN, initial_angle=0, min_angle=MIN_ANGLE, max_angle=MAX_ANGLE, pin_factory=factory)

# set up motor driver hat
motor_driver_hat = Board(1, 0x10) # RaspberryPi select bus 1, set address to 0x10


#========================================================================================

def pwmCallback(msg):
    # read ros msg
    left,right = msg.data.split(",")
    left_dc = float(left)*100
    right_dc = float(right)*100
    rospy.loginfo('Received duty cycle: %f, %f', left_dc, right_dc)

    # set left wheel duty cycle
    if left_dc >= 0:
        left_dir = motor_driver_hat.CCW
    else:
        left_dir = motor_driver_hat.CW
    motor_driver_hat.motor_movement([motor_driver_hat.M1], left_dir, abs(left_dc))

    # set right wheel duty cycle
    if right_dc >= 0:
        right_dir = motor_driver_hat.CCW
    else:
        right_dir = motor_driver_hat.CW
    motor_driver_hat.motor_movement([motor_driver_hat.M2], right_dir, abs(right_dc))

def shooterFireCommandCallback(msg):
	fire_command = msg.data
	if fire_command == FIRE:
		rospy.loginfo("Firing shooter!")
		fire_servo.value = FIRE_ROTATE_SPEED
		time.sleep(0.75)
		fire_servo.value = FIRE_REVERSE_SPEED
		time.sleep(0.25)
	else:
		rospy.loginfo("Invalid shooter fire command! Not firing shooter...")
	
def shooterPitchCommandCallback(msg):
	pitch_command = msg.data
	pitch_command = max(min(pitch_command, MAX_ANGLE), MIN_ANGLE) # to restrict the angle to be between the min and max angle allowed
	rospy.loginfo("Pitching shooter to angle %d degrees!", pitch_command)
	pitch_servo.angle = pitch_command

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
	fire_servo.value = 0 
	pitch_servo.angle = 0
	print("Stopping motors and resetting servos to init position on shutdown!")


def main():
	rospy.init_node('robot_controller_node', anonymous=True)
	rospy.on_shutdown(stopMotors)
	
	# for motor driver hat
	while motor_driver_hat.begin() != motor_driver_hat.STA_OK:    # Board begin and check board status
		printBoardStatus()
		rospy.loginfo("Motor driver hat: board begin failed")
		time.sleep(2)
	rospy.loginfo("Motor driver hat: board begin success")
	fire_servo.value = 0
	pitch_servo.angle = 0
	motor_driver_hat.set_motor_pwm_frequency(1000)   # Set DC motor pwm frequency to 1kHz

	rospy.Subscriber('duty_cycle', String, pwmCallback)
	rospy.Subscriber('shooter_fire_driver_command', UInt8, shooterFireCommandCallback)
	rospy.Subscriber('shooter_pitch_driver_command', Int8, shooterPitchCommandCallback)
	rospy.loginfo("Started robot controller node!")

	rospy.spin()
	
		

if __name__ == '__main__':
	main()