#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

import RPi.GPIO as GPIO # for the rpi gpio lib

# Global variables
# Setup GPIO stuff
# TODO: change the following 4 pin numbers according to your wiring 
left_wheel_forward_pin = 13				        # PWM pin connected to left wheel
left_wheel_backward_pin = 15
right_wheel_forward_pin = 16			        # PWM pin connected to right wheel
right_wheel_backward_pin = 18	

GPIO.setwarnings(False)			        # disable warnings
GPIO.setmode(GPIO.BOARD)                # set pin numbering system

GPIO.setup(left_wheel_forward_pin,GPIO.OUT)
GPIO.setup(left_wheel_backward_pin,GPIO.OUT)
GPIO.setup(right_wheel_forward_pin,GPIO.OUT)
GPIO.setup(right_wheel_backward_pin,GPIO.OUT)

# create PWM instance with frequency
left_forward_pwm = GPIO.PWM(left_wheel_forward_pin,100)
left_backward_pwm = GPIO.PWM(left_wheel_backward_pin,100) 
right_forward_pwm = GPIO.PWM(right_wheel_forward_pin,100)
right_backward_pwm = GPIO.PWM(right_wheel_backward_pin,100)

def pwmCallback(msg):
    # read ros msg
    left,right = msg.data.split(",")
    left_dc = float(left)*100
    right_dc = float(right)*100
    rospy.loginfo('I heard: %f, %f', left_dc, right_dc)

    # change the duty cycle accordingly
    if left_dc >= 0:
        left_forward_pwm.ChangeDutyCycle(left_dc)
        left_backward_pwm.ChangeDutyCycle(0)
    else:
        left_forward_pwm.ChangeDutyCycle(0)
        left_backward_pwm.ChangeDutyCycle(-left_dc)
        
    if right_dc >= 0:
        right_forward_pwm.ChangeDutyCycle(right_dc)
        right_backward_pwm.ChangeDutyCycle(0)
    else:
        right_forward_pwm.ChangeDutyCycle(0)
        right_backward_pwm.ChangeDutyCycle(-right_dc)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('motor_driver_node', anonymous=True)
    rospy.Subscriber('/duty_cycle', String, pwmCallback)
    rospy.loginfo("Started PWM listener!")

    
    
    left_forward_pwm.start(0)
    left_backward_pwm.start(0)	
    right_forward_pwm.start(0)
    right_backward_pwm.start(0)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()