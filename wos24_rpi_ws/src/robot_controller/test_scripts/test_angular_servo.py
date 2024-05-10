from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import AngularServo
from time import sleep 

GPIO_PIN=17

factory = PiGPIOFactory()
servo = AngularServo(GPIO_PIN, min_angle=-90, max_angle=90, pin_factory=factory)

while True:
	servo.angle = 0
	print("mid")
	sleep(1)
	servo.angle = -90
	print("min")
	sleep(1)
	servo.angle = 0
	print("mid")
	sleep(1)
	servo.angle = 90
	print("max")
	sleep(1)
