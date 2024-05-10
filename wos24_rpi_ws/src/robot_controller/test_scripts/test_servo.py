from gpiozero import Servo
from time import sleep 

GPIO_PIN=17

mycorrection=0.2 # need to to tune this val
maxPW=(2.0+mycorrection)/1000
minPW=(1.0-mycorrection)/1000

servo = Servo(GPIO_PIN, min_pulse_width=minPW, max_pulse_width=maxPW)
#servo = Servo(GPIO_PIN)

while True:
	#servo.mid()
	#print("mid")
	#sleep(1)
	servo.min()
	print("min")
	sleep(1)
	#servo.mid()
	#print("mid")
	#sleep(1)
	servo.max()
	print("max")
	sleep(1)
