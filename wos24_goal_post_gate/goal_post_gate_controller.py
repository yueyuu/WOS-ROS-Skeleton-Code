from gpiozero.pins.pigpio import PiGPIOFactory
from gpiozero import Servo
import RPi.GPIO as GPIO
from time import sleep 

# constants
GATE_PIN=17
IR_PIN=18
OPEN_DUR = 5 #s
CLOSE_DUR = 5 #s

# set up servo 
factory = PiGPIOFactory()
gate = Servo(GATE_PIN, pin_factory=factory)

# setup GPIO stuff
GPIO.setwarnings(False)	# disable warnings
GPIO.setmode(GPIO.BCM)
GPIO.setup(IR_PIN, GPIO.OUT)

# main loop that alternates between opening and closing gate automatically
while True:
	# close gate
	gate.max()
	GPIO.output(IR_PIN, GPIO.LOW)
	print("Gate state: closed\n")
	sleep(CLOSE_DUR)
	
	# open gate
	gate.min()
	GPIO.output(IR_PIN, GPIO.HIGH)
	print("Gate state: open\n")
	sleep(OPEN_DUR)
	
