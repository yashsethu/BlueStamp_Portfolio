import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

MOTOR1B=15 # LEFT motor
MOTOR1E=16
  
MOTOR2B=11 # RIGHT motor
MOTOR2E=13

GPIO.setup(MOTOR1B, GPIO.OUT)
GPIO.setup(MOTOR1E, GPIO.OUT)
GPIO.setup(MOTOR2B, GPIO.OUT)
GPIO.setup(MOTOR2E, GPIO.OUT)

while(True):
	userInput = input()
	
	if(userInput == 'w'):
		GPIO.output(MOTOR1B,GPIO.LOW)
		GPIO.output(MOTOR1E,GPIO.HIGH)
		GPIO.output(MOTOR2B,GPIO.HIGH)
		GPIO.output(MOTOR2E,GPIO.LOW)

	if(userInput == 'a'):
		GPIO.output(MOTOR1B,GPIO.HIGH)
		GPIO.output(MOTOR1E,GPIO.LOW)
		GPIO.output(MOTOR2B,GPIO.HIGH)
		GPIO.output(MOTOR2E,GPIO.LOW)
                
	if(userInput == 's'):
		GPIO.output(MOTOR1B,GPIO.HIGH)
		GPIO.output(MOTOR1E,GPIO.LOW)
		GPIO.output(MOTOR2B,GPIO.LOW)
		GPIO.output(MOTOR2E,GPIO.HIGH)
            
	if(userInput == 'd'):
		GPIO.output(MOTOR1B,GPIO.LOW)
		GPIO.output(MOTOR1E,GPIO.HIGH)
		GPIO.output(MOTOR2B,GPIO.LOW)
		GPIO.output(MOTOR2E,GPIO.HIGH)
        
	if(userInput == 'x'):
		GPIO.output(MOTOR1B,GPIO.LOW)
		GPIO.output(MOTOR1E,GPIO.LOW)
		GPIO.output(MOTOR2B,GPIO.LOW)
		GPIO.output(MOTOR2E,GPIO.LOW)
		
	if(userInput == 'end'):
		GPIO.cleanup()
		break
