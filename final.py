# Import motor and camera setup from our files
from motor import forward, reverse, leftturn, rightturn, stop, sharp_left, sharp_right, back_left, back_right
from ball import find_color_mask, find_largest_contour
from time import sleep
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)

# Setup ultrasonic sensor
Trigger_C = 22
Echo_C = 18
Trigger_L = 31
Echo_L = 29
Trigger_R = 33
Echo_R = 32

GPIO.setup(Trigger_C, GPIO.OUT)  # Trigger 1
GPIO.setup(Echo_C, GPIO.IN)  # Echo 1

GPIO.setup(Trigger_L, GPIO.OUT)  # Trigger 1
GPIO.setup(Echo_L, GPIO.IN)  # Echo 1

GPIO.setup(Trigger_R, GPIO.OUT)  # Trigger 1
GPIO.setup(Echo_R, GPIO.IN)  # Echo 1

GPIO.output(Trigger_C, False)
GPIO.output(Trigger_L, False)
GPIO.output(Trigger_R, False)

def sonar(GPIO_TRIGGER,GPIO_ECHO):
    start=0                     
    stop=0
    # Set pins as output and input
    GPIO.setup(GPIO_TRIGGER,GPIO.OUT)  # Trigger
    GPIO.setup(GPIO_ECHO,GPIO.IN)    # Echo
     
    # Set trigger to False (Low)
    GPIO.output(GPIO_TRIGGER, False)
     
    # Allow module to settle
    time.sleep(0.01)
         
    #while distance > 5:
    #Send 10us pulse to trigger
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    begin = time.time()
    while GPIO.input(GPIO_ECHO)==0 and time.time()<begin+0.05:
        start = time.time()
     
    while GPIO.input(GPIO_ECHO)==1 and time.time()<begin+0.1:
        stop = time.time()
     
    # Calculate pulse length
    elapsed = stop-start
    
    # Distance pulse traveled in that time is time multiplied by the speed of sound (cm/s)
    distance = elapsed * 34300
     
    # That was the distance there and back, so take half of the value
    distance = distance / 2

    # Reset GPIO settings, return distance (in cm) appropriate for robot movements 
    return distance

def no_obstacle(distance_c, distance_l, distance_r):
    if(distance_c > sensor_proximity and distance_l > sensor_proximity and distance_r > sensor_proximity):
        return True
    else:
        return False

# Setup motors

MOTOR1B = 16  # LEFT motor
MOTOR1E = 15

MOTOR2B = 11  # RIGHT motor
MOTOR2E = 13

GPIO.setup(MOTOR1B, GPIO.OUT)
GPIO.setup(MOTOR1E, GPIO.OUT)
GPIO.setup(MOTOR2B, GPIO.OUT)
GPIO.setup(MOTOR2E, GPIO.OUT)

# Setup camera
from picamera2 import Picamera2
import numpy as np
import cv2
import time

redLower = (150, 140, 1)
redUpper = (190, 255, 255)

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (320, 240)}))
picam2.start()
time.sleep(1)

# Ultrasonic Sensor proximity parameter (centimeter)
sensor_proximity = 10
rerouting_proximity = 17.5

# Computer vision lower and upper turning range parameters for tracking
lower_range = 30
upper_range = 290

found = False
in_frame = False
direction = "none"

while True:
    # Process current frame with our functions
    frame = picam2.capture_array()

    if frame is None:
        print("Error: Frame not captured")
        break

    height, width = frame.shape[:2]
    
    print(str(height) + " X " + str(width))

    mask = find_color_mask(frame)
    x, y, radius, center, area = find_largest_contour(mask)
    
    print ("Area: " + str(area))
    
    print("Coordinates: " + str(x) + ", " + str(y))

    # Distance coming from front ultrasonic sensor
    distance_C = sonar(Trigger_C, Echo_C)
    distance_L = sonar(Trigger_L, Echo_L)
    distance_R = sonar(Trigger_R, Echo_R)
    print("D: " + str(distance_C) + ", " + str(distance_L) + ", " + str(distance_R))
    
    if radius > 10:
        if not found:
            found = True
            print("Found: " + str(found))
        in_frame = True
        cv2.circle(frame, (int(x), int(y)), int(radius), (255, 0, 0), 2)
        cv2.circle(frame, center, 5, (255, 0, 0), -1)
    else:
        in_frame = False
    
    if not no_obstacle(distance_C, distance_L, distance_R):
        print("Obstacle detetcted")
        stop()
        sleep(0.05)
    elif not found:
        sharp_right()
        sleep(0.05)
        stop()
    elif found and in_frame:
        if x > 210:
            direction = "right"
            sharp_right()
        elif x < 110:
            direction = "left"
            sharp_left()
        elif 110 <= x <= 210:
            forward()
        sleep(0.05)
        stop()
    elif found and not in_frame:
        if direction == "right":
            sharp_right()
        elif direction == "left":
            sharp_left()
        sleep(0.05)
        stop()
    
    stop()
    
    print(direction)
    
    cv2.imshow("Feed", frame)  # Shows frame with bounding box

    if cv2.waitKey(1) & 0xff == ord('q'):  # Press q to break the loop and stop moving
        stop()
        break

cv2.destroyAllWindows()
picam2.stop()
GPIO.cleanup()
