# Import motor and camera setup from our files
from motor import (
    forward,
    stop,
    sharp_left,
    sharp_right,
)
from ball import find_color_mask, find_largest_contour
from time import sleep
import RPi.GPIO as GPIO
import pigpio

pan = 16
tilt = 26

pan_a = 1500
tilt_a = 1500

v_direction = "none"
h_direction = "right"

pwm = pigpio.pi()

pwm.set_mode(pan, pigpio.OUTPUT)
pwm.set_mode(tilt, pigpio.OUTPUT)

pwm.set_PWM_frequency(pan, 50)
pwm.set_PWM_frequency(tilt, 50)

print("90 deg")
pwm.set_servo_pulsewidth(pan, 1500)
pwm.set_servo_pulsewidth(tilt, 1500)

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


def sonar(GPIO_TRIGGER, GPIO_ECHO):
    start = 0
    stop = 0
    # Set pins as output and input
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)  # Trigger
    GPIO.setup(GPIO_ECHO, GPIO.IN)  # Echo

    # Set trigger to False (Low)
    GPIO.output(GPIO_TRIGGER, False)

    # Allow module to settle
    time.sleep(0.01)

    # while distance > 5:
    # Send 10us pulse to trigger
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    begin = time.time()
    while GPIO.input(GPIO_ECHO) == 0 and time.time() < begin + 0.05:
        start = time.time()

    while GPIO.input(GPIO_ECHO) == 1 and time.time() < begin + 0.1:
        stop = time.time()

    # Calculate pulse length
    elapsed = stop - start

    # Distance pulse traveled in that time is time multiplied by the speed of sound (cm/s)
    distance = elapsed * 34300

    # That was the distance there and back, so take half of the value
    distance = distance / 2

    # Reset GPIO settings, return distance (in cm) appropriate for robot movements
    return distance


def no_obstacle(distance_c, distance_l, distance_r):
    if (
        distance_c > sensor_proximity
        and distance_l > sensor_proximity
        and distance_r > sensor_proximity
    ):
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
import cv2
import time

redLower = (150, 140, 1)
redUpper = (190, 255, 255)

picam2 = Picamera2()
picam2.configure(
    picam2.create_preview_configuration(main={"format": "RGB888", "size": (320, 240)})
)
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

    print("Area: " + str(area))

    print("Coordinates: " + str(x) + ", " + str(y))

    # Distance coming from front ultrasonic sensor
    distance_C = sonar(Trigger_C, Echo_C)
    distance_L = sonar(Trigger_L, Echo_L)
    distance_R = sonar(Trigger_R, Echo_R)
    print("D: " + str(distance_C) + ", " + str(distance_L) + ", " + str(distance_R))

    if radius > 40:
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
        sleep(0.075)
        stop()
    elif found and in_frame:
        if x > 260:
            direction = "right"
            sharp_right()
            sleep(0.075)
        elif x < 60:
            direction = "left"
            sharp_left()
            sleep(0.075)
        elif 110 <= x <= 210:
            forward()
            sleep(0.2)
        stop()
    elif found and not in_frame:
        if direction == "right":
            sharp_right()
        elif direction == "left":
            sharp_left()
        sleep(0.075)
        stop()

    print(direction)

    cv2.imshow("Feed", frame)  # Shows frame with bounding box

    if cv2.waitKey(1) & 0xFF == ord("q"):  # Press q to break the loop and stop moving
        stop()
        break

while True:
    frame = picam2.capture_array()

    mask = find_color_mask(frame)
    x, y, radius, center, area = find_largest_contour(mask)

    if radius > 20:
        found = True
        cv2.circle(frame, (int(x), int(y)), int(radius), (255, 0, 0), 2)
        cv2.circle(frame, center, 5, (255, 0, 0), -1)
    else:
        found = False

    if found:
        if x < 150:
            h_direction = "left"
            if pan_a < 2500:
                pan_a += 20
            pwm.set_servo_pulsewidth(pan, pan_a)
        elif x > 170:
            h_direction = "right"
            if pan_a > 500:
                pan_a -= 20
            pwm.set_servo_pulsewidth(pan, pan_a)

        if y > 130:
            v_direction = "up"
            if tilt_a < 2500:
                tilt_a += 20
            pwm.set_servo_pulsewidth(tilt, tilt_a)
        elif y < 110:
            v_direction = "down"
            if tilt_a > 500:
                tilt_a -= 20
            pwm.set_servo_pulsewidth(tilt, tilt_a)

    elif v_direction != "none" and h_direction != "none":
        if h_direction == "left":
            if pan_a < 2500:
                pan_a += 20
            pwm.set_servo_pulsewidth(pan, pan_a)
        elif h_direction == "right":
            if pan_a > 500:
                pan_a -= 20
            pwm.set_servo_pulsewidth(pan, pan_a)

        if v_direction == "up":
            if tilt_a < 2500:
                tilt_a += 20
            pwm.set_servo_pulsewidth(tilt, tilt_a)
        elif v_direction == "down":
            if tilt_a > 500:
                tilt_a -= 20
            pwm.set_servo_pulsewidth(tilt, tilt_a)

    cv2.imshow("Feed", frame)  # Shows frame with bounding box

    if cv2.waitKey(1) & 0xFF == ord("q"):  # Press q to break the loop and stop moving
        break

pwm.set_servo_pulsewidth(pan, 1500)
pwm.set_servo_pulsewidth(tilt, 1500)
sleep(1)

# Cleanup
pwm.set_PWM_dutycycle(pan, 0)
pwm.set_PWM_dutycycle(tilt, 0)

pwm.set_PWM_frequency(pan, 0)
pwm.set_PWM_frequency(tilt, 0)

cv2.destroyAllWindows()
picam2.stop()
GPIO.cleanup()
