The Ball-tracking Robot uses the Raspberry Pi system along with the Open Computer Vision Library (OpenCV) to track a red ball with a camera. Once the ball is found, the robot drives, with 2 DC motors and a motor driver, towards the ball, gauging the distance to the ball with an ultrasonic sensor. In this portfolio, I will describe the process I took to assemble and program the robot, any materials I used along the way, along with relevant schematics and diagrams where neccesary.

| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Yash S. | Leland High School | Mechanical Engineering / Robotics | Incoming Sophmore

![Headshot](Yash_S.jpg)

***

<!---

# Final Milestone

**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**

<iframe width="560" height="315" src="https://www.youtube.com/embed/F7M7imOVGug" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

For your final milestone, explain the outcome of your project. Key details to include are:
- What you've accomplished since your previous milestone
- What your biggest challenges and triumphs were at BSE
- A summary of key topics you learned about
- What you hope to learn in the future after everything you've learned at BSE
-->

# Second Milestone
For my second milestone, I have completed building my ultrasonic sensor voltage divider and circuit and have built and tested my ball-tracking code with OpenCV.

### Summary
To assemble my ultrasonic sensor circuit, I used a voltage divider [^9] with a 1K and 2K Ohm resistor to connect the 5V Ultrasonic sensor to my 3.3V operating Raspberry Pi. For my OpenCV software, my program has two main parts; the first is capturing camera footage with PiCamera2 [^10] (A PiCamera Library for camera functions) and using OpenCV (cv.imShow()) [^11] to show the frame on a preview. The second is image processing, using OpenCV to resize the image, blur the frame to reduce noise, and then generate a color mask for red with the HSV color space, dilating and eroding the image for even more precision. Then, a separate function takes in the color mask and finds the largest contour, identifying the ball quickly and precisely.

![Image of a voltage divider circuit](voltage_divider.png)

*How a voltage divider works and how to calculate the output: [https://studymind.co.uk/notes/potential-dividers/](https://studymind.co.uk/notes/potential-dividers/)*

As shown above, two resistors, in series allow us to alter the voltage sent to Vout, which will be smaller than Vin

### Testing
To test my ball-tracking functionality and my ultrasonic sensor, I used 2 additional test programs.

Here is my ultrasonic sensor testing program:
```python
from gpiozero import DistanceSensor

ultrasonic = DistanceSensor(echo=24, trigger=25)

while True:
    print(ultrasonic.distance)
```
This program sets up the ultrasonic sensors with the respective board pins and prints the distance received from the sensor continuously.

Here is my ball-tracking test code:
```python
from picamera2 import Picamera2
import numpy as np
import cv2
import time

redLower = (150, 140, 1)
redUpper = (190, 255, 255)

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (320, 240)}))
picam2.start()
time.sleep(2)

#Returns a mask of all colors within the red HSV space
def find_color_mask(frame):
	#Blur the image and convert to HSV color space
	blurred = cv2.GaussianBlur(frame, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	
	#Erode and Dilate to remove noise
	mask = cv2.inRange(hsv, redLower, redUpper)
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	
	return mask

#Finds the largest "blob" on the screen
def find_largest_contour(frame):
	# Finds contours in the provided image
	cnts, _ = cv2.findContours(frame.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if len(cnts) > 0:
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
	else:
		(x, y) = (0, 0)
		radius = 5
		center = (0, 0)
	
	return x, y, radius, center
	

while True:
	frame = picam2.capture_array()
	
	if frame is None:
		print("Error: Frame not captured")
		break
	
	mask = find_color_mask(frame)
	
	x, y, radius, center = find_largest_contour(mask)
	
	if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(frame, (int(x), int(y)), int(radius), (255, 0, 0), 2)
			cv2.circle(frame, center, 5, (255, 0, 0), -1)
	
	cv2.imshow("Tracking", frame)
	
	if(cv2.waitKey(1) & 0xff == ord('q')): #Press q to break the loop and stop moving 
		break

cv2.destroyAllWindows()
picam2.stop()
```
As explained, the code processes the current frame from the live camera feed and draws a circle around the detected ball.

Using this code, we can test to see that our ultrasonic sensor records accurate distances and that our ball-tracking functionality works.

### Challenges
My main challenge with this milestone was the ultrasonic sensor, while making the circuit I accidently fried a sensor due to an incorrect connection, but after replacing the sensor and verifying my layout, the sensor worked as expected

Now, I'll move on to building my main code, compiling together all of my work so far.

***

# First Milestone
For my first milestone, I completed the construction of the robot kit and all the wiring and tested that the robot and camera work with camera and motor testing code. 

### Summary
On the top of the robot is a Raspberry Pi 4B, powered by a large lithium-ion power bank and connected to the ArduCam 5MP camera in an acrylic case. On the bottom, we have a 2 H-Bridge [^6] (A circuit that can reverse polarity) motor driver wired up with two 6V DC [^7] (Direct current) motors. This driver is powered by four AA batteries, providing 6V of power to the driver, in series [^8] with this switch that can open or close the circuit to control whether the motor driver is on. Finally, the bridge between the two sides, 4 jumper wires, two for each motor, come in from the Raspberry Pi and are inputted into the motor driver to control the motors.

![Image of an H-bridge circuit](H-bridge.png)

*H-bridge circuit: [https://digilent.com/blog/what-is-an-h-bridge/#:~:text=An%20H%2Dbridge%20is%20built,directions%20by%20closing%20two%20switches.](https://digilent.com/blog/what-is-an-h-bridge/#:~:text=An%20H%2Dbridge%20is%20built,directions%20by%20closing%20two%20switches.)* 

As shown above, if 1 and 4 were closed, the current would flow to the right through the motor, making it spin in one direction. However, if 2 and 3 are closed, then the current flows to the left through the motor, spinning it the other way. This way, we can control the motors with 4 switches instead of 4 wires.

### Testing
To test the functionality of my robot, I used 2 test programs, one for the motors and one for the camera.

Here is a basic motor testing program:
```python
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
```
This program uses WASD inputs to direct the directions of the motors, using a ```while True``` loop to make testing easier and using the 'end' key to break out of the infinite loop and cleanup the GPIO outputs

Now, here is some basic camera testing code:
```python
from picamera2 import Picamera2
import numpy as np
import cv2
import time

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'RGB888', "size": (320, 240)}))
picam2.start()
time.sleep(2)

while True:
    frame = picam2.capture_array()
    
    if frame is None:
        print("Error: Frame not captured")
        break
    
    cv2.imshow("Tracking", frame)

    if(cv2.waitKey(1) & 0xff == ord('q')): #Press q to break the loop and stop moving 
        break

cv2.destroyAllWindows()
picam2.stop()
```
This code has 2 main parts, with a library for each:

1. The PiCamera2 Library helps us configure the camera with ```PiCamera2()``` and captures the current frame every few milliseconds with ```picam2.capture_array()```

3. The OpenCV library helps us take the current frame captured by PiCamera2 and display it in a preview window, updating it on new frame from ```picam2```

We can efficiently test and debug our camera and motors with these two programs, allowing us to continue building the robot's full code.

### Challenges
My main challenge with this process was that almost every part of the kit was broken, including the battery pack which didn't supply any voltage, the motor driver which gave no output voltage, and the switch that melted and wouldn't turn on. I had to replace the battery pack and motor driver and use a Dremel Saw to make a larger hole for a new switch. In addition, I was originally connecting power to +5V, but this was incorrect, I had to connect it to VCC, and once I fixed this error, the motors worked normally and I could control the robot as I wanted. VCC accepts up to +12V on this driver and should always be used. The +5V port only exists for additional power supply in case of excessive voltage load on the driver itself.

Now, I am ready to move on to my second milestone, building and testing OpenCV ball-tracking code on my Raspberry Pi.

<iframe width="560" height="315" src="https://www.youtube.com/embed/S4u5ani-CXQ?si=F4eayMsDoPPQVb7L" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

***

# Starter Project
For my starter project, I built the calculator ([link](https://www.amazon.com/Kit-Calculator-Resistance-Electronic-HUAGZIMO/dp/B0D13C9SYT/ref=sr_1_3?crid=3HGJTLNZ9O2GX&dib=eyJ2IjoiMSJ9._SWtzcdxglPoBR9j02Ru8HdkQYctYGhXoQSzf1MVwW8-wdJNSkQkLmCAtn4dRp6g-6R7J9461vhIP2EF_nk7Tig6XDG9bCrlMTSlmck5MBQwLRhhnSiQUGo0QJa1GwgSj6a6-1yBKFqneN2-Z0AqO-StnMGL2G8655x5qfsjKhjBt48dYiTRy3_0E2_Jk5agtyEMTLExRFtYVrPI5ML2CKLPh8c4dT4clp-o5in2kS0.ajrpXguZyLba8zZbq_b1WT_1ccEQlOe_PpesP9bkSUM&dib_tag=se&keywords=calculator+solder&qid=1717994208&sprefix=calculator+solde%2Caps%2C148&sr=8-3)), because I thought it would be a fun way to get better at soldering and a fun project to find later on and use to my liking. It can perform all standard operations, including addition, subtraction, multiplication, and division, even with decimals.

### Summary
The main components of the calculator are the General Purpose Input Output [^1] (GPIO) buttons, which can either be in a HIGH or LOW state. These signals are sent electrically to this chip, acting as the "brain" of the calculator. It contains Floating Point units (FPU) [^2], which are tiny circuits that process the floating point numbers and send an output to these two seven-segment displays [^3]. These displays can display digits from 0-9 in addition to decimal points, and update on signal from the chip. They are a simpler way to display floating-point numbers than more complex matrix displays, due to only having seven main Light-emitting diode (LED) [^4] segments. We also have a micro USB adapter, allowing us to connect our calculator to a computer and edit code as needed. Finally, we have a battery socket and a CR2032 battery [^5], a standard 3-volt battery, which powers the entire calculator and can be removed and replaced as needed.

![Image of 2 circuits](GPIO_Button.jpg)

*How a GPIO button works: [https://www.electronicshub.org/raspberry-pi-push-button-interface/](https://www.electronicshub.org/raspberry-pi-push-button-interface/)*

On the left, we can see that this circuit "holds" the voltage of GPIO_IN at +5v by opening the circuit at the switch. On the right, the same way the voltage is held at 0V by opening the circuit and disconnecting the +5V source.

### Challenges
The most difficult challenge I faced during this build was related to the chip. There was originally an Integrated Circuit, or IC socket that the chip was placed in before soldering that made it easier to remove the chip. Still, a pin on the socket had broken, so after consideration, I decided that I would never need to remove the chip and soldered it onto the board to solve the problem. 

At this point, I will move on to my main project, the ball-tracking robot, and begin assembling my Raspberry Pi circuit and components.

<iframe width="560" height="315" src="https://www.youtube.com/embed/65gltkMqXt4?si=qwEcdHus98TpXgqG" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

***

<!--
# Schematics 
Here's where you'll put images of your schematics. [Tinkercad](https://www.tinkercad.com/blog/official-guide-to-tinkercad-circuits) and [Fritzing](https://fritzing.org/learning/) are both great resoruces to create professional schematic diagrams, though BSE recommends Tinkercad becuase it can be done easily and for free in the browser. 

# Code
Here's where you'll put your code. The syntax below places it into a block of code. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize it to your project needs. 

```c++
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Hello World!");
}

void loop() {
  // put your main code here, to run repeatedly:

}
```
-->

# Bill of Materials

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--|
| Raspberry Pi 4B (4GB) | Computer used to process OpenCV algorithms and run code | $55 | [Adafruit Link](https://www.adafruit.com/product/4292?gad_source=1&gclid=CjwKCAjwjqWzBhAqEiwAQmtgT90CMD9fBQO5jZe9hAO2hHY35kJDs0KMfQkFNoEpym-fFbT87SJinhoCLmUQAvD_BwE) |
| ArduCam 5MP Camera | Used to capture video footage of the ball for the RPi | $12.99 | [Amazon Link](https://www.amazon.com/Arducam-Raspberry-Camera-Module-1080P/dp/B07RWCGX5K?ref_=ast_sto_dp&th=1&psc=1) |
| Robot kit | Physical structure, motors, battery pack, and switch | $12.99 | [Amazon Link](https://www.amazon.com/Smart-Chassis-Motors-Encoder-Battery/dp/B01LXY7CM3/ref=asc_df_B01LXY7CM3/?tag=hyprod-20&linkCode=df0&hvadid=693448563566&hvpos=&hvnetw=g&hvrand=3851559169840908154&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=9032131&hvtargid=pla-584495246069&psc=1&mcid=3f54405bf4d93ae384d78ef10f75b962&gad_source=1) |
| Ultrasonic Sensors (3) | Used to gauge distance with ultrasonic waves | $5.50 | [Amazon Link (5 pack)](https://www.amazon.com/HC-SR04-Ultrasonic-Distance-Measuring-MEGA2560/dp/B088BT8CDW) |
| Breadboard | Used for prototyping and ultrasonic sensor voltage dividers | $2.47 | [DigiKey Link](https://www.digikey.com/en/products/detail/universal-solder-electronics-ltd/SOLDERLESS%2520BREADBOARD%2520400/16819785?utm_adgroup=&utm_source=google&utm_medium=cpc&utm_campaign=PMax_DK%2BProduct_Product%20Categories%20-%20Top%2015&utm_term=&utm_content=&utm_id=go_cmp-19646629144_adg-_ad-__dev-c_ext-_prd-16819785_sig-CjwKCAjw1emzBhB8EiwAHwZZxd6v-xSnrxKPvl7hmWV6exRQQ6x-2BGD0p4nHQ5W0mPDOCQUBKcy6xoClwMQAvD_BwE&gad_source=1&gclid=CjwKCAjw1emzBhB8EiwAHwZZxd6v-xSnrxKPvl7hmWV6exRQQ6x-2BGD0p4nHQ5W0mPDOCQUBKcy6xoClwMQAvD_BwE) |
| H-Bridge Motor driver | Used to control motors with direction | $2.30 | [Link](https://www.smart-prototyping.com/L298N-Dual-H-bridge-Motor-Driver-Board) |

***

# Resources 

[^1]: [GPIO Buttons](https://www.renesas.com/us/en/support/engineer-school/mcu-programming-peripherals-01-gpio)

[^2]: [FPU (Floating point unit)](https://www.techopedia.com/definition/2865/floating-point-unit-fpu)

[^3]: [Seven-segment displays](https://www.electronics-tutorials.ws/blog/7-segment-display-tutorial.html)

[^4]: [LED's (Light Emitting Diodes)](https://www.rohm.com/electronics-basics/leds/what-are-leds#:~:text=LEDs%20(Light%20Emitting%20Diodes)%20are,semiconductor%20(larger%20electron%20concentration).)

[^5]: [CR2032 Battery](https://www.cr2032.co/cr2032-functions-article.html#:~:text=The%20CR2032%20battery%20uses%20a,like%20watches%20and%20remote%20controls.)

[^6]: [H-bridge circuits](https://digilent.com/blog/what-is-an-h-bridge/#:~:text=An%20H%2Dbridge%20is%20built,directions%20by%20closing%20two%20switches.)

[^7]: [Direct current vs. Alternating current](https://learn.sparkfun.com/tutorials/alternating-current-ac-vs-direct-current-dc/all)

[^8]: [Series vs. Parallel Circuits](https://learn.sparkfun.com/tutorials/series-and-parallel-circuits/all)

[^9]: [Voltage divider](https://learn.sparkfun.com/tutorials/voltage-dividers/all)

[^10]: [PiCamera2 Library](https://pypi.org/project/picamera2/0.2.2/)

[^11]: [OpenCV Preview](https://docs.opencv.org/3.4/dd/d43/tutorial_py_video_display.html)
