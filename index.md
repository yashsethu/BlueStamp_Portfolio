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

# Second Milestone

**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**

<iframe width="560" height="315" src="https://www.youtube.com/embed/y3VAmNlER5Y" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

For your second milestone, explain what you've worked on since your previous milestone. You can highlight:
- Technical details of what you've accomplished and how they contribute to the final goal
- What has been surprising about the project so far
- Previous challenges you faced that you overcame
- What needs to be completed before your final milestone 
-->

# First Milestone
For my first milestone, I completed the construction of the robot kit and all the wiring and tested that the robot and camera work with camera and motor testing code. 

### Summary
On the top of the robot is a Raspberry Pi 4B, powered by a large lithium-ion power bank and connected to the ArduCam 5MP camera in an acrylic case. On the bottom, we have a 2 H-Bridge [^6] (A circuit that can reverse polarity) motor driver wired up with two 6V DC [^7] (Direct current) motors. This driver is powered by four AA batteries, providing 6V of power to the driver, in series [^8] with this switch that can open or close the circuit to control whether the motor driver is on. Finally, the bridge between the two sides, 4 jumper wires, two for each motor, come in from the Raspberry Pi and are inputted into the motor driver to control the motors.

![Image of an H-bridge circuit](H-bridge.png)

*H-bridge circuit: https://digilent.com/blog/what-is-an-h-bridge/#:~:text=An%20H%2Dbridge%20is%20built,directions%20by%20closing%20two%20switches.* 

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

***

# Starter Project
For my starter project, I built the calculator ([link](https://www.amazon.com/Kit-Calculator-Resistance-Electronic-HUAGZIMO/dp/B0D13C9SYT/ref=sr_1_3?crid=3HGJTLNZ9O2GX&dib=eyJ2IjoiMSJ9._SWtzcdxglPoBR9j02Ru8HdkQYctYGhXoQSzf1MVwW8-wdJNSkQkLmCAtn4dRp6g-6R7J9461vhIP2EF_nk7Tig6XDG9bCrlMTSlmck5MBQwLRhhnSiQUGo0QJa1GwgSj6a6-1yBKFqneN2-Z0AqO-StnMGL2G8655x5qfsjKhjBt48dYiTRy3_0E2_Jk5agtyEMTLExRFtYVrPI5ML2CKLPh8c4dT4clp-o5in2kS0.ajrpXguZyLba8zZbq_b1WT_1ccEQlOe_PpesP9bkSUM&dib_tag=se&keywords=calculator+solder&qid=1717994208&sprefix=calculator+solde%2Caps%2C148&sr=8-3)), because I thought it would be a fun way to get better at soldering and a fun project to find later on and use to my liking. It can perform all standard operations, including addition, subtraction, multiplication, and division, even with decimals.

### Summary
The main components of the calculator are the General Purpose Input Output [^1] (GPIO) buttons, which can either be in a HIGH or LOW state. These signals are sent electrically to this chip, acting as the "brain" of the calculator. It contains Floating Point units (FPU) [^2], which are tiny circuits that process the floating point numbers and send an output to these two seven-segment displays [^3]. These displays can display digits from 0-9 in addition to decimal points, and update on signal from the chip. They are a simpler way to display floating-point numbers than more complex matrix displays, due to only having seven main Light-emitting diode (LED) [^4] segments. We also have a micro USB adapter, allowing us to connect our calculator to a computer and edit code as needed. Finally, we have a battery socket and a CR2032 battery [^5], a standard 3-volt battery, which powers the entire calculator and can be removed and replaced as needed.

![Image of 2 circuits](GPIO_Button.jpg)

*How a GPIO button works: https://www.electronicshub.org/raspberry-pi-push-button-interface/*

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
| Raspberry Pi 4B (4GB) | Computer used to process OpenCV algorithms and run code | $55 | <a href="[https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/](https://www.adafruit.com/product/4292?gad_source=1&gclid=CjwKCAjwjqWzBhAqEiwAQmtgT90CMD9fBQO5jZe9hAO2hHY35kJDs0KMfQkFNoEpym-fFbT87SJinhoCLmUQAvD_BwE)"> Link </a> |
| ArduCam 5MP Camera | Used to capture video footage of the ball for the RPi | $12.99 | <a href="[https://www.amazon.com/Arduino-A000066-ARDUINO-UNO-R3/dp/B008GRTSV6/](https://www.amazon.com/Arducam-Raspberry-Camera-Module-1080P/dp/B07RWCGX5K?ref_=ast_sto_dp&th=1&psc=1)"> Link </a> |

***

# Resources 

[^1]: GPIO Buttons: https://www.renesas.com/us/en/support/engineer-school/mcu-programming-peripherals-01-gpio

[^2]: FPU (Floating point unit): https://www.techopedia.com/definition/2865/floating-point-unit-fpu

[^3]: Seven-segment displays: https://www.electronics-tutorials.ws/blog/7-segment-display-tutorial.html

[^4]: LED's (Light Emitting Diodes): https://www.rohm.com/electronics-basics/leds/what-are-leds#:~:text=LEDs%20(Light%20Emitting%20Diodes)%20are,semiconductor%20(larger%20electron%20concentration).

[^5]: CR2032 Battery: https://www.cr2032.co/cr2032-functions-article.html#:~:text=The%20CR2032%20battery%20uses%20a,like%20watches%20and%20remote%20controls.

[^6]: H-bridge circuits: https://digilent.com/blog/what-is-an-h-bridge/#:~:text=An%20H%2Dbridge%20is%20built,directions%20by%20closing%20two%20switches.

[^7]: Direct current vs. Alternating current: https://learn.sparkfun.com/tutorials/alternating-current-ac-vs-direct-current-dc/all

[^8]: Series vs. Parallel Circuits: https://learn.sparkfun.com/tutorials/series-and-parallel-circuits/all

<!--
# Other Resources/Examples
One of the best parts about Github is that you can view how other people set up their own work. Here are some past BSE portfolios that are awesome examples. You can view how they set up their portfolio, and you can view their index.md files to understand how they implemented different portfolio components.
- [Example 1](https://trashytuber.github.io/YimingJiaBlueStamp/)
- [Example 2](https://sviatil0.github.io/Sviatoslav_BSE/)
- [Example 3](https://arneshkumar.github.io/arneshbluestamp/)

To watch the BSE tutorial on how to create a portfolio, click here.
-->

