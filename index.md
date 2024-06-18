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
For my first milestone, I completed the construction of the robot kit and all the wiring and tested that the robot and camera work with camera and motor testing code. My main challenge with this process was that almost every part of the kit was broken, including the battery pack which didn't supply any voltage, the motor driver which gave no output voltage, and the switch that melted and wouldn't turn on. I had to replace the battery pack and motor driver and use a Dremel Saw to make a larger hole for a new switch. In addition, I was originally connecting power to +5V, but this was incorrect, I had to connect it to VCC, and once I fixed this error, the motors worked normally and I could control the robot as I wanted. 

### Structure
IN PROGRESS

***

# Starter Project
For my starter project, I built the calculator ([link](https://www.amazon.com/Kit-Calculator-Resistance-Electronic-HUAGZIMO/dp/B0D13C9SYT/ref=sr_1_3?crid=3HGJTLNZ9O2GX&dib=eyJ2IjoiMSJ9._SWtzcdxglPoBR9j02Ru8HdkQYctYGhXoQSzf1MVwW8-wdJNSkQkLmCAtn4dRp6g-6R7J9461vhIP2EF_nk7Tig6XDG9bCrlMTSlmck5MBQwLRhhnSiQUGo0QJa1GwgSj6a6-1yBKFqneN2-Z0AqO-StnMGL2G8655x5qfsjKhjBt48dYiTRy3_0E2_Jk5agtyEMTLExRFtYVrPI5ML2CKLPh8c4dT4clp-o5in2kS0.ajrpXguZyLba8zZbq_b1WT_1ccEQlOe_PpesP9bkSUM&dib_tag=se&keywords=calculator+solder&qid=1717994208&sprefix=calculator+solde%2Caps%2C148&sr=8-3)), because I thought it would be a fun way to get better at soldering and a fun project to find later on and use to my liking. It can perform all standard operations, including addition, subtraction, multiplication, and division, even with decimals.

### Summary
The main components of the calculator are the General Purpose Input Output^[^1]^ (GPIO) buttons, which can either be in a HIGH or LOW state. These signals are sent electrically to this chip, acting as the "brain" of the calculator. It contains Floating Point units (FPU) [^2], which are tiny circuits that process the floating point numbers and send an output to these two seven-segment displays [^3]. These displays can display digits from 0-9 in addition to decimal points, and update on signal from the chip. They are a simpler way to display floating-point numbers than more complex matrix displays, due to only having seven main Light-emitting diode (LED) [^4] segments. We also have a micro USB adapter, allowing us to connect our calculator to a computer and edit code as needed. Finally, we have a battery socket and a CR2032 battery [^5], a standard 3-volt battery, which powers the entire calculator and can be removed and replaced as needed.

![Image of 2 circuits](GPIO_Button.jpg)

*How a GPIO Button works*

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
### Starter project:

[^1]: GPIO Buttons: https://www.renesas.com/us/en/support/engineer-school/mcu-programming-peripherals-01-gpio

[^2]: FPU (Floating point unit): https://www.techopedia.com/definition/2865/floating-point-unit-fpu

[^3]: Seven-segment displays: https://www.electronics-tutorials.ws/blog/7-segment-display-tutorial.html

[^4]: LED's (Light Emitting Diodes): https://www.rohm.com/electronics-basics/leds/what-are-leds#:~:text=LEDs%20(Light%20Emitting%20Diodes)%20are,semiconductor%20(larger%20electron%20concentration).

[^5]: CR2032 Battery: https://www.cr2032.co/cr2032-functions-article.html#:~:text=The%20CR2032%20battery%20uses%20a,like%20watches%20and%20remote%20controls.

<!--
# Other Resources/Examples
One of the best parts about Github is that you can view how other people set up their own work. Here are some past BSE portfolios that are awesome examples. You can view how they set up their portfolio, and you can view their index.md files to understand how they implemented different portfolio components.
- [Example 1](https://trashytuber.github.io/YimingJiaBlueStamp/)
- [Example 2](https://sviatil0.github.io/Sviatoslav_BSE/)
- [Example 3](https://arneshkumar.github.io/arneshbluestamp/)

To watch the BSE tutorial on how to create a portfolio, click here.
-->

