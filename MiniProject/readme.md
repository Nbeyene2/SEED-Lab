This folder contains all of the code, simulations, and models that were created for the mini project.
The purpose of these files is to spi n a wheel to a desired position, when a camera detects an aruco marker
in a corresponding quadrant. In order to accomplish this, we created a rasberry pi script to detect which quadrant the 
aruco marker is in. Then, the script tells outputs the quadrant number to the arduino code. The arduino code uses a PWM
and a feedback loop in order to control the wheel's position. The code also uses the inputted quadrant position to 
control the wheel to spin to the correct position. We did this by splitting the total number of counts that the motor has 
into four sections.
