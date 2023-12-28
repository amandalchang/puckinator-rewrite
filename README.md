# Puckinator
Puckinator is a robotic air hockey defender project created by Amanda Chang, 
Meagan Lipsman, Ben Tarr, Lily Dao, and Elin O'Neill. The files puckinator.py
and arm_control.ino work in conjunction to control a parallel SCARA arm to 
defend an air hockey goal. 

## Website Link
Detailed documentation on the design, creation and implementation of our
project can be found on our project website [here](https://meaganlipsman.github.io/pie-2023-03/puckinator/index.html).

## In Action
Watch a short video of our design process [here](https://www.youtube.com/watch?v=9_xLwALPo-s)!

## Installation 
Use our *requirements.txt* file to install the required python libraries.
You must manually install the FlexyStepper Arduino library.

```bash
pip install -r requirements.txt 
```

## Usage
Once you have replicated our electrical and mechanical setup, connect an 
Arduino Uno (R3 or R4) to a USB port on your computer and upload 
arm_control.ino. Identify the port name and edit the PORT constant in 
puckinator.py. From there, run the python file puckinator.py to start the game!
