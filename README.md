# Sumo-Bot

This is a project that I worked on for my class ME 405: Mechatronics.  
My partner and I were tasked to design and build a sumo-bot that runs completely autonomously.  
The robot is equipped with an IR sensor to start/stop the robot, line detecting 
sensors to detect when it reaches the limit of the sumo ring, an ultrasonic sensor to detect enemy sumo bots, and 
motor encoders in order to implement PID motion control.  We controlled our robot by implementing the task-state
programming approach. We writing a series of tasks, which were then run by a scheduler written by our professor
that determines which task to run based on its priority and timing requirements.  We programmed in Python and used a
Nucleo Development board equipped with and STM32 microcontroller in order to run our code.  Please reach out for more information.
