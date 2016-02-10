# auto_ardunio_nano

#main
control power_board using green_button attached Pin PD4
Timer2 is used to turn off the board if green_button is pressed more than 3 seconds  

Control lights using serial port commands
Timer2 is used for flashing during turing left/right

control servo motor using serial port commands
Timer1 is used to make pwm

Install MsTimer2 on {arduino-path}/libraries/
Ref:http://playground.arduino.cc/Main/MsTimer2
_______
#servo
code just for control servo motor using serial port and ardunio nano

