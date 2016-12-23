#Line follower robot

This is the firmware of a 2-wheel robot which follows a curved line. Because this requires some settings, the robot includes an on-board OLED display through which the parameters are visualized. The display draws a menu and its items are selected using 4 buttons -- UP, DOWN, SELECT, BACK. The UP and DOWN buttons are also used to increment and decrement the values of the parameters.

We use Pololu's QTR library to control the DC motors and U8glib to control the OLED display. They can be imported through the Arduino IDE.
