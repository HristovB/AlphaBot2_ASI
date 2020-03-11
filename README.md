# AlphaBot2-Ar Function Library
AlphaBot2 library for Arduino Uno microcontroller for RoboMac Junior 2020.

## Library Documentation

### void initialize()

This function should always be called in the ***void setup()*** function of the Arduino IDE, as it initializes basic pins for the sensors and motors, the OLED display, RGB diodes as well as the Serial Monitor.

**Input:** *None*
**Output:** *None*

### void lights()

This function is for setting a custom colour for each of the RGB diodes on the back of the AlphaBot2.

**Input:** *Four integer arrays containing three values for R, G, and B, for each of the four diodes*
**Output:** *None*

### int read_ultrasonic(bool verbose=false)

This function is for reading the value of the ultrasonic sensor placed on the front of the AlphaBot2.

**Input:** *Boolean value for printing result (true/false)*
**Output:** *Approximate distance to obstacle given in centimeters*

### char read_infrared(bool verbose=false)

This function is for reading the front-facing infrared sensors for obstacle detection on the AlphaBot2.

**Input:** *Boolean value for printing results (true/false)*
**Output:** *Character representing which sensor/s is/are active ('L' - left, 'R' - right, 'B' - both, 'N' - none)*

### int \*read_infrared_line(bool verbose=false)

This function is for reading the values of the bottom infrared sensors intended for line following. If calibrated properly, the individual sensor output should be equal to **0** when there is no line detected, and close to **1000** when the line is directly under the sensor.

**Input:** *Boolean value for printing results (true/false)*
**Output:** *Integer pointer pointing to an array containing the values for all five infrared sensors*

### void calibrate_line_sensors()

This function is for calibrating the bottom infrared sensors intended for line following. The values printed after the calibration should be exchanged for the already used values in the ***initialize()*** function.

**Input:** *None*
**Output:** *None*

### void forward(int speed)

This function is for making the AlphaBot2 move forward with a given speed (maximum of 180, for safety).

**Input:** *Integer value for speed*
**Output:** *None*

### void backward(int speed)

This function is for making the AlphaBot2 move backward with a given speed (maximum of 180, for safety).

**Input:** *Integer value for speed*
**Output:** *None*

### void left(int speed)

This function is for making the AlphaBot2 turn left with a given speed (maximum of 180, for safety).

**Input:** *Integer value for speed*
**Output:** *None*

### void right(int speed)

This function is for making the AlphaBot2 turn right with a given speed (maximum of 180, for safety).

**Input:** *Integer value for speed*
**Output:** *None*

### void stay()

This function is for stopping any movement of the AlphaBot2.

**Input:** *None*
**Output:** *None*

### void set_left_motor_speed(int speed)

This function is for setting the speed of *ONLY* the left motor, without changing the speed of the right motor (maximum of 180, for safety).

**Input:** *Integer value for speed*
**Output:** *None*

### void set_right_motor_speed(int speed)

This function is for setting the speed of *ONLY* the right motor, without changing the speed of the left motor (maximum of 180, for safety).

**Input:** *Integer value for speed*
**Output:** *None*
