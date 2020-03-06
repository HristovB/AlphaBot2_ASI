//
// Created by Blagoj Hristov on 26/02/20.
//

#ifndef ALPHABOT2_ASI_H
#define ALPHABOT2_ASI_H

void initialize_alphabot();

int read_ultrasonic(bool verbose);                  // read ultrasonic sensor output
char read_infrared(bool verbose);                   // read front infrared sensors output

void calibrate_line_sensors();                      // calibrate line infrared sensors
unsigned int *read_infrared_line(bool verbose);     // read line infrared sensors output

// WIP PID controller function - DOES NOT WORK! NOT USABLE!
// void line_follower(unsigned int *s, const float Kp, const float Ki, const float Kd, const int max_speed);

void forward(int speed);                            // make alphabot move forward
void backward(int speed);                           // make alphabot move backward
void left();                                        // make alphabot turn left
void right();                                       // make alphabot turn right
void stay();                                        // make alphabot stay in place

void set_left_motor_speed(int speed_val);           // set left motor speed
void set_right_motor_speed(int speed_val);          // set right motor speed


#endif
