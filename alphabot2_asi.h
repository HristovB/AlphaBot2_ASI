//
// Created by Blagoj Hristov on 26/02/20.
//

#ifndef ALPHABOT2_ASI_H
#define ALPHABOT2_ASI_H

void initialize();

void oled_display_name();   // display competition name on OLED display (scrolling)

void lights(int led_1[3], int led_2[3], int led_3[3], int led_4[3]);    // setup custom-colour RGB LEDs

int read_ultrasonic(bool verbose);                  // read ultrasonic sensor output
char read_infrared(bool verbose);                   // read front infrared sensors output
unsigned int *read_infrared_line(bool verbose);     // read line infrared sensors output

void calibrate_line_sensors();                      // calibrate line infrared sensors

void forward(int speed);                            // move forward with set speed
void backward(int speed);                           // move backward with set speed
void left(int speed);                               // turn left with set speed
void right(int speed);                              // turn right with set speed
void stay();                                        // stop movement and display elapsed time

void set_left_motor_speed(int speed);               // set left motor speed
void set_right_motor_speed(int speed);              // set right motor speed

#endif
