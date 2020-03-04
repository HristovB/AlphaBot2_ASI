//
// Created by Blagoj Hristov on 26/02/20.
//

#ifndef ALPHABOT2_ASI_H
#define ALPHABOT2_ASI_H

void initialize_alphabot();

int read_ultrasonic(bool verbose);
char read_infrared(bool verbose);

void calibrate_line_sensors();
unsigned int *read_infrared_line(bool verbose);
void line_follower(unsigned int *s, const float Kp, const float Ki, const float Kd, const int max_speed);

void forward(int speed);
void backward(int speed);
void left();
void right();
void stay();



#endif
