//
// Created by Blagoj Hristov on 26/02/20.
//

#ifndef ALPHABOT2_ASI_H
#define ALPHABOT2_ASI_H

void initialize_alphabot();
int read_ultrasonic(bool verbose=false);
char read_infrared(bool verbose=false);
void forward(int speed);
void backward(int speed);
void left();
void right();
void stay();
void right_angle(int angle);

#endif
