//
// Created by Blagoj Hristov on 26/02/20.
//

#include "alphabot2_asi.h"
#include <Arduino.h>
#include <Wire.h>

#define ECHO   2        // ultrasonic echo
#define TRIG   3        // ultrasonic trigger
#define PWMA   6        // motor-L speed (ENA)
#define AIN2   A0       // motor-L forward (IN2)
#define AIN1   A1       // motor-L backward (IN1)
#define PWMB   5        // motor-R speed (ENB)
#define BIN1   A2       // motor-R forward (IN3)
#define BIN2   A3       // motor-R backward (IN4)
#define PCFADR  0x20    // PCF8574 I/O expansion module address (for forward facing infrared sensors)


void initialize_alphabot(){
    Serial.begin(115200);
    Wire.begin();
    pinMode(ECHO, INPUT);    // define the ultrasonic echo input pin
    pinMode(TRIG, OUTPUT);   // define the ultrasonic trigger input pin
    pinMode(PWMA, OUTPUT);   // define the left motor speed pin
    pinMode(AIN2, OUTPUT);   // define the left motor forward pin
    pinMode(AIN1, OUTPUT);   // define the left motor backward pin
    pinMode(PWMB, OUTPUT);   // define the right motor speed pin
    pinMode(AIN1, OUTPUT);   // define the right motor forward pin
    pinMode(AIN2, OUTPUT);   // define the right motor backward pin
}

void write_PCF8574(byte data){
    Wire.beginTransmission(PCFADR);
    Wire.write(data);
    Wire.endTransmission();
}

byte read_PCF8574(){
    int data = -1;
    Wire.requestFrom(PCFADR, 1);
    if(Wire.available()) {
        data = Wire.read();
    }
    return data;
}

char read_infrared(bool verbose=false){
    byte value;     // declare sensor value as byte type variable
    write_PCF8574(0xC0 | read_PCF8574());   // write sensor output to PCF8574 I/O expansion module
    value = read_PCF8574() | 0x3F;         // read PCF module (sensor output)

    if(verbose){    // print result
        if(value == 0x7F)
            Serial.println("Left sensor detection.");

        else if(value == 0xBF)
            Serial.println("Right sensor detection.");

        else if(value == 0x3F)
            Serial.println("Both sensors detection.");

        else if(value == 0xFF)
            Serial.println("No detection.");
    }

    if(value == 0x7F)       // check if left sensor is active
        return 'L';

    else if(value == 0xBF)  // check if right sensor is active
        return 'R';

    else if(value == 0x3F)  // check if both sensors are active
        return 'B';

    else if(value == 0xFF)  // check if no sensors are active
        return 'N';

    else
        Serial.println("ERROR!");
}

void sort(int *arr, int n){
    int tmp;

    for(int i = 0; i < n; i++){
        for(int j = i; j < n; j++){
            if (*(arr + j) < *(arr + i)){
                tmp = *(arr + i);
                *(arr + i) = *(arr + j);
                *(arr + j) = tmp;
            }
            else{
                continue;
            }
        }
    }
}

int read_ultrasonic(bool verbose){        // measure distance with ultrasonic sensor
    int length = 15;    // number of measurements
    int measurements[length];
    float f_dist;

    for(int i = 0; i < length; i++){    // measure 'length' times for better accuracy
        digitalWrite(TRIG, LOW);   // set trig pin to low for 2μs (reset trigger)
        delayMicroseconds(2);

        digitalWrite(TRIG, HIGH);  // set trig pin to high for 10μs (generate ultrasonic sound wave)
        delayMicroseconds(10);

        digitalWrite(TRIG, LOW);    // set trig pin to low (stop generating ultrasonic sound wave)

        f_dist = pulseIn(ECHO, HIGH); // read echo pin (time for wave to bounce back)
        f_dist= f_dist/58;      // Y m=（X s*344）/2; X s=（ 2*Y m）/344 ==》X s=0.0058*Y m ==》cm = us /58

        *(measurements + i) = (int)f_dist;
    }

    sort(measurements, length);     // sort measurements array

    int distance = 0;   // set initial measurement as 0

    if(*measurements < 6)       // if minimum measurement is < 6cm, then get the minimum (for avoiding errors)
        distance = *measurements;
    else
        distance = *(measurements + (int)length/2);  // if the measurement is > 6, get the middle measurement

    if(verbose){    // print result

        if((distance < 3) || (distance > 400)){     // ultrasonic range ranging 3cm to 400cm

            Serial.println("ERROR! OUT OF RANGE! ==》Ultrasonic range: 3cm - 400cm");
        }
        else{
            Serial.println("Distance = ");  // print distance
            Serial.print(distance);
            Serial.println("cm");
        }
    }

    return distance;
}

void set_motor_speed(int speed_val){
    analogWrite(PWMA, speed_val);   // set speed for left motor
    analogWrite(PWMB, speed_val);   // set speed for right motor
}

void forward(int speed){
    set_motor_speed(speed);     // set motor speed
    digitalWrite(AIN1, LOW);    // set motor-L backward pin to low
    digitalWrite(AIN2, HIGH);   // set motor-L forward pin to high
    digitalWrite(BIN1, LOW);    // set motor-R backward pin to low
    digitalWrite(BIN2, HIGH);   // set motor-R forward pin to high
}

void backward(int speed){
    set_motor_speed(speed);     // set motor speed
    digitalWrite(AIN1, HIGH);   // set motor-L backward pin to high
    digitalWrite(AIN2, LOW);    // set motor-L forward pin to low
    digitalWrite(BIN1, HIGH);   // set motor-R backward pin to high
    digitalWrite(BIN2, LOW);    // set motor-R forward pin to low
}

void left(){
    set_motor_speed(70);        // set motor speed to 60
    digitalWrite(AIN1, HIGH);   // set motor-L backward pin to high
    digitalWrite(AIN2, LOW);    // set motor-L forward pin to low
    digitalWrite(BIN1, LOW);    // set motor-R backward pin to low
    digitalWrite(BIN2, HIGH);   // set motor-R forward pin to high
}

void right(){
    set_motor_speed(70);        // set motor speed to 60
    digitalWrite(AIN1, LOW);    // set motor-L backward pin to low
    digitalWrite(AIN2, HIGH);   // set motor-L forward pin to high
    digitalWrite(BIN1, HIGH);   // set motor-R backward pin to high
    digitalWrite(BIN2, LOW);    // set motor-R forward pin to low
}

void stay(){
    set_motor_speed(0);         // set motor speed to 0
    digitalWrite(AIN1, LOW);    // set motor-L backward pin to low
    digitalWrite(AIN2, LOW);    // set motor-L forward pin to low
    digitalWrite(BIN1, LOW);    // set motor-R backward pin to low
    digitalWrite(BIN2, LOW);    // set motor-R forward pin to low
}

void right_angle(int angle){
    int measure_time = millis();
    Serial.println(measure_time);

    if(measure_time <= (int)1100*(angle/360)) {
        Serial.println("Here");
        right();
    }

    else{
        stay();
    }

}