//
// Created by Blagoj Hristov on 26/02/20.
//

#include <alphabot2_asi.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TRSensors.h>
//#include <Adafruit_NeoPixel.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define ECHO   2        // ultrasonic echo
#define TRIG   3        // ultrasonic trigger
#define PWMA   6        // motor-L speed (ENA)
#define AIN2   A0       // motor-L forward (IN2)
#define AIN1   A1       // motor-L backward (IN1)
#define PWMB   5        // motor-R speed (ENB)
#define BIN1   A2       // motor-R forward (IN3)
#define BIN2   A3       // motor-R backward (IN4)
#define PCFADR  0x20    // PCF8574 I/O expansion module address (for forward facing infrared sensors)
#define LED_PIN 7       // LEDs pin
#define OLED_RESET 9    // OLED display reset pin
#define OLED_SA0   8    // OLED display set pin

Adafruit_SSD1306 display(OLED_RESET, OLED_SA0);
//Adafruit_NeoPixel RGB = Adafruit_NeoPixel(4, LED_PIN, NEO_GRB + NEO_KHZ800);    // setup RGB LEDs

TRSensors line_sensors = TRSensors();       // declare line sensors as TRSensor type object
unsigned int line_sensor_values[5];         // declare global array for line sensor values

float I = 0;                                // declare initial value for integral of error for PID
float time_prev = 0;                        // declare initial value of previous time measurement for PID
int line_pos_prev = 0;                      // declare initial value for previous line position for PID

long int time_start;   // declare start time variable
long int time_end;     // declare end time variable
long int time_elapsed; // declare elapsed time variable

void initialize(){
    Serial.begin(115200);
    Wire.begin();
    pinMode(ECHO, INPUT);    // define the ultrasonic echo input pin
    pinMode(TRIG, OUTPUT);   // define the ultrasonic trigger input pin
    pinMode(PWMA, OUTPUT);   // define the left motor speed pin
    pinMode(AIN2, OUTPUT);   // define the left motor forward pin
    pinMode(AIN1, OUTPUT);   // define the left motor backward pin
    pinMode(PWMB, OUTPUT);   // define the right motor speed pin
    pinMode(BIN2, OUTPUT);   // define the right motor forward pin
    pinMode(BIN1, OUTPUT);   // define the right motor backward pin
    
    // Uncomment the calibrated_max_vals and calibrated_min_vals for YOUR robot only!
	
    // Bot 1:
    int calibrated_max_vals[5] = {937, 934, 891, 762, 955};
    int calibrated_min_vals[5] = {187, 205, 156, 138, 292};

    // Bot 2:
    //int calibrated_max_vals[5] = {723, 758, 941, 945, 843};
    //int calibrated_min_vals[5] = {150, 164, 171, 135, 133};

    // Bot 3:
    //int calibrated_max_vals[5] = {704, 686, 626, 958, 797};
    //int calibrated_min_vals[5] = {134, 109, 116, 260, 149};

    // Bot 4:
    //int calibrated_max_vals[5] = {594, 678, 632, 552, 758};
    //int calibrated_min_vals[5] = {150, 164, 171, 135, 133};

    // Bot 5:
    //int calibrated_max_vals[5] = {753, 706, 764, 940, 708};
    //int calibrated_min_vals[5] = {117, 123, 126, 227, 127};

    // Bot 6:
    //int calibrated_max_vals[5] = {776, 586, 706, 552, 745};
    //int calibrated_min_vals[5] = {228, 158, 184, 134, 222};

    // Bot 7:
    //int calibrated_max_vals[5] = {853, 932, 589, 913, 881};
    //int calibrated_min_vals[5] = {143, 225, 160, 152, 139};

    // Bot 8:
    //int calibrated_max_vals[5] = {813, 787, 690, 573, 689};
    //int calibrated_min_vals[5] = {199, 218, 165, 161, 186};

    // Bot 9:
    //int calibrated_max_vals[5] = {953, 817, 880, 770, 803};
    //int calibrated_min_vals[5] = {288, 208, 239, 212, 222};

    // Bot 10 (X):
    //int calibrated_max_vals[5] = {941, 918, 751, 953, 698};
    //int calibrated_min_vals[5] = {270, 237, 232, 363, 180};

    // Bot 11:
    //int calibrated_max_vals[5] = {864, 629, 724, 746, 952};
    //int calibrated_min_vals[5] = {204, 160, 181, 158, 278};

    // Bot 12:
    //int calibrated_max_vals[5] = {681, 940, 817, 959, 596};
    //int calibrated_min_vals[5] = {177, 264, 236, 269, 170};

    for(int i = 0; i < 5; i++) {
     	line_sensors.calibratedMax[i] = calibrated_max_vals[i];
	line_sensors.calibratedMin[i] = calibrated_min_vals[i];
    }

    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize OLED display
    oled_display_name();                      // write competition name on OLED display (scrolling)

    time_start = millis();                      // read initial start time in milliseconds 
}

void oled_display_name(){
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,2);
  display.clearDisplay();
  display.println("RoboMac");
  display.setCursor(6,20);
  display.println("Junior");
  display.setCursor(17,40);
  display.println("2022");
  display.display();
 
  display.startscrollright(0x00, 0x0F);
}

void oled_display_markers(int mrk){
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,2);
  display.clearDisplay();
  display.println(mrk);
  display.display();
}

void oled_display_time(long int ts){
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0,2);
  display.clearDisplay();
  display.println("Time:");
  display.setCursor(6,20);
  display.print(ts/1000);
  display.print(" s");
  display.display();
}

void write_PCF8574(byte data){      // write to the PCF8574 I/O expansion module
    Wire.beginTransmission(PCFADR);
    Wire.write(data);
    Wire.endTransmission();
}


byte read_PCF8574(){    // read from the PCF8574 I/O expansion module
    int data = -1;
    Wire.requestFrom(PCFADR, 1);
    if(Wire.available()) {
        data = Wire.read();
    }
    return data;
}


char read_infrared(bool verbose=false){
    byte value;                             // declare sensor value as byte type variable
    write_PCF8574(0xC0 | read_PCF8574());   // write sensor output to PCF8574 I/O expansion module
    value = read_PCF8574() | 0x3F;          // read PCF module (sensor output)

    if(verbose){    // print result
        if(value == 0x7F)
            Serial.println("Left sensor has detected an obstacle.");

        else if(value == 0xBF)
            Serial.println("Right sensor has detected an obstacle.");

        else if(value == 0x3F)
            Serial.println("Both sensors have detected an obstacle.");

        else if(value == 0xFF)
            Serial.println("No obstacle has been detected.");
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


void sort(int *arr, int n){     // function for sorting arrays
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


int read_ultrasonic(bool verbose=false){
    int length = 15;                            // number of measurements
    int measurements[length];
    float f_dist;

    for(int i = 0; i < length; i++){            // measure 'length' times for better accuracy
        digitalWrite(TRIG, LOW);                // set trig pin to low for 2μs (reset trigger)
        delayMicroseconds(2);

        digitalWrite(TRIG, HIGH);               // set trig pin to high for 10μs (generate ultrasonic sound wave)
        delayMicroseconds(10);

        digitalWrite(TRIG, LOW);                // set trig pin to low (stop generating ultrasonic sound wave)

        f_dist = pulseIn(ECHO, HIGH);           // read echo pin (time for wave to bounce back)
        f_dist= f_dist/58;                      // Y m=（X s*344）/2; X s=（ 2*Y m）/344 ==》X s=0.0058*Y m ==》cm = us /58

        *(measurements + i) = (int)f_dist;
    }

    sort(measurements, length);                 // sort measurements array

    int distance = 0;                           // set initial measurement to 0

    if(*measurements < 6)                       // if minimum measurement is < 6cm, then get the minimum (for avoiding errors)
        distance = *measurements;
    else
        distance = *(measurements + (int)length/2);     // if the measurement is > 6cm, get the middle measurement

    if(verbose){    // print result

        if((distance < 3) || (distance > 400)){         // ultrasonic range ranging 3cm to 400cm

            Serial.println("ERROR! OUT OF RANGE! ==》Ultrasonic range: 3cm - 400cm");
        }
        else{
            Serial.println("Distance = ");
            Serial.print(distance);
            Serial.println("cm");
        }
    }

    return distance;
}


// This function is strictly for calibrating the line sensors and should NOT to be used!
void calibrate_line_sensors(){

    for (int i = 0; i < 100; i++)  // make the calibration take about 10 seconds
    {
        line_sensors.calibrate();        // reads all sensors 100 times
    	line_sensors.readCalibrated(line_sensor_values);
	for(int i = 0; i < 5; i++){
        	Serial.print(line_sensor_values[i]);
        	Serial.print(" ");
    	}
	Serial.println();
	delay(100);
    }
    Serial.print("Calibrated max values: ");
    for(int i = 0; i < 5; i++){ 
        Serial.print(line_sensors.calibratedMax[i]);
        Serial.print(" ");
    }

    Serial.println();
    Serial.print("Calibrated min values: ");
    for(int i = 0; i < 5; i++){    
        Serial.print(line_sensors.calibratedMin[i]);
        Serial.print(" ");
    }
    Serial.println(); 
}


unsigned int *read_infrared_line(bool verbose=false){
    line_sensors.readLine(line_sensor_values);      // read sensor values

    if(verbose){    // print result
        Serial.print("Line sensor values: ");
        for (unsigned char i = 0; i < 5; i++)
        {
            Serial.print(line_sensor_values[i]);
            Serial.print('\t');
        }
        Serial.println();
    }
    return line_sensor_values;
}


// This function is for PID controlled line following, is not finished, and should NOT be used!
void line_follower(unsigned int *s, const float Kp, const float Ki, const float Kd, const int max_speed){
    unsigned int line_pos = line_sensors.readLine(line_sensor_values);    // read sensor values

    float time = millis();
    float dt = time - time_prev;

    int error = 2000 - (int)line_pos;       // calculate error in accordance to middle of line

    float P = error * Kp;                           // calculate proportional part of PID controller
    I += error * dt * Ki;                           // calculate integral part of PID controller
    float D = (line_pos - line_pos_prev)/dt * Kd;   // calculate derivative part of PID controller

    time_prev = time;
    line_pos_prev = line_pos;

    int control_signal = (int)(P + I + D);  // calculate PID controller

    if (control_signal > max_speed)         // regulate motor speed to be below set maximum
        control_signal = max_speed;

    if (control_signal < -max_speed)        // regulate motor speed to be non-negative
        control_signal = -max_speed;

    Serial.println(control_signal);         // print control signal value

    if (control_signal < 0)
    {
        analogWrite(PWMA, max_speed + control_signal);
        analogWrite(PWMB, max_speed);
    }

    else
    {
        analogWrite(PWMA, max_speed);
        analogWrite(PWMB, max_speed - control_signal);
    }

    if (s[1] > 750 && s[2] > 750 && s[3] > 750)     // check if line is still detectable
    {
        stay();
        Serial.println("Line not detected!");
    }
}


void set_motor_speed(int speed){
    analogWrite(PWMA, speed);   // set speed for left motor
    analogWrite(PWMB, speed);   // set speed for right motor
}


void set_left_motor_speed(int speed){
    if(speed > 180)             // max allowed speed of 180 for robot safety
        speed = 180;

    analogWrite(PWMA, speed);   // set speed for left motor
}


void set_right_motor_speed(int speed){
    if(speed > 180)             // max allowed speed of 180 for robot safety
        speed = 180;

    analogWrite(PWMB, speed);   // set speed for right motor
}


void forward(int speed){
    if(speed > 180)             // max allowed speed of 180 for robot safety
        speed = 180;

    set_motor_speed(speed);     // set motor speed
    digitalWrite(AIN1, LOW);    // set motor-L backward pin to low
    digitalWrite(AIN2, HIGH);   // set motor-L forward pin to high
    digitalWrite(BIN1, LOW);    // set motor-R backward pin to low
    digitalWrite(BIN2, HIGH);   // set motor-R forward pin to high
}


void backward(int speed){
    if(speed > 180)             // max allowed speed of 180 for robot safety
        speed = 180;

    set_motor_speed(speed);     // set motor speed
    digitalWrite(AIN1, HIGH);   // set motor-L backward pin to high
    digitalWrite(AIN2, LOW);    // set motor-L forward pin to low
    digitalWrite(BIN1, HIGH);   // set motor-R backward pin to high
    digitalWrite(BIN2, LOW);    // set motor-R forward pin to low
}


void left(int speed){
    if(speed > 180)             // max allowed speed of 180 for robot safety
        speed = 180;

    set_motor_speed(speed);     // set motor speed
    digitalWrite(AIN1, HIGH);   // set motor-L backward pin to high
    digitalWrite(AIN2, LOW);    // set motor-L forward pin to low
    digitalWrite(BIN1, LOW);    // set motor-R backward pin to low
    digitalWrite(BIN2, HIGH);   // set motor-R forward pin to high
}


void right(int speed){
    if(speed > 180)             // max allowed speed of 180 for robot safety
        speed = 180;

    set_motor_speed(speed);     // set motor speed
    digitalWrite(AIN1, LOW);    // set motor-L backward pin to low
    digitalWrite(AIN2, HIGH);   // set motor-L forward pin to high
    digitalWrite(BIN1, HIGH);   // set motor-R backward pin to high
    digitalWrite(BIN2, LOW);    // set motor-R forward pin to low
}


void stay(){
    set_motor_speed(0);                     // set motor speed to 0
    digitalWrite(AIN1, LOW);                // set motor-L backward pin to low
    digitalWrite(AIN2, LOW);                // set motor-L forward pin to low
    digitalWrite(BIN1, LOW);                // set motor-R backward pin to low
    digitalWrite(BIN2, LOW);                // set motor-R forward pin to low
    time_end = millis();                    // read final end time in milliseconds
    time_elapsed = time_end - time_start;   // calculate elapsed time in milliseconds

    oled_display_time(time_elapsed);        // display elapsed time on OLED display
    delay(100000);
}


/*void lights(int led_1[3], int led_2[3], int led_3[3], int led_4[3]){
    RGB.begin();

    RGB.setPixelColor(0, RGB.Color(led_1[0], led_1[1], led_1[2]));      // set colour of first LED
    RGB.setPixelColor(1, RGB.Color(led_2[0], led_2[1], led_2[2]));      // set colour of second LED
    RGB.setPixelColor(2, RGB.Color(led_3[0], led_3[1], led_3[2]));      // set colour of third LED
    RGB.setPixelColor(3, RGB.Color(led_4[0], led_4[1], led_4[2]));      // set colour of fourth LED

    RGB.show();     // turn on LEDs
}*/
