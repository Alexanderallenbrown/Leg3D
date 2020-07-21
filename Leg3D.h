/*
Leg3D quadrupedal leg class

Alexander Brown, Ph. D.
July 2020
*/


#ifndef Leg3D_h
#define Leg3D_h

#include "Arduino.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>



class Leg3D
{
public:
    Leg3D(int side, int servozero_h, int servozero_f, int servozero_t, int servonum_h, int servonum_f, int servonum_t);
    void update(float xrel, float yrel, float zrel);
    float lf;
    float lt;
    float tht_offset;
    float thf_offset;
    float thh_offset;
    float zerox;
    float zeroz;
private:
    void servoAngles(float xrel, float yrel, float zrel);
    void rawAngles(float xrel, float yrel, float zrel);
    void writeServos();
    Adafruit_PWMServoDriver _pwm;
    int _side;
    float _servozero_h;
    float _servozero_f;
    float _servozero_t;
    int _servonum_h;
    int _servonum_f;
    int _servonum_t;
    float _thf_raw;
    float _tht_raw;
    float _thh_raw;
    float _thh;
    float _thf;
    float _tht;
    int USMIN; // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
    int USMAX; // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
    int SERVO_FREQ; // Analog servos run at ~50 Hz updates
};

#endif