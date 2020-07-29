/*
  Morse.cpp - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Leg3D.h"
#include <math.h>

Leg3D::Leg3D(int side, int diagonal, float servozero_h, float servozero_f, float servozero_t, int servonum_h, int servonum_f, int servonum_t)
{
  _side = side;
  _diagonal = diagonal;
  _servozero_h = servozero_h*3.14/180;
  _servozero_f = servozero_f*3.14/180;
  _servozero_t = servozero_t*3.14/180;
  _servonum_h = servonum_h;
  _servonum_f = servonum_f;
  _servonum_t = servonum_t;
  USMIN = 600;
  USMAX = 2400;
  SERVO_FREQ = 50;
  // _pwm = Adafruit_PWMServoDriver();
  lf = .044;
  lt = .071;
  tht_offset = 1.34;
  thf_offset = 2.36;
  zerox = .0065278;
  zeroz = -.091;

   // _pwm.begin();
   // _pwm.setOscillatorFrequency(27000000);
   // _pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
   // delay(10);


}

void Leg3D::attach(){
  // _pwm = Adafruit_PWMServoDriver();
  _pwm.begin();
  _pwm.setOscillatorFrequency(27000000);
  _pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  delay(10);
}

void Leg3D::rawAngles(float xrel, float yrel, float zrel)
{
  float x = xrel+zerox;
  float z = zrel+zeroz;
  //find hip angle, correcting for zeros
  if(z!=0){
    _thh_raw = atan(yrel/z);
  }
  else{
    if(yrel>=0){
      _thh_raw = 3.141592654;
    }
    else{
      _thh_raw = -3.141592654;
    }
  }
  //inverse kin calculations with intermediate vars
  float zp = z/(cos(_thh_raw));  // displacement in z on the x-z plane (accounts for hip angle)
  float d = sqrt(pow(zp,2)+pow(x,2));  // distance between foot and femur joint
  float thleg = atan2(zp,x);  // angle between x-axis and line from femur joint to foot (should be negative when foot is below the chassis)
  float opthlt = (pow(d,2)+pow(lf,2) - pow(lt,2))/(2*d*lf);
  //if opthlt is >1, it can't be used with acos.
  //if this happens, it means we're out of the workspace.
  //to avoid runtime problems, just go as far as we can by setting the cos to +/- 1
  if(abs(opthlt)>=1){
    // If the quantity's magnitude is larger than 1, it can't be evaluted
    // by arccos. Assigns it a value of 1 or -1
    opthlt = 1.0*opthlt/abs(opthlt);
  }
  float thlt = acos(opthlt);   // angle between femur and line from femur joint to foot
  _thf_raw = abs(thleg-thlt);  // angle of femur from x-axis (should be negative)

  float opthd = (pow(lt,2)+pow(lf,2)-pow(d,2))/(2*lt*lf);
  if(abs(opthd)>=1){
    // If the quantity's magnitude is larger than 1, it can't be evaluted
    // by arccos. Assigns it a value of 1 or -1
    opthd = 1*opthd/abs(opthd);
  }
  float thd = acos(opthd);  // angle between femur and tibia
  _tht_raw = 3.141592654-thd;  // supplementary angle

}

void Leg3D::servoAngles(float xrel, float yrel, float zrel)
{
  //nominally, servos are zeroed at 90, so there's that, and the angular offset of the limb at rest
  float off_femur = -(_servozero_f+thf_offset);
  float off_tibia = _servozero_t - tht_offset;
  //calculate angles including servo offsets
  _thf = -(_thf_raw+off_femur);
  _tht = _tht_raw + off_tibia;
  _thh = _thh_raw + _servozero_h;

  // Adjust for your robot
  if(_servonum_h == 2) {
      _thh = _thh + 2*3.141592654/180;
      _thf = _thf + 2*3.141592654/180;
      _tht = _tht - 11*3.141592654/180;
  }
  else if(_servonum_h == 5) {
      _thh = _thh - 8*3.141592654/180;
      _thf = _thf + 3*3.141592654/180;
      _tht = _tht - 2*3.141592654/180;
  }
  else if(_servonum_h == 8){
      _thh = _thh + 1*3.141592654/180; // consider increasing adjustment
      _thf = _thf + 7*3.141592654/180;
      _tht = _tht - 13*3.141592654/180;
  }
  else {
      _thh = _thh + 10*3.141592654/180;
      _thf = _thf + 10*3.141592654/180;
      _tht = _tht - 8*3.141592654/180;
  }

  // Adjust angles according to the servo's orientation
  if(_side==2){
    //servo is on the left side (inboard) or right (outboard)
    _thf = _thf;
    _tht = 3.141592654 - _tht;

    // _thh = 3.141592654 - _thh;
  }
  else{
    // servo is on the right side (inboard) or left (outboard)
    // _tht = _tht;
    _thf = 3.141592654 - _thf;
    _thh = _thh;
  }

  // Adjust hip angles according to the servo's orientation
  if(_diagonal == 2) {
    // FR and RL hips
    _thh = 3.141592654 - _thh;
  }
  else {
    // FL and RR hips
    _tht = _tht;
  }
}


void Leg3D::writeServos(){

  //convert angles (in radians) to microseconds for writing
  uint16_t us_h = (USMAX-USMIN)/3.1415*_thh+USMIN;
  uint16_t us_f = (USMAX-USMIN)/3.1415*_thf+USMIN;
  uint16_t us_t = (USMAX-USMIN)/3.1415*_tht+USMIN;

  // Serial.print(us_h);
  // Serial.print("\t");
  // Serial.print(us_f);
  // Serial.print("\t");
  // Serial.println(us_t);
  //write to servo driver
  _pwm.writeMicroseconds(_servonum_h, us_h);
  _pwm.writeMicroseconds(_servonum_f, us_f);
  _pwm.writeMicroseconds(_servonum_t, us_t);

}

void Leg3D::update(float xrel, float yrel, float zrel){
  rawAngles(xrel,yrel,zrel);
  servoAngles(xrel,yrel,zrel);
  writeServos();
}
