/*
  Morse.cpp - Library for flashing Morse code.
  Created by David A. Mellis, November 2, 2007.
  Released into the public domain.
*/

#include "Arduino.h"
#include "Leg3D.h"
#include <math.h>

Leg3D::Leg3D(int side, float servozero_h, float servozero_f, float servozero_t, int servonum_h, int servonum_f, int servonum_t)
{
  _side = side;
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
  float zp = z/(cos(_thh_raw));
  float d = sqrt(pow(zp,2)+pow(x,2));
  float thleg = atan2(zp,x);
  float opthlt = (pow(d,2)+pow(lf,2) - pow(lt,2))/(2*d*lf);
  //if opthlt is >1, it can't be used with acos.
  //if this happens, it means we're out of the workspace.
  //to avoid runtime problems, just go as far as we can by setting the cos to +/- 1
  if(abs(opthlt)>=1){
    opthlt = 1.0*opthlt/abs(opthlt);
  }
  float thlt = acos(opthlt);
  _thf_raw = abs(thleg-thlt);

  float opthd = (pow(lt,2)+pow(lf,2)-pow(d,2))/(2*lt*lf);
  if(abs(opthd)>1){
    opthd = 1*opthd/abs(opthd);
  }
  float thd = acos(opthd);
  _tht_raw = 3.141592654-thd;

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

  if(_side==2){
    //servo is on the left side
    _thf = _thf;
    _tht = 3.141592654 - _tht;
    _thh = 3.141592654 - _thh;
  }
  else{
    // servo is on the right side
    _tht = _tht;
    _thf = 3.141592654 - _thf;
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
