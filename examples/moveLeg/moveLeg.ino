#include <Leg3D.h>

//amplitude of sine wave in meters
float amp = .02;
//frequency of sine wave in rad/s
float freq = 6;

Leg3D leg(1, 90,90,90,0,1,2);

void setup(){
    Serial.begin(115200);
}

void loop(){
    //get current time
    float t = millis()/1000.0;

    float x = 0;
    float y = 0;
    float z = amp*sin(freq*t);

    leg.update(x,y,z);

    Serial.print(x);
    Serial.print("\t");
    Serial.print(y);
    Serial.print("\t");
    Serial.print(z);
    Serial.println();
}