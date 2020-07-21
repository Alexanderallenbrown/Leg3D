#include <Leg3D.h>

//amplitude of sine wave in meters
float amp = .01252;
//frequency of sine wave in rad/s
float freq = 6;


//set up for the Right Front leg:
//function prototype is Leg3d(side, hip_center, femur_center, tibia_center, hip_num, femur_num, tibia_num)
//side is 2 for left, 1 for right
Leg3D leg = Leg3D(1, 90,90,90,0,1,2);

void setup(){
    Serial.begin(115200);
    leg.attach();
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