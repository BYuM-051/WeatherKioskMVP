#include <Servo.h>

Servo pan, tilt;

const int PAN_PIN  = 9;
const int TILT_PIN = 10;

void setup() 
{
    Serial.begin(115200);
    pan.attach(PAN_PIN);
    tilt.attach(TILT_PIN);
}

void loop() 
{
    if (Serial.available()) 
    {
        String line = Serial.readStringUntil('\n');
        int pIdx = line.indexOf("P:");
        int tIdx = line.indexOf("T:");

        if (pIdx != -1) 
        {
            int comma = line.indexOf(',', pIdx);
            String v = line.substring(pIdx + 2, (comma == -1 ? line.length() : comma));
            int panAngle = v.toInt();
            pan.write(panAngle);
        }

        if (tIdx != -1) 
        {
            int comma = line.indexOf(',', tIdx);
            String v = line.substring(tIdx + 2, (comma == -1 ? line.length() : comma));
            int tiltAngle = v.toInt();
            tilt.write(tiltAngle);
        }
    }
}