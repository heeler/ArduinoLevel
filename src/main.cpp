#include <Arduino.h>
#include <Wire.h>
#include "PitchAndRoll.h"

#define BNO055_SAMPLERATE_DELAY_MS 100

PitchAndRoll *controller;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    controller = new PitchAndRoll();
    Serial.println("Sensor Calibrated!");
}

void loop()
{
    controller->update();

    double theta = controller->get_theta();
    double phi = controller->get_phi();

//    Serial.print(theta);
//    Serial.print(" ");
//    Serial.println(phi);

    controller->update_old();
    controller->pause();

}
