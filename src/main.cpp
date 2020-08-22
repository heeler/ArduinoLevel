#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

#define BNO055_SAMPLERATE_DELAY_MS 100

static const double G = 9.80665;
Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {
    // put your setup code here, to run once:
    Serial.begin(115200);
    myIMU.begin();
    delay(1000);
    int8_t temp = myIMU.getTemp();
    myIMU.setExtCrystalUse(true);
}

void loop() {
// write your code here
    // put your main code here, to run repeatedly:
    imu::Vector<3> acc =  myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//  imu::Vector<3> gyro =  myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//  imu::Vector<3> mag =  myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    uint8_t my_sys=0;
    uint8_t my_gyro=0;
    uint8_t my_accel=0;
    uint8_t my_mag=0;

    myIMU.getCalibration(&my_sys, &my_gyro, &my_accel, &my_mag);

    double theta = atan2(acc.x(),acc.z())*180.0/PI;
    double phi = atan2(acc.y(),acc.z())*180.0/PI;


    Serial.print(theta);
    Serial.print(", ");
    Serial.print(phi);
    Serial.print(", ");

//  Serial.print("(");
    Serial.print(my_accel);
    Serial.print(", ");
    Serial.print(my_gyro);
    Serial.print(", ");
    Serial.print(my_mag);
    Serial.print(", ");
    Serial.println(my_sys);

//  Serial.print(gyro.z());
//  Serial.print(")   ");
//
//  Serial.print("(");
//  Serial.print(mag.x());
//  Serial.print(", ");
//  Serial.print(mag.y());
//  Serial.print(", ");
//  Serial.print(mag.z());
//  Serial.println(")   ");

    delay(BNO055_SAMPLERATE_DELAY_MS);

}
