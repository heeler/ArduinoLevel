//
// Created by Jamie Sherman on 8/22/20.
//

#include "PitchAndRoll.h"
#include <math.h>

Adafruit_BNO055 *PitchAndRoll::m_sensor = new Adafruit_BNO055();

void printThree(double a, double b, double c){
    Serial.print(a); Serial.print(" ");
    Serial.print(b); Serial.print(" ");
    Serial.println(c);
}

PitchAndRoll::PitchAndRoll(unsigned long sleep_for){
    m_sensor->begin();
    delay(sleep_for);
    m_sensor->setExtCrystalUse(true);
    pause();
    uint8_t sys=0, acc=0, gyro=0, mag=0;
    m_sensor->getCalibration(&sys, &gyro, &acc, &mag);
    pause();
    while (!this->is_calibrated(sys, acc, gyro, mag)){
        m_sensor->getCalibration(&sys, &gyro, &acc, &mag);
        Serial.print(sys);
        Serial.print(" ");
        Serial.print(gyro);
        Serial.print(" ");
        Serial.print(mag);
        Serial.print(" ");
        Serial.println(acc);
        pause();
    }
    update_old();
}

void
PitchAndRoll::update() {

    m_acc = m_sensor->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    m_gyro = m_sensor->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    m_mag =  m_sensor->getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    m_theta_acc = -atan2(m_acc.x(), m_acc.z())*180.0/PI;
    m_phi_acc = -atan2(m_acc.y(), m_acc.z())*180.0/PI;

    m_dt = (millis() - m_old_time)/1000.0;

    m_theta_gyro = m_theta + m_gyro.y()*m_dt;
    m_phi_gyro =  m_phi - m_gyro.x()*m_dt;

    m_theta = 0.95*m_theta_gyro + 0.05*m_theta_acc;
    m_phi = 0.95*m_phi_gyro + 0.05*m_phi_acc;

//    printThree(m_phi_acc, m_phi_gyro, m_phi);

}



double
PitchAndRoll::get_theta_radians() { return m_theta*PI/180.0; }

double
PitchAndRoll::get_phi_radians() { return m_phi*PI/180.0; }

void
PitchAndRoll::update_old(void){
    m_old_time = millis();
}
bool PitchAndRoll::is_calibrated(uint8_t sys, uint8_t acc, uint8_t gyro, uint8_t mag)
{
    //return true;
    if(acc < 3 || gyro < 3 || mag < 3 || sys < 3 ) return false;
    return true;
}


