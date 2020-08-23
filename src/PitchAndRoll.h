//
// Created by Jamie Sherman on 8/22/20.
//

#ifndef LIDARLEVEL_PITCHANDROLL_H
#define LIDARLEVEL_PITCHANDROLL_H

#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

class PitchAndRoll {
#define BNO055_SAMPLERATE_DELAY_MS 100

    constexpr static const double m_g = 9.80665;
    static Adafruit_BNO055* m_sensor;
    imu::Vector<3> m_acc;
    imu::Vector<3> m_gyro;
    imu::Vector<3> m_mag;

    double m_theta;
    double m_phi;
    double m_theta_acc;
    double m_phi_acc;
    double m_theta_gyro;
    double m_phi_gyro;

    unsigned long m_old_time;
    double m_dt;

public:
    PitchAndRoll(unsigned long sleep_for = 1000);

    void update(void);
    double get_theta(void) { return m_theta; }
    double get_theta_radians(void);

    double get_phi(void) { return m_phi; }
    double get_phi_radians(void);

    void update_old(void);

    void pause(void) { delay(BNO055_SAMPLERATE_DELAY_MS); }

    bool is_calibrated(uint8_t sys, uint8_t acc, uint8_t gyro, uint8_t mag);
};

#endif //LIDARLEVEL_PITCHANDROLL_H
