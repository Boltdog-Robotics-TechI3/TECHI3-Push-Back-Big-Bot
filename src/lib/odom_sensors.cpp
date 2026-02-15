#include <cmath>
#include "lib/odom_sensors.hpp"

void OdomSensors::reset() {
    if (verticalWheel) {
        verticalWheel->reset();
    }
    if (horizontalWheel) {
        horizontalWheel->reset();
    }
    if (imu) {
        imu->reset(true);
        imu->tare();
    }
}

std::array<double, 3> OdomSensors::getReadings() {
    std::array<double, 3> readings = {0.0, 0.0, 0.0}; // vertical, horizontal, rotation
    if (verticalWheel) {
        readings[0] = verticalWheel->getDistance();
    }
    if (horizontalWheel) {
        readings[1] = horizontalWheel->getDistance();
    }
    if (imu) {
        readings[2] = imu->get_rotation() * (M_PI / 180.0); // convert degrees to radians
    }
    return readings;
}

double OdomSensors::getRotationRadians() {
    if (imu) {
        return imu->get_rotation() * (M_PI / 180.0); // convert degrees to radians
    }
    return 0.0;
}

double OdomSensors::getRotationDegrees() {
    if (imu) {
        return imu->get_rotation();
    }
    return 0.0;
}

double OdomSensors::getHeadingRadians() {
    if (imu) {
        return imu->get_heading();
    }
    return 0.0;
}
