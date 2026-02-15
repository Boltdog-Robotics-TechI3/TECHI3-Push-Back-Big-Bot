#include <cmath>
#include "lib/tank_chassis.hpp"
#include "pros/rtos.hpp"

/**
 * @brief Move the robot in arcade mode. The left joystick controls the forward/backward movement, and the right joystick controls the rotation.
 * @param leftY The value of the left joystick (forward/backward movement).
 * @param rightX The value of the right joystick (rotation).
 */
void TankChassis::arcade(int leftY, int rightX) {
    if (drivetrain) {
        int scaledLeftY = scaleInput(leftY);
        int scaledRightX = scaleInput(rightX);
        int leftPower = scaledLeftY + scaledRightX;
        int rightPower = scaledLeftY - scaledRightX;
        drivetrain->setMotorSpeeds({leftPower, rightPower});
    }
}

/**
 * @brief Move the robot in tank mode. The left joystick controls the left side motors, and the right joystick controls the right side motors.
 * @param leftY The value of the left joystick (left side motors).
 * @param rightY The value of the right joystick (right side motors).
 */
void TankChassis::tank(int leftY, int rightY) {
    if (drivetrain) {
        int scaledLeftY = scaleInput(leftY);
        int scaledRightY = scaleInput(rightY);
        drivetrain->setMotorSpeeds({scaledLeftY, scaledRightY});
    }
}

/**
 * @brief Move the robot to a specific position using PID control. This method blocks until the target position is reached.
 * 
 * @note This method ignores the angle of the target pose and only drives to the x and y coordinates.
 * @note THIS FUNCTION IS NOT A REPLACEMENT FOR TURNING. ALWAYS TURN TO FACE THE TARGET POSE BEFORE USING THIS FUNCTION.
 * 
 * @param targetPose The target pose to move to.
 * @param timeout The amount of time in milliseconds that the robot will try to reach the pose before giving up
 * @param maxSpeed The maximum speed the robot can travel, from 0 to 127
 */
void TankChassis::moveToPose(const Pose& targetPose, int timeout, int maxSpeed) {
    if (!movePID || !alignPID) {
        return;
    }

    isAtSetpoint = false;

    // Calculate starting angular error to determine if robot should drive forwards or backwards
    float testAngleError = (pose->angleTo(targetPose) - M_PI_2) - fmod((pose->getTheta() + 2*M_PI), 2*M_PI);
    if (testAngleError > M_PI) {
        testAngleError = testAngleError - 2*M_PI;
    } 
    else if (testAngleError < -M_PI) {
        testAngleError = 2*M_PI + testAngleError;
    }

    // If error is greater than PI/2, we are probably trying to go backwards (hopefully).
    bool isBackwards = abs(testAngleError) >= M_PI_2;

    float error = 0;
    float angularError = 0;
    int linearOutput = 0;
    int angularOutput = 0;

    movePID->reset();
    alignPID->reset();

    movePID->setOutputLimits(-maxSpeed, maxSpeed);
    movePID->setSmallErrorRange(.5);
    movePID->setLargeErrorRange(1.25);
    movePID->setSlewRate(200);

    Timer timeoutTimer(timeout, +[]() { Chassis::isAtSetpoint = true; });
    Timer smallErrorTimer(500, +[]() { Chassis::isAtSetpoint = true; });
    Timer largeErrorTimer(1000, +[]() { Chassis::isAtSetpoint = true; });

    timeoutTimer.start();
    while (!isAtSetpoint) {
        error = pose->distanceTo(targetPose);
        linearOutput = movePID->calculate(0, error) * (isBackwards ? -1 : 1);

        if (error > minAlignDistance) {
            // Calculate and normalize the angle error
            angularError = (pose->angleTo(targetPose) + (isBackwards ? M_PI : 0) - M_PI_2) - fmod((pose->getTheta() + 2*M_PI), 2*M_PI);
            if (angularError > M_PI) {
                angularError = angularError - 2*M_PI;
            } 
            else if (angularError < -M_PI) {
                angularError = 2*M_PI + angularError;
            }

            angularOutput = alignPID->calculate(0, angularError);

            drivetrain->setMotorSpeeds({linearOutput - angularOutput, linearOutput + angularOutput});
            // std::cout << "linear: " << linearOutput << "; align: " << angularOutput << std::endl;
        }
        else {
            drivetrain->setMotorSpeeds({linearOutput, linearOutput});
        }

        if (movePID->isInSmallErrorRange()) {
            if (!smallErrorTimer.isRunning()) {
                smallErrorTimer.start();
            }
        } else {
            smallErrorTimer.stop();
        }
        if (movePID->isInLargeErrorRange()) {
            if (!largeErrorTimer.isRunning()) {
                largeErrorTimer.start();
            }
        } else {
            largeErrorTimer.stop();
        }  
        
		pros::delay(20);
    }

    smallErrorTimer.stop();
    largeErrorTimer.stop();
    timeoutTimer.stop();
    stop();
}

/**
 * @brief Drive the robot a specified distance. Positive distance drives forwards,
 * negative distance drives backwards.
 * 
 * @param distance the distance to drive in inches.
 */
void TankChassis::moveDistance(double distance, int timeout) {
    if (!movePID || !alignPID) {
        return;
    }

    isAtSetpoint = false;

    Timer timeoutTimer(timeout, +[]() { Chassis::isAtSetpoint = true; });
    Timer smallErrorTimer(500, +[]() { Chassis::isAtSetpoint = true; });
    Timer largeErrorTimer(2000, +[]() { Chassis::isAtSetpoint = true; });

    timeoutTimer.start();
    double initialPosition = odometry ? odometry->getReadings()[0] : (drivetrain->getMotors()[0]->get_position() + drivetrain->getMotors()[1]->get_position()) / 2.0;
    
    movePID->reset();
    alignPID->reset();

    movePID->setOutputLimits(-50, 50);
    movePID->setSmallErrorRange(1);
    movePID->setLargeErrorRange(2);
    movePID->setSlewRate(300);

    alignPID->setOutputLimits(-10, 10);

    while (!isAtSetpoint) {
        double currentPosition = odometry ? odometry->getReadings()[0] : (drivetrain->getMotors()[0]->get_position() + drivetrain->getMotors()[1]->get_position()) / 2.0;

        if (movePID->isInSmallErrorRange()) {
            smallErrorTimer.start();
        }
        else if (movePID->isInLargeErrorRange()) {
            smallErrorTimer.stop();
            largeErrorTimer.start();
        }
        else {
            smallErrorTimer.stop();
            largeErrorTimer.stop();
        }

        int output = movePID->calculate(currentPosition-initialPosition, distance);

        drivetrain->setMotorSpeeds({output, output});

        pros::delay(20);
    }

    timeoutTimer.stop();
    smallErrorTimer.stop();
    largeErrorTimer.stop();
    stop();
}

/**
 * @brief Turn the robot to a specific angle using PID control.
 * 0 Degrees is facing "forward" from the starting orientation. Counterclockwise is positive
 * 
 * @param targetAngle The target angle to turn to (in degrees).
 * @param timeout The amount of time in milliseconds that the robot will try to reach the angle before giving up
 */
void TankChassis::turnToAngle(double targetAngle, int timeout) {
    if (!turnPID) {
        return;
    }

    isAtSetpoint = false;

    Timer timeoutTimer(timeout, +[]() { Chassis::isAtSetpoint = true; }); 
    Timer smallErrorTimer(500, +[]() { Chassis::isAtSetpoint = true; });
    Timer largeErrorTimer(1500, +[]() { Chassis::isAtSetpoint = true; });

    turnPID->reset();
    turnPID->setOutputLimits(-45, 45);
    turnPID->setSmallErrorRange(0.02);
    turnPID->setLargeErrorRange(0.08);
    turnPID->setIZone(.5);

    timeoutTimer.start();

    while (!isAtSetpoint) {
        double error = Pose::degToRad(targetAngle) - fmod((pose->getTheta() + 2*M_PI), 2*M_PI);
        if (error > M_PI) {
            error = error - 2*M_PI;
        } 
        else if (error < -M_PI) {
            error = 2*M_PI + error;
        }

        int output = turnPID->calculate(0, error);
        drivetrain->setMotorSpeeds({-output, output});

        if (turnPID->isInSmallErrorRange()) {
            smallErrorTimer.start();
        }
        else if (turnPID->isInLargeErrorRange()) {
            smallErrorTimer.stop();
            largeErrorTimer.start();
        }
        else {
            smallErrorTimer.stop();
            largeErrorTimer.stop();
        }

        pros::delay(20);
    }
    smallErrorTimer.stop();
    largeErrorTimer.stop();
    timeoutTimer.stop();
    stop();
}
