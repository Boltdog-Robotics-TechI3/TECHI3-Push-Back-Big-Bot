#include <cmath>
#include "lib/differentialchassis.hpp"
#include "pros/rtos.hpp"

/**
 * @brief Move the robot in arcade mode. The left joystick controls the forward/backward movement, and the right joystick controls the rotation.
 * @param leftY The value of the left joystick (forward/backward movement).
 * @param rightX The value of the right joystick (rotation).
 */
void DifferentialChassis::arcade(int leftY, int rightX) {
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
void DifferentialChassis::tank(int leftY, int rightY) {
    if (drivetrain) {
        int scaledLeftY = scaleInput(leftY);
        int scaledRightY = scaleInput(rightY);
        drivetrain->setMotorSpeeds({scaledLeftY, scaledRightY});
    }
}

/**
 * @brief Move the robot towards a specific position using a single step of PID control.
 * 
 * @note This method is intended to be called repeatedly in a loop until the target position is reached.
 * Use moveToPose() for a blocking call that handles the loop internally and if the target pose won't change during the loop.
 * Use this method if your target position may change dynamically.
 * 
 * @note This method ignores the angle of the target pose and only drives to the x and y coordinates.
 * 
 * @param targetPose The target pose to move to.
 */
void DifferentialChassis::moveToPoseStep(const Pose& targetPose, bool isForward) {    
    double linearError = pose->distanceTo(targetPose);

    double absTargetAngle = pose->angleTo(targetPose) + (isForward ? 0 : M_PI);
    absTargetAngle = absTargetAngle < 0 ? absTargetAngle + M_TWOPI : absTargetAngle;
    
    double angularError = absTargetAngle - getWorldFrameHeading();
    if (angularError > M_PI or angularError < (-1 * M_PI)) {
        angularError = -1 * std::copysign(1, angularError) * (M_TWOPI - abs(angularError));
    }

    double lateralOutput = (lateralPID->calculate(linearError, 0) - 10) * (isForward ? -1 : 1);
    double turnOutput = -alignPID->calculate(angularError, 0);

    // std::cout << "Linear Error: " << linearError << " | Angular Error: " << Pose::radToDeg(angularError) << " | Lateral Output: " << lateralOutput << " | Turn Output: " << turnOutput << std::endl;
        
    double leftOutput = lateralOutput - turnOutput;
    double rightOutput = lateralOutput + turnOutput;

    tank(leftOutput, rightOutput);
}

/**
 * @brief Move the robot to a specific position using PID control.
 * @param targetPose The target pose to move to.
 */
void DifferentialChassis::moveToPose(const Pose& targetPose, bool isForward) {
    lateralPID->reset();
    alignPID->reset();

    while (pose->distanceTo(targetPose) > .5) {
        moveToPoseStep(targetPose, isForward);

		pros::delay(20);
    }
}

/**
 * @brief Drives the robot until its x coordinate matches the given target
 * 
 * @param x the x coordinate to drive to, in inches
 */
void DifferentialChassis::driveToX(double x) {
    isAtSetpoint = false;

    Timer timeout(5000, +[]() { Chassis::isAtSetpoint = true; }); 
    Timer smallTimer(500, +[]() { Chassis::isAtSetpoint = true; });
    Timer largeTimer(1500, +[]() { Chassis::isAtSetpoint = true; });

    double currentAngle = pose->getTheta();

    lateralPID->reset();
    alignPID->reset();

    lateralPID->setOutputLimits(-70, 70);
    lateralPID->setSmallErrorRange(1);
    lateralPID->setLargeErrorRange(2);

    alignPID->setOutputLimits(-10, 10);

    timeout.start();

    while (!isAtSetpoint) {
        int output = lateralPID->calculate(pose->getX(), x) + alignPID->calculate(pose->getTheta(), currentAngle);
        drivetrain->setMotorSpeeds({output, output});

        if (lateralPID->isInSmallErrorRange()) {
            smallTimer.start();
        }
        else if (lateralPID->isInLargeErrorRange()) {
            smallTimer.stop();
            largeTimer.start();
        }
        else {
            smallTimer.stop();
            largeTimer.stop();
        }

        pros::delay(20);

    }
    smallTimer.stop();
    largeTimer.stop();
    timeout.stop();
    drivetrain->setMotorSpeeds({0, 0});
}

/**
 * @brief Drives the robot until its x coordinate matches the given target
 * 
 * @param x the x coordinate to drive to, in inches
 */
void DifferentialChassis::driveToY(double y) {
    isAtSetpoint = false;

    Timer timeout(10000, +[]() { Chassis::isAtSetpoint = true; }); 
    Timer smallTimer(500, +[]() { Chassis::isAtSetpoint = true; });
    Timer largeTimer(1500, +[]() { Chassis::isAtSetpoint = true; });

    double currentAngle = pose->getTheta();

    lateralPID->reset();
    alignPID->reset();

    lateralPID->setOutputLimits(-70, 70);
    lateralPID->setSmallErrorRange(1);
    lateralPID->setLargeErrorRange(2);

    alignPID->setOutputLimits(-10, 10);

    timeout.start();

    while (!isAtSetpoint) {
        int output = lateralPID->calculate(pose->getY(), y) + alignPID->calculate(pose->getTheta(), currentAngle);
        drivetrain->setMotorSpeeds({output, output});

        if (lateralPID->isInSmallErrorRange()) {
            smallTimer.start();
        }
        else if (lateralPID->isInLargeErrorRange()) {
            smallTimer.stop();
            largeTimer.start();
        }
        else {
            smallTimer.stop();
            largeTimer.stop();
        }

        pros::delay(20);
    }
    smallTimer.stop();
    largeTimer.stop();
    timeout.stop();
    drivetrain->setMotorSpeeds({0, 0});
}

/**
 * @brief Turns to face the pose, then drives in a straight line to the pose.
 * 
 * @param targetPose the pose to drive to
 */
void DifferentialChassis::turnThenMoveToPose(const Pose& targetPose, const bool isForward) {
    double targetAngle = pose->angleTo(targetPose) - (M_PI / 2);
    targetAngle += isForward ? 0 : M_PI;
    
    // std::cout << "Target Angle: " << targetAngle << std::endl;
    // std::cout << "Current Angle: " << targetAngle << std::endl;
    // std::cout << "Error " << fabs(pose->getTheta() - targetAngle) << std::endl;

    // Dynamic Turn Angle
    if (fabs(pose->getTheta() - targetAngle) > .01) {
        isAtSetpoint = false;

        Timer timeoutTurn(5000, +[]() { Chassis::isAtSetpoint = true; }); 
        Timer smallTimerTurn(500, +[]() { Chassis::isAtSetpoint = true; });
        Timer largeTimerTurn(1500, +[]() { Chassis::isAtSetpoint = true; });

        turnPID->reset();
        turnPID->setOutputLimits(-45, 45);
        turnPID->setSmallErrorRange(0.02);
        turnPID->setLargeErrorRange(0.08);
        turnPID->setIZone(.5);

        timeoutTurn.start();

        while (!isAtSetpoint) {
            targetAngle = pose->angleTo(targetPose) - (M_PI / 2);
            targetAngle += isForward ? 0 : M_PI;
            double error = targetAngle - fmod((pose->getTheta() + 2*M_PI), 2*M_PI);
            if (error > M_PI) {
                error = error - 2*M_PI;
            } 
            else if (error < -M_PI) {
                error = 2*M_PI + error;
            }

            int output = turnPID->calculate(-1 * error, 0);
            drivetrain->setMotorSpeeds({-output, output});

            if (turnPID->isInSmallErrorRange()) {
                smallTimerTurn.start();
            }
            else if (turnPID->isInLargeErrorRange()) {
                smallTimerTurn.stop();
                largeTimerTurn.start();
            }
            else {
                smallTimerTurn.stop();
                largeTimerTurn.stop();
            }

            pros::delay(20);
        }
        smallTimerTurn.stop();
        largeTimerTurn.stop();
        timeoutTurn.stop();
        drivetrain->setMotorSpeeds({0, 0});
    }
    
    // Drive to Pose
    isAtSetpoint = false;
    double lastError = 0;

    Timer timeout(5000, +[]() { Chassis::isAtSetpoint = true; }); 
    Timer smallTimer(500, +[]() { Chassis::isAtSetpoint = true; });
    Timer largeTimer(1500, +[]() { Chassis::isAtSetpoint = true; });

    double currentAngle = pose->getTheta();

    lateralPID->reset();
    alignPID->reset();

    lateralPID->setOutputLimits(-50, 50);
    lateralPID->setSmallErrorRange(1);
    lateralPID->setLargeErrorRange(2);

    alignPID->setOutputLimits(-10, 10);

    timeout.start();

    while (!isAtSetpoint) {
        double error = -1 * pose->distanceTo(targetPose);

        int output = lateralPID->calculate(error, 0) + alignPID->calculate(pose->getTheta(), currentAngle);
        output *= isForward ? 1 : -1;   
        drivetrain->setMotorSpeeds({output, output});

        if (lateralPID->isInSmallErrorRange()) {
            smallTimer.start();
        }
        else if (lateralPID->isInLargeErrorRange()) {
            smallTimer.stop();
            largeTimer.start();
        }
        else {
            smallTimer.stop();
            largeTimer.stop();
        }

        // if we passed the point, just exit
        if (lastError - error < -0.25) {
            break; 
        }

        lastError = error;

        pros::delay(20);
    }
    smallTimer.stop();
    largeTimer.stop();
    timeout.stop();
    drivetrain->setMotorSpeeds({0, 0});
}

/**
 * @brief Drive the robot a specified distance. Positive distance drives forwards,
 * negative distance drives backwards.
 * 
 * @param distance the distance to drive in inches.
 */
void DifferentialChassis::moveDistance(double distance, int timeout) {
    isAtSetpoint = false;

    if (!lateralPID || !turnPID) {
        return;
    }
    lateralPID->reset();

    Timer timeoutTimer(timeout, +[]() { Chassis::isAtSetpoint = true; });
    Timer smallErrorTimer(500, +[]() { Chassis::isAtSetpoint = true; });
    Timer largeErrorTimer(2000, +[]() { Chassis::isAtSetpoint = true; });

    timeoutTimer.start();
    double initialPosition = odometry ? odometry->getReadings()[0] : (drivetrain->getMotors()[0]->get_position() + drivetrain->getMotors()[1]->get_position()) / 2.0;
    
    lateralPID->reset();
    alignPID->reset();

    lateralPID->setOutputLimits(-50, 50);
    lateralPID->setSmallErrorRange(1);
    lateralPID->setLargeErrorRange(2);

    alignPID->setOutputLimits(-10, 10);

    while (!isAtSetpoint) {
        double currentPosition = odometry ? odometry->getReadings()[0] : (drivetrain->getMotors()[0]->get_position() + drivetrain->getMotors()[1]->get_position()) / 2.0;

        if (lateralPID->isInSmallErrorRange()) {
            if (!smallErrorTimer.isRunning()) {
                smallErrorTimer.start();
            }
        } else {
            smallErrorTimer.stop();
        }

        if (lateralPID->isInLargeErrorRange()) {
            if (!largeErrorTimer.isRunning()) {
                largeErrorTimer.start();
            }
        } else {
            largeErrorTimer.stop();
        }

        int output = lateralPID->calculate(currentPosition-initialPosition, distance);

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
 * 0 Degrees is facing "forward" from the starting orientation.
 * 
 * @param targetAngle The target angle to turn to (in degrees), from 0 to 360. Turns to the right will be decreasing angle
 */
void DifferentialChassis::turnAngle(double targetAngle, int timeout) {
    if (!turnPID) {
        return;
    }

    isAtSetpoint = false;

    Timer timeoutTimer(timeout, +[]() { Chassis::isAtSetpoint = true; }); 
    Timer smallTimer(500, +[]() { Chassis::isAtSetpoint = true; });
    Timer largeTimer(1500, +[]() { Chassis::isAtSetpoint = true; });

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

        int output = turnPID->calculate(-1 * error, 0);
        drivetrain->setMotorSpeeds({-output, output});

        if (turnPID->isInSmallErrorRange()) {
            smallTimer.start();
        }
        else if (turnPID->isInLargeErrorRange()) {
            smallTimer.stop();
            largeTimer.start();
        }
        else {
            smallTimer.stop();
            largeTimer.stop();
        }

        pros::delay(20);
    }
    smallTimer.stop();
    largeTimer.stop();
    timeoutTimer.stop();
    drivetrain->setMotorSpeeds({0, 0});
}
