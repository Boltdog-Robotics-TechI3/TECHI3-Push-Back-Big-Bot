#include <cmath>
#include "lib/holonomicchassis.hpp"
#include "pros/rtos.hpp"

/**
 * @brief Drive the robot at a specific angle with translational and rotational speeds.
 * @param angle The angle to drive in radians.
 * @param transSpeed The translational speed.
 * @param rotSpeed The rotational speed.
 */
void HolonomicChassis::driveAngle(double angle, int transSpeed, int rotSpeed) {
	double adjustedAngle = angle;
	int x = cos(adjustedAngle) * transSpeed;
	int y = sin(adjustedAngle) * transSpeed;

	drivetrain->setMotorSpeeds({y+x+rotSpeed,
                                -y+x+rotSpeed,
                                y-x+rotSpeed,
                                -y-x+rotSpeed});
}

/**
 * @brief Move the robot in field-centric mode using joystick inputs.
 * @param leftX The x-value of the left joystick.
 * @param leftY The y-value of the left joystick.
 * @param rightX The x-value of the right joystick.
 */
void HolonomicChassis::fieldCentricDrive(int leftX, int leftY, int rightX) {
    double y = (double)leftY;
    double x = (double)leftX;
    double r = scaleInput(rightX);

    double targetAngle = atan2(y, x);
    double speed = scaleInput(sqrt(x*x + y*y));

	driveAngle(targetAngle + odometry->getRotationRadians(), speed, r);
}

/**
 * @brief Move the robot in robot-centric mode using joystick inputs.
 * @param leftX The x-value of the left joystick.
 * @param leftY The y-value of the left joystick.
 * @param rightX The x-value of the right joystick.
 */
void HolonomicChassis::robotCentricDrive(int leftX, int leftY, int rightX) {
    double y = (double)leftY;
    double x = (double)leftX;
    double r = scaleInput(rightX);

    double targetAngle = atan2(y, x);
    double speed = scaleInput(sqrt(x*x + y*y));

    driveAngle(targetAngle, speed, r);
}

/**
 * @brief Move the robot in field-centric mode using joystick inputs with direct heading control.
 * The left joystick controls translation, and the right joystick controls the angle the robot is facing.
 * @param leftX The x-value of the left joystick.
 * @param leftY The y-value of the left joystick.
 *  @param rightX The x-value of the right joystick.
 * @param rightY The y-value of the right joystick.
 */
void HolonomicChassis::fieldCentricHeadingDrive(int leftX, int leftY, int rightX, int rightY) {
    double y = (double)leftY;
    double x = (double)leftX;

    double targetDriveAngle = atan2(y, x);
    double speed = scaleInput(sqrt(x*x + y*y));

    // Find the target heading angle based on the right joystick and convert it to the robot-centric frame
    double targetHeadingAngle = -(atan2((double)rightX, (double)rightY) - M_PI_2);
	targetHeadingAngle = targetHeadingAngle > M_PI ? targetHeadingAngle - M_TWOPI : targetHeadingAngle;

    double r = turnPID->calculate(pose->getTheta(), targetHeadingAngle);

	driveAngle(targetDriveAngle + odometry->getRotationRadians(), speed, r);
}


/**
 * @brief Move the robot to a specific position using PID control.
 * @param targetPose The target pose to move to.
 */
void HolonomicChassis::moveToPose(const Pose& targetPose, bool isForward) {
    // TODO: Implement MoveTo for HolonomicChassis
}

/**       
 * @brief Turn the robot to a specific angle using PID control.
 * 0 Degrees is facing "forward" from the starting orientation.
 * 
 * @param targetAngle The target angle to turn to (in degrees).
 */
void HolonomicChassis::turnAngle(double targetAngle, int timeout) {
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
        drivetrain->setMotorSpeeds({output, output, output, output});

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