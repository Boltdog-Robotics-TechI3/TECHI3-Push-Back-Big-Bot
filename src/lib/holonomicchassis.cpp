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
 * @brief Move the robot towards a specific position using a single step of PID control.
 * @param targetPose The target pose to move to.
 * @param isForward Whether the robot should move forward (true) or backward (false) to the target pose.
 */
#warning Untested!!!
void HolonomicChassis::moveToPoseStep(const Pose& targetPose, bool isForward) {
    if (!lateralPID || !turnPID || !odometry) {
        return;
    }

    // Calculate the direction to the target pose
    double dx = targetPose.getX() - pose->getX();
    double dy = targetPose.getY() - pose->getY();
    double distance = sqrt(dx*dx + dy*dy);
    
    // If we're already at the target, stop
    if (distance < 0.1) {
        drivetrain->setMotorSpeeds({0, 0, 0, 0});
        return;
    }

    // Calculate the target angle and drive speed
    double targetDriveAngle = atan2(dy, dx);
    int speed = lateralPID->calculate(distance, 0);
    
    // Apply forward/backward preference
    if (!isForward) {
        speed = -speed;
    }

    // Calculate the heading correction
    double targetHeadingAngle = targetPose.getTheta();
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

void HolonomicChassis::moveDistanceJANKY(double distance, int timeout, double slewRate) {
    if (!lateralPID) {
        return;
    }

    // bool isBackwards = distance < 0;

    isAtSetpoint = false;

    Timer timeoutTimer(timeout, +[]() { Chassis::isAtSetpoint = true; }); 
    Timer smallTimer(500, +[]() { Chassis::isAtSetpoint = true; });
    Timer largeTimer(1500, +[]() { Chassis::isAtSetpoint = true; });

    double currentAngle = pose->getTheta();

    lateralPID->reset();

    lateralPID->setOutputLimits(-70, 70);
    lateralPID->setSmallErrorRange(30);
    lateralPID->setLargeErrorRange(60);

    lateralPID->setSlewRate(slewRate);

    timeoutTimer.start();

    double initialPosition = drivetrain->getMotors()[0]->get_position();

    while (!isAtSetpoint) {
        double currentPosition = drivetrain->getMotors()[0]->get_position();

        int output = lateralPID->calculate(currentPosition-initialPosition, distance);
        // double correctionAngle = alignPID->calculate(pose->getTheta(), currentAngle);
        driveAngle(Pose::degToRad(90), output, 0);

        // std::cout << "Output: " << output << std::endl;
        // std::cout << "Error: " <<  distance - (currentPosition-initialPosition) << std::endl;

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
    timeoutTimer.stop();
    drivetrain->setMotorSpeeds({0, 0});
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
        drivetrain->setMotorSpeeds({-output, -output, -output, -output});

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
    drivetrain->setMotorSpeeds({0, 0, 0, 0});
}