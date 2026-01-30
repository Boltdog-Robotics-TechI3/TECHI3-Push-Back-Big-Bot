#pragma once

#include "differentialdrivetrain.hpp"
#include "chassis.hpp"
#include "odometry.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"

class DifferentialChassis : public Chassis {
    public:
        /**
         * @brief Construct a new DifferentialChassis object with full odometry and autonomous capabilities.
         * @param drivetrain Pointer to the differential drivetrain.
         * @param odometry Pointer to the odometry.
         * @param lateralPID Pointer to the lateral PID controller, used for translation movements.
         * @param turnPID Pointer to the turn PID controller, used for rotation.
         * @param alignPID Pointer to the align PID Controller, used in conjuction with lateralPID to control the heading.
         */
        DifferentialChassis(DifferentialDrivetrain *drivetrain, Odometry *odometry, PIDController *lateralPID, PIDController *turnPID, PIDController *alignPID)
        : Chassis(drivetrain, odometry, lateralPID, turnPID, alignPID) {}

        /**
         * @brief Construct a new DifferentialChassis object with a drivetrain and odometry. 
         * This DifferentialChassis will have full odometry capabilities, but will not have autonomous features.
         * @param drivetrain Pointer to the differential drivetrain.
         * @param odometry Pointer to the odometry.
         */
        DifferentialChassis(DifferentialDrivetrain *drivetrain, Odometry *odometry)
        : Chassis(drivetrain, odometry) {}

        /**
         * @brief Construct a new DifferentialChassis object with a drivetrain and PID controllers. 
         * This DifferentialChassis will not have odometry capabilities, but will have basic autonomous capabilities.
         * @param drivetrain Pointer to the differential drivetrain.
         * @param lateralPID Pointer to the lateral PID controller, used for translation movements.
         * @param turnPID Pointer to the turn PID controller, used for rotation.
         * @param alignPID Pointer to the align PID Controller, used in conjuction with lateralPID to control the heading.
         */
        DifferentialChassis(DifferentialDrivetrain *drivetrain, PIDController *lateralPID, PIDController *turnPID, PIDController *alignPID) 
        : Chassis(drivetrain, lateralPID, turnPID, alignPID) {}

        /**
         * @brief Construct a new DifferentialChassis object with only a drivetrain. 
         * This DifferentialChassis will not have odometry capabilities nor autonomous features.
         * @param drivetrain Pointer to the differential drivetrain.
         */
        DifferentialChassis(DifferentialDrivetrain *drivetrain) 
        : Chassis(drivetrain) {}

         /**
         * @brief Move the robot in arcade mode. The left joystick controls the forward/backward movement, and the right joystick controls the rotation.
         * @param leftY The value of the left joystick (forward/backward movement).
         * @param rightX The value of the right joystick (rotation).
         */
        void arcade(int leftY, int rightX);

        /**
         * @brief Move the robot in tank mode. The left joystick controls the left side motors, and the right joystick controls the right side motors.
         * @param leftY The value of the left joystick (left side motors).
         * @param rightY The value of the right joystick (right side motors).
         */
        void tank(int leftY, int rightY);

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
         * @param isForward Whether the robot should move forward (true) or backward (false) to the target pose.
         */
        void moveToPoseStep(const Pose& targetPose, bool isForward = true) override;

        /**
         * @brief Move the robot to a specific position using PID control. This method blocks until the target position is reached.
         * 
         * @note This method ignores the angle of the target pose and only drives to the x and y coordinates.
         * 
         * @param targetPose The target pose to move to.
         * @param isForward Whether the robot should move forward (true) or backward (false) to the target pose.
         */
        void moveToPose(const Pose& targetPose, bool isForward = true) override;

        /**
         * @brief Drives the robot until its x coordinate matches the given target
         * 
         * @param x the x coordinate to drive to, in inches
         */
        void driveToX(double x);

        /**
         * @brief Drives the robot until its x coordinate matches the given target
         * 
         * @param x the x coordinate to drive to, in inches
         */
        void driveToY(double y);

        /**
         * @brief Turns to face the pose, then drives in a straight line to the pose.
         * 
         * @param targetPose the pose to drive to
         */
        void turnThenMoveToPose(const Pose& targetPose, const bool isForward = true);

        /**
         * @brief Drive the robot a specified distance. Positive distance drives forwards,
         * negative distance drives backwards.
         * 
         * @param distance the distance to drive in inches.
         */
        void moveDistance(double distance, int timeout);

        /**
         * @brief Turn the robot to a specific angle using PID control.
         * 0 Degrees is facing "forward" from the starting orientation.
         * 
         * @param targetAngle The target angle to turn to (in degrees).
         */
        void turnAngle(double targetAngle, int timeout) override;
};