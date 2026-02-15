#pragma once

#include "tank_drivetrain.hpp"
#include "chassis.hpp"
#include "odom_sensors.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"

class TankChassis : public Chassis {
    private:
        float minAlignDistance = 5; // This distance is threshold for which angle alignment is disabled in autonomous driving functions

        PIDController *alignPID;
    public:
        /**
         * @brief Construct a new TankChassis object with full odometry and autonomous capabilities.
         * @param drivetrain Pointer to the Tank drivetrain.
         * @param odometry Pointer to the Odom Sensors.
         * @param movePID Pointer to the move PID controller, used for translation movements.
         * @param turnPID Pointer to the turn PID controller, used for rotation.
         * @param alignPID Pointer to the align PID Controller, used in conjuction with movePID to control the heading.
         */
        TankChassis(TankDrivetrain *drivetrain, OdomSensors *odometry, PIDController *movePID, PIDController *turnPID, PIDController *alignPID)
        : alignPID(alignPID), Chassis(drivetrain, odometry, movePID, turnPID) {}

        /**
         * @brief Construct a new TankChassis object with a drivetrain and odometry. 
         * This TankChassis will have full odometry capabilities, but will not have autonomous features.
         * @param drivetrain Pointer to the Tank drivetrain.
         * @param odometry Pointer to the odometry.
         */
        TankChassis(TankDrivetrain *drivetrain, OdomSensors *odometry)
        : alignPID(nullptr), Chassis(drivetrain, odometry) {}

        /**
         * @brief Construct a new TankChassis object with a drivetrain and PID controllers. 
         * This TankChassis will not have odometry capabilities, but will have basic autonomous capabilities.
         * @param drivetrain Pointer to the Tank drivetrain.
         * @param movePID Pointer to the move PID controller, used for translation movements.
         * @param turnPID Pointer to the turn PID controller, used for rotation.
         * @param alignPID Pointer to the align PID Controller, used in conjuction with movePID to control the heading.
         */
        TankChassis(TankDrivetrain *drivetrain, PIDController *movePID, PIDController *turnPID, PIDController *alignPID) 
        : alignPID(alignPID),  Chassis(drivetrain, movePID, turnPID) {}

        /**
         * @brief Construct a new TankChassis object with only a drivetrain. 
         * This TankChassis will not have odometry capabilities nor autonomous features.
         * @param drivetrain Pointer to the Tank drivetrain.
         */
        TankChassis(TankDrivetrain *drivetrain) 
        : alignPID(nullptr), Chassis(drivetrain) {}

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
         * @brief Move the robot to a specific position using PID control. This method blocks until the target position is reached.
         * 
         * @note This method ignores the angle of the target pose and only drives to the x and y coordinates.
         * 
         * @param targetPose The target pose to move to.
         * @param timeout The amount of time in milliseconds that the robot will try to reach the pose before giving up (default 5 seconds).
         * @param maxSpeed The maximum speed the robot can travel, from 0 to 127
         */
        void moveToPose(const Pose& targetPose, int timeout = 5000, int maxSpeed = 50) override;

        /**
         * @brief Drive the robot a specified distance. Positive distance drives forwards,
         * negative distance drives backwards.
         * 
         * @param distance the distance to drive in inches.
         */
        void moveDistance(double distance, int timeout);

        /**
         * @brief Turn the robot to a specific angle using PID control.
         * 0 Degrees is facing "forward" from the starting orientation. Counterclockwise is positive
         * 
         * @param targetAngle The target angle to turn to (in degrees).
         */
        void turnToAngle(double targetAngle, int timeout) override;
};