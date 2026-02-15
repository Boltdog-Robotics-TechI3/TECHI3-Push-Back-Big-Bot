#pragma once

#include "x_drivetrain.hpp"
#include "chassis.hpp"
#include "odom_sensors.hpp"
#include "pid.hpp"
#include "util/pose.hpp"
#include "pros/rtos.hpp"

class XChassis : public Chassis {
    public:
        /**
         * @brief Construct a new xChassis object with full odometry and autonomous capabilities.
         * @param drivetrain Pointer to the x drivetrain.
         * @param odometry Pointer to the odometry.
         * @param movePID Pointer to the move PID controller, used for translation movements.
         * @param turnPID Pointer to the turn PID controller, used for rotation.
         */
        XChassis(XDrivetrain *drivetrain, OdomSensors *odometry, PIDController *movePID, PIDController *turnPID)
        : Chassis(drivetrain, odometry, movePID, turnPID) {}

        /**
         * @brief Construct a new xChassis object with a drivetrain and odometry. 
         * This xChassis will have full odometry capabilities, but will not have autonomous features.
         * @param drivetrain Pointer to the x drivetrain.
         * @param odometry Pointer to the odometry.
         */
        XChassis(XDrivetrain *drivetrain, OdomSensors *odometry)
        : Chassis(drivetrain, odometry) {}

        /**
         * @brief Construct a new xChassis object with a drivetrain and PID controllers. 
         * This XChassis will not have odometry capabilities, but will have basic autonomous capabilities.
         * @param drivetrain Pointer to the x drivetrain.
         * @param movePID Pointer to the move PID controller, used for translation movements.
         * @param turnPID Pointer to the turn PID controller, used for rotation.
         */
        XChassis(XDrivetrain *drivetrain, PIDController *movePID, PIDController *turnPID) 
        : Chassis(drivetrain, movePID, turnPID) {}

        /**
         * @brief Construct a new xChassis object with only a drivetrain. 
         * This xChassis will not have odometry capabilities nor autonomous features.
         * @param drivetrain Pointer to the x drivetrain.
         */
        XChassis(XDrivetrain *drivetrain) 
        : Chassis(drivetrain) {}

        /**
        * @brief Drive the robot at a specific angle with translational and rotational speeds.
        * @param angle The angle to drive in radians.
        * @param transSpeed The translational speed.
        * @param rotSpeed The rotational speed.
        */
        void driveAngle(double angle, int transSpeed, int rotSpeed);

        /**
         * @brief Move the robot in field-centric mode using joystick inputs.
         * @param leftX The x-value of the left joystick.
         * @param leftY The y-value of the left joystick.
         * @param rightX The x-value of the right joystick.
         */
        void fieldCentricDrive(int leftX, int leftY, int rightX);

        /**
         * @brief Move the robot in robot-centric mode using joystick inputs.
         * @param leftX The x-value of the left joystick.
         * @param leftY The y-value of the left joystick.
         * @param rightX The x-value of the right joystick.
         */
        void robotCentricDrive(int leftX, int leftY, int rightX);

        /**
         * @brief Move the robot in field-centric mode using joystick inputs with direct heading control.
         * The left joystick controls translation, and the right joystick controls the angle the robot is facing.
         * @param leftX The x-value of the left joystick.
         * @param leftY The y-value of the left joystick.
        *  @param rightX The x-value of the right joystick.
         * @param rightY The y-value of the right joystick.
         */
        void fieldCentricHeadingDrive(int leftX, int leftY, int rightX, int rightY);
        
        /**
         * @brief Move the robot to a specific position using PID control.
         * 
         * @note This method will obey the angle of the target pose while driving to the x and y coordinates.
         * 
         * @param targetPose The target pose to move to.
         * @param timeout The amount of time in milliseconds that the robot will try to reach the pose before giving up (default 5 seconds).
         * @param maxSpeed The maximum speed the robot can travel, from 0 to 127
         */
        void moveToPose(const Pose& targetPose, int timeout = 5000, int maxSpeed = 50) override;

        /**
         * @brief Turn the robot to a specific angle using PID control.
         * 0 Degrees is facing "forward" from the starting orientation.
         * 
         * @param targetAngle The target angle to turn to (in degrees).
         * @param timeout The amount of time in milliseconds that the robot will try to reach the pose before giving up
         */
        void turnToAngle(double targetAngle, int timeout) override;
};