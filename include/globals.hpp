#pragma once
#include "main.h"

inline bool skills = false;
inline bool match = true;
inline bool redAlliance = true;
inline bool blueAlliance = false;
inline int autoSelection = 0;

inline const double wheelDiameter = 2.75;
inline const double trackWidth = 12.5;
inline const double gearRatio = 1;

// Drivetrain Motors
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);
inline pros::MotorGroup frontLeftDrive({12, -15});
inline pros::MotorGroup frontRightDrive({11, -14}); 
inline pros::MotorGroup backLeftDrive({18, -20});
inline pros::MotorGroup backRightDrive({17, -16});

inline XDrivetrain drivetrain(&frontLeftDrive, &frontRightDrive, &backLeftDrive, &backRightDrive, wheelDiameter, trackWidth, gearRatio);

// Intake 
inline pros::Motor intake(1, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
inline pros::Motor indexer(-2, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);
inline pros::Motor ejector(-3, pros::v5::MotorGears::blue, pros::v5::MotorUnits::degrees);

inline pros::adi::Pneumatics wingPiston('A', false);
inline pros::adi::Pneumatics matchLoadPiston('B', false);

inline pros::Optical colorSensor(4);

// Odometry
inline TrackingWheel verticalTrackingWheel(10, 2.08, 2.5, WheelPosition::VERTICAL);
inline TrackingWheel horizontalTrackingWheel(9, 2.08, -6.625, WheelPosition::HORIZONTAL);

inline pros::IMU imu(8);

inline OdomSensors odometry(&verticalTrackingWheel, &horizontalTrackingWheel, &imu);

inline PIDController Lateral(6, 0.015, 5);
inline PIDController Turn(70, .2, 0);

inline XChassis chassis(&drivetrain, &odometry ,&Lateral, &Turn);
