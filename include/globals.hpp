#pragma once
#include "main.h"

inline bool skills = false;
inline bool match = true;
inline bool redAlliance = false;
inline bool blueAlliance = true;
inline int autoSelection = 0;

inline const double wheelDiameter = 2.75;
inline const double trackWidth = 12.5;
inline const double gearRatio = 1;

// Drivetrain Pieces
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);
inline pros::MotorGroup frontLeftDrive({1, -16});
inline pros::MotorGroup frontRightDrive({-19,20});
inline pros::MotorGroup backLeftDrive({-13, 14});
inline pros::MotorGroup backRightDrive({3, -12});

inline XDrivetrain drivetrain(&frontLeftDrive, &frontRightDrive, &backLeftDrive, &backRightDrive, wheelDiameter, trackWidth, gearRatio);

// Intake Pieces
inline pros::MotorGroup intakes({2,-21});
inline pros::Motor frontRoller(-6);
inline pros::Motor scoringRoller(-7);
inline pros::Motor midRoller(-8);
inline pros::Motor topLevel(-10);
inline pros::Motor bottomRoller(-15);

inline pros::adi::Pneumatics intakePiston('B', false);
inline pros::adi::Pneumatics wingPiston('A', false);

// Odometry Pieces
inline TrackingWheel verticalTrackingWheel(17, 2.08, 5.5, WheelPosition::VERTICAL);
inline TrackingWheel horizontalTrackingWheel(18, 2.08, 0, WheelPosition::HORIZONTAL);

inline pros::IMU imu(5);

inline OdomSensors odometry(&verticalTrackingWheel, &horizontalTrackingWheel, &imu);

inline PIDController Lateral(6, .001, 0);
inline PIDController Turn(70, .15, .5);

inline XChassis chassis(&drivetrain, &odometry ,&Lateral, &Turn);
