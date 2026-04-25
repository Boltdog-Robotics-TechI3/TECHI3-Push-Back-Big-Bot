#include "main.h"

void goalRushAuto() {
	chassis.setPose(-14.5, 49, 0);
	chassis.startTracking();

	// Drive To Loader
	matchLoadPiston.extend();
	chassis.moveToPose({.targetPose={-46, 49, 0}, .timeout=2000, .maxMoveSpeed=100});
	intakeBlocks(127);

	// Wiggle loader
	chassis.moveToPose({.targetPose={-46, 63, 0}, .timeout=100, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-46, 58, 0}, .timeout=500, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-46, 63, 0}, .timeout=500, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-46, 58, 0}, .timeout=500, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-46, 63, 0}, .timeout=500, .maxMoveSpeed=127});

	// Go Score Long Goal
	chassis.moveToPose({.targetPose={-47, 31, 0}, .timeout=1000, .maxMoveSpeed=80});
	scoreHigh(127);
	pros::delay(2000);
	stopIntake();

	// Go wing it
	chassis.moveToPose({.targetPose={-33, 35, 0}, .timeout=1000, .maxMoveSpeed=100});
	wingPiston.extend();
	chassis.moveToPose({.targetPose={-35, 8, 0}, .timeout=2000, .maxMoveSpeed=127});
	chassis.turnToAngle({.targetAngle=45, .timeout=500, .maxTurnSpeed=100});
}

void lowThenRushAuto() {
	// Auto Setup
	chassis.setPose(-14.5, 49, M_PI);
	chassis.startTracking();

	// Grab two mid blocks
	lowIntaking(90);
	chassis.moveToPose({.targetPose={-41, 10, 135}, .timeout=3000, .maxMoveSpeed=80, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5});
	stopIntake();

	// Move to low goal and score 
	chassis.moveToPose({.targetPose={-14.5, 17.5, 225}, .timeout=2000, .maxMoveSpeed=80, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5, .maxTurnSpeed=100,});
	lowScore(70);
	pros::delay(2000);
	stopIntake();

	// Drive To Loader
	chassis.moveToPose({.targetPose={-48, 49, 0}, .timeout=2000, .maxMoveSpeed=100, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5, .maxTurnSpeed=80});
	matchLoadPiston.extend();
	pros::delay(200);
	intakeBlocks(127);

	// Wiggle loader
	chassis.moveToPose({.targetPose={-47.5, 62, 0}, .timeout=750, .maxMoveSpeed=60, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5});
	// chassis.moveToPose({.targetPose={-48, 57, 5}, .timeout=250, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-47.5, 80, 0}, .timeout=750, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-47.5, 57, 0}, .timeout=250, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-47.5, 80, 0}, .timeout=750, .maxMoveSpeed=127});

	// Go Score Long Goal
	chassis.moveToPose({.targetPose={-47.5, 31, 0}, .timeout=1000, .maxMoveSpeed=80, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5});
	stopIntake();
	matchLoadPiston.retract();
	scoreHigh(127);
	pros::delay(2000);
	stopIntake();

	// Go wing it
	chassis.moveToPose({.targetPose={-33, 38, 10}, .timeout=1000, .maxMoveSpeed=100, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5});
	wingPiston.extend();
	chassis.moveToPose({.targetPose={-35, 8, 10}, .timeout=2000, .maxMoveSpeed=127});
	chassis.turnToAngle({.targetAngle=30, .timeout=500, .maxTurnSpeed=100});
}

void fullLoaderAuto() {
	// Auto Setup
	chassis.setPose(-14.5, 49, M_PI);
	chassis.startTracking();

	// Grab two mid blocks
	lowIntaking(90);
	chassis.moveToPose({.targetPose={-41, 10, 135}, .timeout=3000, .maxMoveSpeed=80, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5});
	stopIntake();

	// Move to low goal and score 
	chassis.moveToPose({.targetPose={-14.5, 17.5, 225}, .timeout=2000, .maxMoveSpeed=80, .smallMoveErrorRange=0.5, .largeMoveErrorRange=1, .maxTurnSpeed=100,});
	lowScore(80);
	pros::delay(2000);
	stopIntake();

	// Drive To Loader
	chassis.moveToPose({.targetPose={-48, 44, 0}, .timeout=2000, .maxMoveSpeed=100, .smallMoveErrorRange=0.5, .largeMoveErrorRange=1, .maxTurnSpeed=80});
	matchLoadPiston.extend();
	intakeBlocks(127);

	// Wiggle loader
	chassis.moveToPose({.targetPose={-47.5, 62, 0}, .timeout=750, .maxMoveSpeed=60, .smallMoveErrorRange=0.5, .largeMoveErrorRange=1});
	chassis.moveToPose({.targetPose={-47.5, 80, 0}, .timeout=750, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-47.5, 57, 0}, .timeout=250, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-47.5, 80, 0}, .timeout=750, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-47.5, 57, 0}, .timeout=250, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-47.5, 80, 0}, .timeout=750, .maxMoveSpeed=127});

	// Go Score Long Goal
	chassis.moveToPose({.targetPose={-47.5, 31, 0}, .timeout=1000, .maxMoveSpeed=80, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5});
	stopIntake();
	matchLoadPiston.retract();
	scoreHigh(127);
	pros::delay(2000);
	stopIntake();

		// Wiggle loader
	intakeBlocks(127);
	matchLoadPiston.extend();
	chassis.moveToPose({.targetPose={-47.5, 62, 0}, .timeout=750, .maxMoveSpeed=60, .smallMoveErrorRange=0.5, .largeMoveErrorRange=1});
	// chassis.moveToPose({.targetPose={-48, 57, 5}, .timeout=250, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-47.5, 80, 0}, .timeout=750, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-47.5, 57, 0}, .timeout=250, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-47.5, 80, 0}, .timeout=750, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-47.5, 57, 0}, .timeout=250, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-47.5, 80, 0}, .timeout=750, .maxMoveSpeed=127});

	// Go Score Long Goal
	chassis.moveToPose({.targetPose={-47.5, 31, 0}, .timeout=1000, .maxMoveSpeed=80, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5});
	stopIntake();
	matchLoadPiston.retract();
	scoreHigh(127);
	pros::delay(2000);
	stopIntake();

	// Go wing it
	chassis.moveToPose({.targetPose={-35, 38, 10}, .timeout=1000, .maxMoveSpeed=100, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5});
	wingPiston.extend();
	chassis.moveToPose({.targetPose={-39, 8, 10}, .timeout=2000, .maxMoveSpeed=127});
	chassis.turnToAngle({.targetAngle=30, .timeout=500, .maxTurnSpeed=100});
}


void AWPAuto() {
	// Auto Setup
	chassis.setPose(-9, 45.5, M_PI);
	chassis.startTracking();

	// Grab left two mid blocks
	intakeBlocks(127);
	chassis.moveToPose({.targetPose={36, 10, 225}, .timeout=1500, .maxMoveSpeed=127, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5});
	chassis.moveToPose({.targetPose={42, 8, 250}, .timeout=1000, .maxMoveSpeed=60, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5});

	// Move to mid goal and score 
	chassis.moveToPose({.targetPose={11.5, 11.5, 315}, .timeout=2000, .maxMoveSpeed=80, .smallMoveErrorRange=0.5, .largeMoveErrorRange=1, .maxTurnSpeed=100});
	scoreMid(100);
	pros::delay(1000);
	stopIntake();
	chassis.moveToPose({.targetPose={20, 21, 315}, .timeout=300, .maxMoveSpeed=127, .smallMoveErrorRange=0.5, .largeMoveErrorRange=1, .maxTurnSpeed=100});


	// Drive To Loader
	pros::Task matchLoader1 = pros::Task([&]() {
		pros::delay(1000);
		setMatchLoader(true);
	});
	chassis.moveToPose({.targetPose={-48, 38, 0}, .timeout=2000, .maxMoveSpeed=100, .smallMoveErrorRange=0.5, .largeMoveErrorRange=1, .maxTurnSpeed=80});
	pros::delay(200);
	intakeBlocks(127);

	// Wiggle loader
	chassis.moveToPose({.targetPose={-48, 62, 0}, .timeout=750, .maxMoveSpeed=50, .smallMoveErrorRange=0.5, .largeMoveErrorRange=1});
	chassis.moveToPose({.targetPose={-48, 80, 0}, .timeout=1000, .maxMoveSpeed=127});

	// Move to low goal and score 
	pros::Task matchLoader2 = pros::Task([&]() {
		pros::delay(500);
		setMatchLoader(false);
	});
	chassis.moveToPose({.targetPose={-11, 9, 225}, .timeout=2000, .maxMoveSpeed=80, .smallMoveErrorRange=0.5, .largeMoveErrorRange=1, .maxTurnSpeed=60,});
	lowScore(100);
	pros::delay(2000);
	stopIntake();

	// Drive To Loader
	chassis.moveToPose({.targetPose={-48, 38, 0}, .timeout=2000, .maxMoveSpeed=100, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5, .maxTurnSpeed=80});
	matchLoadPiston.extend();
	pros::delay(200);
	intakeBlocks(127);

	// Wiggle loader
	chassis.moveToPose({.targetPose={-48, 62, 0}, .timeout=750, .maxMoveSpeed=60, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5});
	chassis.moveToPose({.targetPose={-48, 80, 0}, .timeout=750, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-48, 57, 0}, .timeout=250, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-48, 80, 0}, .timeout=750, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-48, 57, 0}, .timeout=250, .maxMoveSpeed=127});
	chassis.moveToPose({.targetPose={-48, 80, 0}, .timeout=750, .maxMoveSpeed=127});

	// Go Score Long Goal
	chassis.moveToPose({.targetPose={-48, 22, 0}, .timeout=1500, .maxMoveSpeed=80, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5});
	stopIntake();
	matchLoadPiston.retract();
	scoreHigh(127);
	pros::delay(2000);
	stopIntake();

	// Go wing it
	chassis.moveToPose({.targetPose={-35, 38, 10}, .timeout=1000, .maxMoveSpeed=100, .smallMoveErrorRange=0.75, .largeMoveErrorRange=1.5});
	wingPiston.extend();
	chassis.moveToPose({.targetPose={-39, 3, 10}, .timeout=2000, .maxMoveSpeed=127});
	chassis.turnToAngle({.targetAngle=30, .timeout=500, .maxTurnSpeed=100});
}