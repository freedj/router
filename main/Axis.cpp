//Axis.cpp

#include "Axis.h"
Axis::Axis(int step, int direction) {

	AccelStepper stepper(AccelStepper::DRIVER, step, direction);
	buttonRate = 127;
	cutSpeed = 128;
	rapidSpeed = 800;
	target = 0;
	stepover = 32;
}

void Axis::rapid (int to) {
	stepper.setMaxSpeed(rapidSpeed);
	stepper.moveTo(to);
}

void Axis::cut (int to) {
	stepper.setMaxSpeed(cutSpeed);
	stepper.moveTo(to);
}

void Axis::stop() {
	stepper.stop();
}

bool Axis::isRunning () {
	return stepper.distanceToGo();
}

void Axis::goToOrigin() {
	stepper.setMaxSpeed(rapidSpeed);
	stepper.runToNewPosition(0);
}

void Axis::goToTarget() {
	stepper.setMaxSpeed(rapidSpeed);
	stepper.runToNewPosition(target);
}
