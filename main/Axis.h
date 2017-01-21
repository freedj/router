// Axis.h

#ifndef AXISFILE
#define AXISFILE
#include <AccelStepper.h>
class Axis
{
	public:
		Axis (int step, int direction);

		void rapid(int to);
		void cut(int to);
		bool isRunning();
		void stop();
		void goToOrigin();
		void goToTarget();

		AccelStepper stepper;
		int cutSpeed;
		int rapidSpeed;
		long target;
		int buttonRate;
		int stepover;


	private:
};


#endif
