#include "BaseMotorControl.h"

/*
	postcondition: 0 < RPS < 4

	note: don't go above 90% of max	motor is spec'd to run at max 3.6 
	rotations per second
*/
double CalcRPS( double* twist_msg, char wheel, int linear_multiplier, int angular_multiplier ) {
	if (wheel != 'L' && wheel != 'R') /* handle bad arguments */
		return 0;

	return linear_multiplier * CalcLinearRPS( twist_msg[0] ) + 
		angular_multiplier * CalcAngularRPS( twist_msg[2], wheel );
}

double CalcLinearRPS( double linear_velocity ) {
	return linear_velocity / CIRCUMFERENCE;
}

double CalcAngularRPS( double angular_velocity, char wheel ) {
	double denominator = DISTANCE_TO_CENTER *
			CalcWheelDirection(angular_velocity, wheel) *
			CIRCUMFERENCE;

	return fabs(angular_velocity) / denominator;
}

/*
	precondition: angular_velocity != 0
	precondition: wheel is either 'L' or 'R'

	angular_velocity > 0 => ccw
	angular_velocity < 0 => cw
*/
int CalcWheelDirection( double angular_velocity, char wheel ) {
	if (wheel == 'L') {
		if (angular_velocity > 0)
			return -1;
		else
			return 1;
	}
	else { //wheel == 'R'
		if (angular_velocity > 0)
			return 1;
		else
			return -1;
	}
}
