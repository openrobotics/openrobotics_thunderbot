#ifndef BASE_MOTOR_CONTROLLER_
#define BASE_MOTOR_CONTROLLER_

#define WHEEL_RADIUS 10.0 //TODO CHANGE, only valid for testing purposes
#define PI 3.141592
#define CIRCUMFERENCE (2.0*PI*WHEEL_RADIUS)
#define DISTANCE_TO_CENTER 33.0

#include <math.h>

/* converts angular and linear velocities into RPS for a wheel. 

	It works by summing up the results of converting each velocity into
	an RPS.

	Wheel_i_rps = linear_velocity / (+/-)(2*pi*wheel_i_radius) + angular_velocity / (d * (2*pi*wheel_i_radius))

	the (+/-) is determined by which wheel is doing the calculation
	d : distance from wheel to centre of turning of robot
	i = {left, right}

	twist_msg is of the form < linear x vel, linear y vel, angular z vel >
	wheel is either 'L' or 'R'

	example: takes a received twist message and calculates the left wheel's RPS

	float msg[3] = {1.0, 2.5, 3}
	left_wheel_rps = CalcRPS(msg, 'L')

*/
double CalcRPS( double* twist_msg , char wheel, int linear_x_constant, int angular_z_constant );

/* converts linear velocity into RPS */
double CalcLinearRPS( double linear_velocity ); 

/* converts angular velocity into RPS */
double CalcAngularRPS( double angular_velocity, char wheel ); 

/* determines whether RPS is positive or negative.

	if negative the function contributes a negative RPS (reverse direction)
	and if positive the function contributes a positive RPS (forward direction)	

*/
int CalcWheelDirection( double velocity, char wheel );

#endif // BASE_MOTOR_CONTROLLER_
