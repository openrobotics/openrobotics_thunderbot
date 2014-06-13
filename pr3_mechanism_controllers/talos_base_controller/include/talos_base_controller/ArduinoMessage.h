#ifndef ARDUINO_MESSAGE_H
#define ARDUINO_MESSAGE_H

struct ArduinoMessage 
{
  unsigned long  time_stamp;
	bool e_stop;
	float l_motor_rpm;
	float r_motor_rpm;
	float l_pid;
	float r_pid;
	float l_feedback;
	float r_feedback;
};
#endif
