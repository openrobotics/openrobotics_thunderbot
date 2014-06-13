#include <stdio.h>
#include "gtest/gtest.h"
#include "basemtrcontrol.h"


TEST(CalcRPSTest, Forward) {
	double twist_msg[3] = {3.7, 0, 0};
	double actual_left_RPS = CalcRPS(twist_msg, 'L');
	double actual_right_RPS = CalcRPS(twist_msg, 'R');

	double expected_right_RPS = twist_msg[0]/CIRCUMFERENCE;
	double expected_left_RPS = twist_msg[0]/CIRCUMFERENCE;

	ASSERT_NEAR(actual_left_RPS, expected_left_RPS, 0.0001);
	ASSERT_NEAR(actual_right_RPS, expected_right_RPS, 0.0001);
}

TEST(CalcRPSTest, Backward) {
	double twist_msg[3] = {-10.4, 0, 0};
	double actual_left_RPS = CalcRPS( twist_msg, 'L' );
	double actual_right_RPS = CalcRPS( twist_msg, 'R' );
	double expected_right_RPS = twist_msg[0]/CIRCUMFERENCE;
	double expected_left_RPS = twist_msg[0]/CIRCUMFERENCE;
	ASSERT_NEAR( actual_left_RPS, expected_left_RPS, 0.0001 );
	ASSERT_NEAR( actual_right_RPS, expected_right_RPS, 0.0001 );
}

TEST(CalcRPSTest, Stop) {
	double twist_msg[3] = {0, 0, 0};
	double actual_left_RPS = CalcRPS(twist_msg, 'L');
	double actual_right_RPS = CalcRPS(twist_msg, 'R');

	double expected_right_RPS = twist_msg[0]/CIRCUMFERENCE;
	double expected_left_RPS = twist_msg[0]/CIRCUMFERENCE;
	
	ASSERT_NEAR(actual_left_RPS, expected_left_RPS, 0.0001);
	ASSERT_NEAR(actual_right_RPS, expected_right_RPS, 0.0001);
}

TEST(CalcRPSTest, TurnLeft) {
	double twist_msg[3] = {0, 0, 5};
	double actual_left_RPS = CalcRPS(twist_msg, 'L');
	double actual_right_RPS = CalcRPS(twist_msg, 'R');

	double expected_left_RPS = twist_msg[2]/(CalcWheelDirection(twist_msg[2], 'L')*DISTANCE_TO_CENTER*2*PI*WHEEL_RADIUS);
	double expected_right_RPS = twist_msg[2]/(CalcWheelDirection(twist_msg[2], 'R')*DISTANCE_TO_CENTER*2*PI*WHEEL_RADIUS);

	ASSERT_NEAR(actual_left_RPS, expected_left_RPS, 0.01);
	ASSERT_NEAR(actual_right_RPS, expected_right_RPS, 0.01);
}

TEST(CalcRPSTest, ForwardRight) {
	double twist_msg[3] = {17, 0, -4.273};
	double actual_left_RPS = CalcRPS( twist_msg, 'L' );
	double actual_right_RPS = CalcRPS( twist_msg, 'R' );

	double left_linear_RPS = twist_msg[0]/CIRCUMFERENCE;
	double right_linear_RPS = twist_msg[0]/CIRCUMFERENCE;

	double left_angular_RPS = twist_msg[2]/(CalcWheelDirection(twist_msg[2], 'L')*DISTANCE_TO_CENTER*2*PI*WHEEL_RADIUS);
	double right_angular_RPS = twist_msg[2]/(CalcWheelDirection(twist_msg[2], 'R')*DISTANCE_TO_CENTER*2*PI*WHEEL_RADIUS);

	double expected_left_RPS = left_linear_RPS + left_angular_RPS;
	double expected_right_RPS = right_linear_RPS + right_angular_RPS;

	ASSERT_NEAR(actual_left_RPS, expected_left_RPS, 0.01);
	ASSERT_NEAR(actual_right_RPS, expected_right_RPS, 0.01);
}

TEST(CalcRPSTest, ReverseLeft) {
	double twist_msg[3] = {17, 0, -4.273};

	/* test linear_RPS calculations */

	double actual_left_linear_RPS = CalcLinearRPS( twist_msg[0] );
	double actual_right_linear_RPS = CalcLinearRPS( twist_msg[0] );

//	double expected_left_linear_RPS = twist_msg[0] / CIRCUMFERENCE;
	double expected_left_linear_RPS = (17/CIRCUMFERENCE);
	double expected_right_linear_RPS = (twist_msg[0]/CIRCUMFERENCE);

	ASSERT_NEAR(actual_left_linear_RPS, expected_left_linear_RPS,	0.01);
	ASSERT_NEAR(actual_right_linear_RPS, expected_right_linear_RPS, 0.01);

	/* test angular_rps calculations */

	double actual_left_angular_RPS = CalcAngularRPS( twist_msg[2], 'L' );
	double actual_right_angular_RPS = CalcAngularRPS( twist_msg[2], 'R' );

	double expected_left_angular_RPS = twist_msg[2]/(CalcWheelDirection(twist_msg[2], 'L')*DISTANCE_TO_CENTER*2*PI*WHEEL_RADIUS);

	double expected_right_angular_RPS = twist_msg[2] /
				(	CalcWheelDirection( twist_msg[2], 'R' )*
					DISTANCE_TO_CENTER*2*PI*WHEEL_RADIUS );

	ASSERT_NEAR(actual_left_angular_RPS, expected_left_angular_RPS,
							0.01);
	ASSERT_NEAR(actual_right_angular_RPS, expected_right_angular_RPS,
							0.01);

	/* test actual total rps */

	double expected_left_RPS = actual_left_linear_RPS + actual_left_angular_RPS;
	double expected_right_RPS = actual_right_linear_RPS + actual_right_angular_RPS;
	
	double actual_left_RPS = CalcRPS( twist_msg, 'L' );
	double actual_right_RPS = CalcRPS( twist_msg, 'R' );

	ASSERT_NEAR(actual_left_RPS, expected_left_RPS, 0.01);
	ASSERT_NEAR(actual_right_RPS, expected_right_RPS, 0.01);
}

TEST(CalcRPSTest, LargeForwardLeft) {

	double twist_msg[3] = {1700, 0, -40.273};

	double actual_left_RPS = CalcRPS( twist_msg, 'L' );
	double actual_right_RPS = CalcRPS( twist_msg, 'R' );

	double left_linear_RPS = twist_msg[0]/CIRCUMFERENCE;
	double right_linear_RPS = twist_msg[0]/CIRCUMFERENCE;

	double left_angular_RPS = twist_msg[2]/(CalcWheelDirection(twist_msg[2], 'L')*DISTANCE_TO_CENTER*2*PI*WHEEL_RADIUS);

	double right_angular_RPS = twist_msg[2]/(CalcWheelDirection(twist_msg[2], 'R')*DISTANCE_TO_CENTER*2*PI*WHEEL_RADIUS);

	double expected_left_RPS = left_linear_RPS + left_angular_RPS;
	double expected_right_RPS = right_linear_RPS + right_angular_RPS;

	ASSERT_NEAR(actual_left_RPS, expected_left_RPS, 0.05); /* TODO too large an error? */
	ASSERT_NEAR(actual_right_RPS, expected_right_RPS, 0.05);
}

TEST(CalcWheelDirection, Turning) {
	/* angular velocity > 0 => turn ccw */
	double actual_left_RPS = CalcWheelDirection( 10, 'L' );
	double expected_left_RPS = -1;
	double actual_right_RPS = CalcWheelDirection( 10, 'R' );
	double expected_right_RPS = 1;
	ASSERT_EQ(actual_left_RPS, expected_left_RPS);
	ASSERT_EQ(actual_right_RPS, expected_right_RPS);

	/* angular velocity < 0 => turn cw */
	actual_left_RPS = CalcWheelDirection( -71, 'L' );
	expected_left_RPS = 1;
	actual_right_RPS = CalcWheelDirection( -71, 'R' );
	expected_right_RPS = -1;
	ASSERT_EQ(actual_left_RPS, expected_left_RPS);
	ASSERT_EQ(actual_right_RPS, expected_right_RPS);
}

int main(int argc, char **argv) {
  printf("Running main() from gtest_main.cc\n");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

