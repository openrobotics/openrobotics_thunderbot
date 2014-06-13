#include <talos_base_controller/ArduinoMessage.h>
#include "parser.cpp"
#include <gtest/gtest.h>
#include <string>

TEST(TestSuite, testExpectedParse)
{
	Parser p;
  std::string input( "100, 1, 0.34343, 0.134139, 0.95381, 4.13431, 8.343, 9.0," );
	ArduinoMessage msg = p.parse(input);
  EXPECT_EQ ( msg.time_stamp , 100 );
  EXPECT_EQ ( msg.e_stop , 1 );
  EXPECT_EQ ( msg.l_motor_rpm , 0.34343f );
  EXPECT_EQ ( msg.r_motor_rpm , 0.134139f );
  EXPECT_EQ ( msg.l_pid , 0.95381f );
  EXPECT_EQ ( msg.r_pid , 4.13431f );
  EXPECT_EQ ( msg.l_feedback , 8.343f );
  EXPECT_EQ ( msg.r_feedback , 9.0f );
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}	
