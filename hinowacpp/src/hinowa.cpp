//#include "ros/ros.h"
//#include <stdexcept>
//#include <math.h>
#include <hinowacpp/hinowa.h>
namespace hinowacpp
{
	HINOWA::HINOWA(hinowacpp::CAN_RX_SUBSCRIBER& can_rx, ros::NodeHandle& nh)
	{	
		double j1_min; double j4_min; double j6_min; double j7_min; double j8_min;
		double j1_max; double j4_max; double j6_max; double j7_max; double j8_max;
		nh.getParam("/joint_limits/joint_1/min_position", j1_min); nh.getParam("/joint_limits/joint_4/min_position", j4_min);
		nh.getParam("/joint_limits/joint_1/max_position", j1_max); nh.getParam("/joint_limits/joint_4/max_position", j4_max);
		nh.getParam("/joint_limits/joint_6/min_position", j6_min); nh.getParam("/joint_limits/joint_7/min_position", j7_min);
		nh.getParam("/joint_limits/joint_6/max_position", j6_max); nh.getParam("/joint_limits/joint_7/max_position", j7_max);	
		nh.getParam("/joint_limits/joint_8/min_position", j8_min);
		nh.getParam("/joint_limits/joint_8/max_position", j8_max);
		

		double j1_Vmax; double j4_Vmax; double j6_Vmax; double j7_Vmax; double j8_Vmax;
		nh.getParam("/robot_description_planning/joint_limits/joint_1/max_velocity", j1_Vmax);
		nh.getParam("/robot_description_planning/joint_limits/joint_4/max_velocity", j4_Vmax);
		nh.getParam("/robot_description_planning/joint_limits/joint_6/max_velocity", j6_Vmax);
		nh.getParam("/robot_description_planning/joint_limits/joint_7/max_velocity", j7_Vmax);	
		nh.getParam("/robot_description_planning/joint_limits/joint_8/max_velocity", j8_Vmax);

//--------------------------------------------------------------------------------------------------------------------------
		lift.joints[0].setName("joint_1");
		lift.joints[0].setEncoderId(0x181);
		lift.joints[0].setValveId(0x106); //0x106
		lift.joints[0].setAngleLimits(j1_min, j1_max);
		lift.joints[0].setEncoderRange((j1_max-j1_min)*0.22593669189); //1.183 for full joint range, 1.1794 for rounded, changed to factor dependant on limits: 0.22593669189
		lift.joints[0].setVelocityLimits(j1_Vmax);//rad/s
	
		lift.joints[1].setName("joint_4");
		lift.joints[1].setEncoderId(0x182);
		lift.joints[1].setValveId(0x103);//0x103
		lift.joints[1].setAngleLimits(j4_min, j4_max);
		lift.joints[1].setEncoderRange((j4_max-j4_min)*180/M_PI);
		lift.joints[1].setVelocityLimits(j4_Vmax);//rad/s

		lift.joints[2].setName("joint_6");
		lift.joints[2].setEncoderId(0x183);
		lift.joints[2].setValveId(0x102);//0x102
		lift.joints[2].setAngleLimits(j6_min, j6_max);
		lift.joints[2].setEncoderRange((j6_max-j6_min)*180/M_PI);
		lift.joints[2].setVelocityLimits(j6_Vmax);//rad/s

		lift.joints[3].setName("joint_7");
		lift.joints[3].setEncoderId(0x184);
		lift.joints[3].setValveId(0x105);//0x105
		lift.joints[3].setAngleLimits(j7_min, j7_max);
		lift.joints[3].setEncoderRange(j7_max-j7_min);//m
		lift.joints[3].setVelocityLimits(j7_Vmax);//m/s

		lift.joints[4].setName("joint_8");
		lift.joints[4].setEncoderId(0x185);
		lift.joints[4].setValveId(0x104);//0x104
		lift.joints[4].setAngleLimits(j8_min, j8_max);
		lift.joints[4].setEncoderRange((j8_max-j8_min)*180/M_PI);
		lift.joints[4].setVelocityLimits(j8_Vmax);//rad/s

		double j1_home; double j4_home; double j6_home; double j7_home; double j8_home; 
		nh.getParam("/encoders/joint_1", j1_home); nh.getParam("/encoders/joint_4", j4_home);
		nh.getParam("/encoders/joint_6", j6_home); nh.getParam("/encoders/joint_7", j7_home);
		nh.getParam("/encoders/joint_8", j8_home);
		int num_encoders = 5;
		double initAngles[5] = {j1_home, j4_home, j6_home, j7_home, j8_home};
		for(int i = 0; i < 5; ++i){		
			lift.joints[i].setEncoderLimits(can_rx, initAngles, i); 
		}
	}

	HINOWA::~HINOWA()
	{

	}

	Joint HINOWA::getJoint(std::string jointName)
	{
		int numJointsLift = sizeof(lift.joints) / sizeof(lift.joints[0]);
		for (int i = 0; i < numJointsLift; i++)
		{
			if (lift.joints[i].getName() == jointName)
			{
				return lift.joints[i];
			}
		}
		throw std::runtime_error("Could not find joint with name " + jointName);
	}
}
