#include <hinowa_joint_errors/joint_error_publisher.h>

namespace hinowa_joint_errors
{

	JOINT_ERROR_PUBLISHER::JOINT_ERROR_PUBLISHER()
	{
	}

	JOINT_ERROR_PUBLISHER::JOINT_ERROR_PUBLISHER(ros::NodeHandle nh)
	{
		this->_nh = nh;
		this->_pub = nh.advertise<hinowa_joint_errors::PID_terms>("/hinowa/PID_error_terms", 1000); 
	}

	JOINT_ERROR_PUBLISHER::~JOINT_ERROR_PUBLISHER()
	{
		
	}

	void JOINT_ERROR_PUBLISHER::addErrorTerms(std::vector<Joint> joint_errors_, std::vector<float> actual_accelerations_, std::vector<double> joint_commands_)
	{
		joint_errors = joint_errors_;
		actual_accelerations = actual_accelerations_;
		joint_commands = joint_commands_;

		acceleration_errors.resize(joint_errors.size());
		for(int i=0; i<joint_errors.size(); i++){
			acceleration_errors.at(i) = joint_errors.at(i).acceleration.desired-actual_accelerations_.at(i);
		}
	}

	void JOINT_ERROR_PUBLISHER::publish()
	{
		hinowa_joint_errors::PID_terms msg;
		msg.header.stamp = ros::Time::now();
		for(int i=0; i<joint_errors.size(); i++){
			msg.joint_names.push_back(joint_errors.at(i).name);
//positions
			msg.Pp_errors.push_back(joint_errors.at(i).position.p_error);
			msg.Pi_errors.push_back(joint_errors.at(i).position.i_error);
			msg.Pd_errors.push_back(joint_errors.at(i).position.d_error);
//velocities
			msg.Vp_errors.push_back(joint_errors.at(i).velocity.p_error);
			msg.Vi_errors.push_back(joint_errors.at(i).velocity.i_error);
			msg.Vd_errors.push_back(joint_errors.at(i).velocity.d_error);
//additional
			msg.actual_accelerations.push_back(actual_accelerations.at(i));
			msg.acceleration_errors.push_back(acceleration_errors.at(i));
			msg.joint_commands.push_back(joint_commands.at(i));
		}
		_pub.publish(msg);
		ros::spinOnce();
	}
}
