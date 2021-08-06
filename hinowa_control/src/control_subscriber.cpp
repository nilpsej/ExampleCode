//#include <ros/callback_queue.h>
//#include <ros/spinner.h>
/*
#include <std_msgs/Header.h>
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
*/
#include "hinowa_control/control_subscriber.h"

namespace control
{

	CONTROL_SUBSCRIBER::CONTROL_SUBSCRIBER()
	{

	}

	CONTROL_SUBSCRIBER::CONTROL_SUBSCRIBER(ros::NodeHandle nh)
	{
		this->_nh = nh;
		
	}

	CONTROL_SUBSCRIBER::~CONTROL_SUBSCRIBER()
	{

	}
	
	void CONTROL_SUBSCRIBER::storeData(const sensor_msgs::Joy::ConstPtr& joy)
	{
		this->_select = joy->buttons[8];
		this->_start = joy->buttons[9];
		this->_PS = joy->buttons[10];
		//if(_input) ROS_INFO("Current input: %d.\n", _input);
	}


	std::tuple<int, int, int> CONTROL_SUBSCRIBER::subscribe()	
	{
		_outputSelect = 0; _outputStart = 0; _outputPS = 0;
		this->_sub = _nh.subscribe<sensor_msgs::Joy>("/joy", 1000, &CONTROL_SUBSCRIBER::storeData, this);
		//PS button debouncing
		if(_prevPS & _PS) _outputPS = 0;
		else if(_PS) _outputPS = 1;
		_prevPS = _PS;
		//start button debouncing
		if(_prevStart & _start) _outputStart = 0;
		else if(_start) _outputStart = 1;
		_prevStart = _start;
		//select button debouncing
		if(_prevSelect & _select) _outputSelect = 0;
		else if(_select) _outputSelect = 1;
		_prevSelect = _select;

		//printf("Values - input: %d, prevInput: %d, resulting output: %d.\n", _input, _prevInput, _output);
		std::tuple <int, int, int> _output(_outputSelect, _outputStart, _outputPS);	
		return _output;
	}



}


