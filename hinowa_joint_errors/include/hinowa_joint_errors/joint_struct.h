#ifndef joint_struct
#define joint_struct

struct Position{
	float desired;
	float actual;
	float p_error;
	float prev_p_error;
	float i_error;
	float d_error;
};

struct Velocity{
	float desired;
	float actual;
	float p_error;
	float prev_p_error;
	float i_error;
	float d_error;
};

struct Acceleration{
	float desired;
	float error;
};

struct Joint{
	std::string name;
	Position position;
	Velocity velocity;
	Acceleration acceleration;
};

#endif
