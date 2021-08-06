#ifndef HINOWACPP__FILTER_H
#define HINOWACPP__FILTER_H

#include "ros/ros.h"
#include <math.h>

namespace hinowacpp
{
	class encoder_filter
	{
		private:
			int angleReads;
			static const int filterPrevious = 10;
			double previousAngles[5][filterPrevious];//5 rows for 5 joints
		public:
			
			//encoder_filter();
			encoder_filter();
			~encoder_filter();
			double angle(double angle, int i);

	};
}

#endif
