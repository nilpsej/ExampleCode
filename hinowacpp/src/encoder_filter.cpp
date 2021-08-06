#include <hinowacpp/encoder_filter.h>

namespace hinowacpp
{
	encoder_filter::encoder_filter()
	{
	}

	encoder_filter::~encoder_filter()
	{

	}


	double encoder_filter::angle(double angle, int i)
	{
		if(angleReads < filterPrevious) angleReads++;
		//printf("Reads: %i\n", angleReads);

		// put value at front of array
		for (int j = filterPrevious - 1; j > 0; j--) {
			previousAngles[i][j] = previousAngles[i][j - 1];
		}
		previousAngles[i][0] = angle;
		//printf("%f, %f, %f, %f.\n", previousAngles[i][0], previousAngles[i][1], previousAngles[i][2], previousAngles[i][3]);

		int filterIterations = filterPrevious;
		if (angleReads < filterPrevious) {
			filterIterations = angleReads;
		}

		double angleSum = 0;
		for (int j = 0; j < filterIterations; j++) {
			angleSum = angleSum + previousAngles[i][j];
		}

		double filterResult = angleSum / (filterIterations * 1.0);
		//filterResult = filterResult*180.0/M_PI;
		//filterResult = ((filterResult*10.0)/10.0)*M_PI/180.0;
		//printf("%f, %f, %f, %i.\n", angle, angleSum, filterResult, filterIterations);
		//printf("%f.\n", filterResult);

		return filterResult;
	}



}

