#ifndef HINOWACPP__SEGMENT_H
#define HINOWACPP__SEGMENT_H

#include <hinowacpp/joint.h>

namespace hinowacpp
{
	template <int T> class Segment
	{
		private:

		public:
			Segment() { };
			~Segment() { };
			int size() const { return T; }
			Joint joints[T];
	};
}

#endif
