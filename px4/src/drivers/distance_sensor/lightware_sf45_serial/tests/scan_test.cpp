#include "../SF45Scan.hpp"
#include <cassert>

int main()
{
	SF45Scan<72> scan;
	scan.reset();
	constexpr uint64_t timeout = 100000;

	for (size_t i = 0; i < 72; ++i) {
		assert(scan.distance(i) == UINT16_MAX);
	}

	// Publish the first sample immediately, into its own bin.
	scan.update(10, 400, 1000, timeout);
	assert(scan.distance(10) == 400);
	assert(scan.distance(11) == UINT16_MAX);
	scan.update(10, 250, 2000, timeout);
	scan.update(10, 350, 3000, timeout);
	assert(scan.distance(10) == 250);

	// A transition must not move the previous bin's minimum into the next bin.
	scan.update(11, 900, 4000, timeout);
	assert(scan.distance(10) == 250);
	assert(scan.distance(11) == 900);

	// Reversal starts a new visit; do not retain an older minimum indefinitely.
	scan.update(10, 800, 5000, timeout);
	assert(scan.distance(10) == 800);

	// Dropped samples and partial sweeps must not create clear/obstacle readings.
	scan.update(30, 600, 6000, timeout);
	assert(scan.distance(20) == UINT16_MAX);
	scan.update(71, 700, 7000, timeout);
	scan.update(0, 750, 8000, timeout);
	assert(scan.distance(71) == 700);
	assert(scan.distance(0) == 750);
	assert(scan.distance(1) == UINT16_MAX);

	// Invalid indices cannot overwrite state.
	scan.update(72, 1, 9000, timeout);
	assert(scan.distance(0) == 750);

	// Every bin expires independently, including without incoming samples.
	assert(scan.expire(104000, timeout));
	assert(scan.distance(11) == UINT16_MAX);
	assert(scan.distance(10) == 800);
	assert(scan.expire(108000, timeout));
	assert(scan.distance(0) == UINT16_MAX);
	assert(!scan.expire(109000, timeout));

	// Re-entering the same bin after silence must reset the minimum.
	scan.update(0, 1000, 110000, timeout);
	assert(scan.distance(0) == 1000);
	scan.update(0, 1200, 210000, timeout);
	assert(scan.distance(0) == 1200);

	// No-return sentinel remains distinct from unknown; reset clears all data.
	scan.update(1, 5001, 211000, timeout);
	assert(scan.distance(1) == 5001);
	scan.reset();
	assert(scan.distance(0) == UINT16_MAX);
	assert(scan.distance(1) == UINT16_MAX);
	return 0;
}
