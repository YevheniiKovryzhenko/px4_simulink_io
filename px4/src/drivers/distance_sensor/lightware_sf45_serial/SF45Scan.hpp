#pragma once

#include <cstdint>
#include <cstddef>

// Only measured bins are valid. A new visit starts a new minimum, even when
// the previous visit has not expired yet (e.g. a reversal or a narrow sweep).
template<size_t BinCount>
class SF45Scan
{
public:
	void reset()
	{
		for (size_t i = 0; i < BinCount; ++i) {
			_distances[i] = UINT16_MAX;
			_timestamps[i] = 0;
		}

		_previous_bin = BinCount;
	}

	void update(size_t bin, uint16_t distance, uint64_t now, uint64_t timeout)
	{
		if (bin >= BinCount) {
			return;
		}

		if (bin != _previous_bin || now - _timestamps[bin] >= timeout || distance < _distances[bin]) {
			_distances[bin] = distance;
		}

		_timestamps[bin] = now;
		_previous_bin = bin;
	}

	bool expire(uint64_t now, uint64_t timeout)
	{
		bool changed = false;

		for (size_t i = 0; i < BinCount; ++i) {
			if (_distances[i] != UINT16_MAX && now - _timestamps[i] >= timeout) {
				_distances[i] = UINT16_MAX;
				changed = true;
			}
		}

		return changed;
	}

	uint16_t distance(size_t bin) const { return _distances[bin]; }

private:
	uint16_t _distances[BinCount] {};
	uint64_t _timestamps[BinCount] {};
	size_t _previous_bin{BinCount};
};
