/**************************************************************************
 *
 *   Copyright (c) 2022-2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "../SF45ObstacleMap.hpp"
#include <cassert>
int main()
{
	SF45ObstacleMap uart, can;
	uart.configure(0, 24, 100000); can.configure(0, 24, 100000); uart.reset(); can.reset();
	matrix::Quatf q{};

	for (int angle = -160; angle <= 160; angle += 5) {
		uart.update(float(angle), 1000, 1000, q); can.update(float(angle), 1000, 1000, q);
	}

	auto a = uart.report(1000), b = can.report(1000);
	assert(a.frame == 12 && a.sensor_type == 0 && a.increment == 5 && a.min_distance == 20 && a.max_distance == 5000);

	for (unsigned i = 0; i < 72; ++i) { assert(a.distances[i] == b.distances[i]); }

	assert(a.distances[36] == UINT16_MAX); // rear blind region
	assert(a.distances[0] == 1000);
	assert(uart.expire(101000));
	a = uart.report(101000);

	for (auto d : a.distances) { assert(d == UINT16_MAX); }

	uart.configure(2, 25, 100000); uart.reset(); uart.update(30, 1000, 200000, q);
	a = uart.report(200000); assert(a.distances[66] == 1000); assert(a.angle_offset == 90);
// Pitch 60 degrees halves forward distance; no-return remains no-return.
	matrix::Quatf tilted{matrix::Eulerf(0.f, float(M_PI) / 3.f, 0.f)};
	uart.configure(0, 24, 100000); uart.reset(); uart.update(0, 1000, 300000, tilted);
	a = uart.report(300000); assert(a.distances[0] >= 499 && a.distances[0] <= 500);
	uart.update(5, 65535, 300001, tilted); a = uart.report(300001); assert(a.distances[1] == 5001);
}
