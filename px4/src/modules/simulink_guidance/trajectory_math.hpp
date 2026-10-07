/****************************************************************************
 *
 *    Copyright (C) 2024  Yevhenii Kovryzhenko. All rights reserved.
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Affero General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Affero General Public License Version 3 for more details.
 *
 *    You should have received a copy of the
 *    GNU Affero General Public License Version 3
 *    along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions, and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions, and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *    3. No ownership or credit shall be claimed by anyone not mentioned in
 *       the above copyright statement.
 *    4. Any redistribution or public use of this software, in whole or in part,
 *       whether standalone or as part of a different project, must remain
 *       under the terms of the GNU Affero General Public License Version 3,
 *       and all distributions in binary form must be accompanied by a copy of
 *       the source code, as stated in the GNU Affero General Public License.
 *
 ****************************************************************************/

#pragma once

#include <matrix/math.hpp>
#include <cmath>

namespace guidance
{
// Coefficients are ascending powers of normalized segment time. Evaluate all
// five derivatives together; their units are converted back to seconds below.
// Returns 1 at/after the endpoint, 0 during execution, or -1 for invalid data.
template<size_t NC, size_t ND, size_t NI>
int evaluate(const matrix::Vector<matrix::Vector<matrix::Vector<float, NC>, ND>, NI> &coefficients,
	     const matrix::Vector<float, NI> &durations, size_t n_coeffs, size_t n_dofs, size_t n_segments,
	     double time, matrix::Vector<float, ND> (&output)[5])
{
	if (n_coeffs == 0 || n_coeffs > NC || n_dofs == 0 || n_dofs > ND
	    || n_segments == 0 || n_segments > NI || !std::isfinite(time)) {
		return -1;
	}
	if (time < 0.0) { time = 0.0; }

	size_t segment = n_segments - 1;
	double start = 0.0;
	double total = 0.0;
	bool found = false;
	for (size_t i = 0; i < n_segments; ++i) {
		if (!std::isfinite(durations(i)) || durations(i) <= 0.0f) { return -1; }
		const double end = total + static_cast<double>(durations(i));
		if (!found && time <= end) {
			segment = i;
			start = total;
			found = true;
		}
		total = end;
	}
	const bool finished = time >= total;
	const double tau = finished ? 1.0 : (time - start) / static_cast<double>(durations(segment));
	const double inverse_duration = 1.0 / static_cast<double>(durations(segment));
	for (auto &vector : output) { vector.setZero(); }

	for (size_t axis = 0; axis < n_dofs; ++axis) {
		double derivatives[5]{};
		for (size_t coefficient = n_coeffs; coefficient-- > 0;) {
			for (int order = 4; order > 0; --order) {
				derivatives[order] = derivatives[order] * tau + order * derivatives[order - 1];
			}
			derivatives[0] = derivatives[0] * tau + static_cast<double>(coefficients(segment)(axis)(coefficient));
		}
		double scale = 1.0;
		for (int order = 0; order < 5; ++order) {
			output[order](axis) = static_cast<float>(derivatives[order] * scale);
			if (!std::isfinite(output[order](axis))) { return -1; }
			scale *= inverse_duration;
		}
	}
	return finished ? 1 : 0;
}
} // namespace guidance
