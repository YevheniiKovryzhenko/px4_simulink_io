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

#include <gtest/gtest.h>
#include "../trajectory_math.hpp"
#include "../file_loader_backend.hpp"
#include "../trajectory.hpp"
#include <dirent.h>
#include <cstdio>
#include <limits>
#include <random>

using Coefficients = matrix::Vector<matrix::Vector<matrix::Vector<float, 10>, 4>, 50>;
using Durations = matrix::Vector<float, 50>;
using Output = matrix::Vector<float, 4>[5];

// Direct powers in long double provide an independent reference for the
// differentiated Horner recurrence, including derivative scaling in seconds.
static long double reference(const Coefficients &coefficients, size_t segment, size_t axis,
			     size_t count, unsigned order, long double tau, long double duration)
{
	long double sum = 0;
	for (size_t k = order; k < count; ++k) {
		long double factor = 1;
		for (unsigned j = 0; j < order; ++j) { factor *= k - j; }
		sum += coefficients(segment)(axis)(k) * factor * powl(tau, k - order);
	}
	return sum / powl(duration, order);
}

TEST(TrajectoryMath, RandomPolynomialsAndAllDerivativeOrders)
{
	std::mt19937 rng(42);
	std::uniform_real_distribution<float> coefficient(-100.f, 100.f);
	Coefficients c{}; Durations duration{}; Output out{};
	for (size_t count = 1; count <= 10; ++count) {
		for (float seconds : {.05f, .5f, 1.f, 10.f, 100.f}) {
			duration(0) = seconds;
			for (size_t axis = 0; axis < 4; ++axis) {
				for (size_t k = 0; k < count; ++k) { c(0)(axis)(k) = coefficient(rng); }
			}
			for (int sample = 0; sample <= 100; ++sample) {
				const double tau = sample / 100.0;
				ASSERT_GE(guidance::evaluate(c, duration, count, 4, 1, tau * seconds, out), 0);
				for (size_t axis = 0; axis < 4; ++axis) for (unsigned order = 0; order < 5; ++order) {
					const double expected = static_cast<double>(reference(c, 0, axis, count, order, tau, seconds));
					EXPECT_NEAR(out[order](axis), expected, 1e-6 * std::max(1.0, fabs(expected)));
				}
			}
		}
	}
}

TEST(TrajectoryMath, SegmentBoundariesClampingAndUnusedAxes)
{
	Coefficients c{}; Durations duration{}; Output out{};
	duration(0) = 2.f; duration(1) = 3.f;
	c(0)(0)(0) = 1.f; c(0)(0)(1) = 2.f;
	c(1)(0)(0) = 3.f; c(1)(0)(1) = 3.f;
	EXPECT_EQ(guidance::evaluate(c, duration, 2, 1, 2, -1.0, out), 0);
	EXPECT_FLOAT_EQ(out[0](0), 1.f);
	EXPECT_EQ(guidance::evaluate(c, duration, 2, 1, 2, 2.0, out), 0);
	EXPECT_FLOAT_EQ(out[0](0), 3.f); EXPECT_FLOAT_EQ(out[1](0), 1.f);
	EXPECT_EQ(guidance::evaluate(c, duration, 2, 1, 2, 3.5, out), 0);
	EXPECT_FLOAT_EQ(out[0](0), 4.5f);
	EXPECT_EQ(guidance::evaluate(c, duration, 2, 1, 2, 10.0, out), 1);
	EXPECT_FLOAT_EQ(out[0](0), 6.f);
	for (auto &v : out) for (int axis = 1; axis < 4; ++axis) { EXPECT_FLOAT_EQ(v(axis), 0.f); }
}

TEST(TrajectoryMath, InvalidDataAndOverflowAreRejected)
{
	Coefficients c{}; Durations duration{}; Output out{};
	EXPECT_LT(guidance::evaluate(c, duration, 2, 1, 0, 0.0, out), 0);
	EXPECT_LT(guidance::evaluate(c, duration, 2, 1, 1, 0.0, out), 0);
	duration(0) = 1.f;
	EXPECT_LT(guidance::evaluate(c, duration, 2, 1, 1, NAN, out), 0);
	c(0)(0)(1) = std::numeric_limits<float>::infinity();
	EXPECT_LT(guidance::evaluate(c, duration, 2, 1, 1, .5, out), 0);
	c(0)(0)(1) = std::numeric_limits<float>::max(); duration(0) = .001f;
	EXPECT_LT(guidance::evaluate(c, duration, 2, 1, 1, .0005, out), 0);
}

TEST(TrajectoryMath, InstalledTrajectoryAssets)
{
	const char *asset_dir = getenv("GUIDANCE_TEST_ASSET_DIR");
	if (!asset_dir) { GTEST_SKIP() << "Set GUIDANCE_TEST_ASSET_DIR to validate exported .traj assets"; }
	DIR *directory = opendir(asset_dir);
	ASSERT_NE(directory, nullptr);
	unsigned files = 0, rejected = 0;
	while (dirent *entry = readdir(directory)) {
		const std::string name = entry->d_name;
		if (name.size() < 5 || name.substr(name.size() - 5) != ".traj") { continue; }
		SCOPED_TRACE(name);
		file_loader_backend loader;
		ASSERT_EQ(loader.set_src(entry->d_name, asset_dir), 0);
		traj_file_header_t header{}; ASSERT_EQ(loader.read_header(header), 0);
		ASSERT_GT(header.n_coeffs, 0); ASSERT_LE(header.n_coeffs, 10);
		ASSERT_GT(header.n_int, 0); ASSERT_LE(header.n_int, 50);
		ASSERT_GT(header.n_dofs, 0); ASSERT_LE(header.n_dofs, 4);
		Coefficients c{}; Durations duration{}; Output out{};
		bool valid = true;
		for (size_t segment = 0; segment < header.n_int; ++segment) {
			for (size_t axis = 0; axis < header.n_dofs; ++axis) {
				traj_file_data_t row{}; ASSERT_EQ(loader.read_data(row), 0);
				ASSERT_EQ(row.i_int, segment); ASSERT_EQ(row.i_dof, axis);
				valid &= std::isfinite(row.t_int) && row.t_int > 0.f; duration(segment) = row.t_int;
				for (size_t k = 0; k < header.n_coeffs; ++k) { c(segment)(axis)(k) = row.coefs[k]; valid &= std::isfinite(row.coefs[k]); }
			}
		}
		trajectory candidate;
		if (!valid) {
			EXPECT_LT(candidate.set_src(asset_dir, entry->d_name), 0);
			++rejected;
			continue;
		}
		ASSERT_EQ(candidate.set_src(asset_dir, entry->d_name), 0);
		double start = 0.0;
		for (size_t segment = 0; segment < header.n_int; ++segment) {
			for (int sample = 1; sample <= 100; ++sample) {
				const double time = start + sample / 100.0 * static_cast<double>(duration(segment));
				ASSERT_GE(guidance::evaluate(c, duration, header.n_coeffs, header.n_dofs, header.n_int, time, out), 0);
				const long double tau = (static_cast<long double>(time) - start) / duration(segment);
				for (size_t axis = 0; axis < header.n_dofs; ++axis) for (unsigned order = 0; order < 5; ++order) {
					const double expected = static_cast<double>(reference(c, segment, axis, header.n_coeffs, order, tau, duration(segment)));
					EXPECT_NEAR(out[order](axis), expected, 1e-6 * std::max(1.0, fabs(expected)));
				}
			}
			start += static_cast<double>(duration(segment));
		}
		++files;
	}
	closedir(directory);
	EXPECT_GT(files, 0u);
	std::printf("Validated %u trajectory assets; rejected %u invalid assets\n", files, rejected);
}
