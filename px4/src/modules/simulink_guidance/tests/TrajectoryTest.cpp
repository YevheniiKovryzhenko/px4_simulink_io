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
#include "../trajectory.hpp"
#include <px4_platform_common/time.h>
#include <cstdio>
#include <limits>
#include <memory>
#include <unistd.h>

class GuidanceTest : public ::testing::Test
{
protected:
	std::unique_ptr<trajectory> guidance;
	uORB::Publication<vehicle_local_position_s> position_pub{ORB_ID(vehicle_local_position)};
	uORB::Publication<sim_guidance_request_s> request_pub{ORB_ID(sim_guidance_request)};
	uORB::Publication<debug_array_s> companion_pub{ORB_ID(companion_guidance_outbound)};
	uORB::Publication<debug_array_s> debug_pub{ORB_ID(simulink_inbound)};
	uORB::Subscription setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription custom_sub{ORB_ID(sim_guidance_trajectory)};
	uORB::Subscription status_sub{ORB_ID(sim_guidance_status)};
	vehicle_local_position_s position{};
	char directory[64] = "/tmp/px4-guidance-test-XXXXXX";

	void SetUp() override
	{
#ifndef ENABLE_LOCKSTEP_SCHEDULER
		GTEST_SKIP() << "Use the lockstep SITL host runner for state-machine tests";
#endif
		ASSERT_NE(mkdtemp(directory), nullptr);
		advance(2000000);
		request_pub.advertise();
		guidance.reset(new trajectory);
		guidance->configure(0, 3);
		guidance->disable();
		position.xy_valid = position.z_valid = position.heading_good_for_control = true;
		position.x = 10.f; position.y = -5.f; position.z = -9.f; position.heading = 2.106f;
		position.vx = 3.f; position.vy = 2.f; position.vz = -1.f;
		fresh();
	}
	void TearDown() override
	{
		guidance.reset();
		for (const char *name : {"valid.traj", "bad.traj"}) {
			const std::string path = std::string(directory) + "/" + name;
			unlink(path.c_str());
		}
		rmdir(directory);
	}
	void advance(uint64_t us)
	{
		const uint64_t now = hrt_absolute_time() + us;
		timespec ts{static_cast<time_t>(now / 1000000), static_cast<long>((now % 1000000) * 1000)};
		ASSERT_EQ(px4_clock_settime(CLOCK_MONOTONIC, &ts), 0);
	}
	void fresh() { position.timestamp = hrt_absolute_time(); position_pub.publish(position); }
	void command(bool start = false, bool execute = false, bool reset = false, bool stop = false, bool home = false)
	{
		sim_guidance_request_s request{};
		request.timestamp = hrt_absolute_time();
		request.start = start; request.start_execution = execute; request.reset = reset;
		request.stop = stop; request.set_home = home;
		request_pub.publish(request);
	}
	sim_guidance_status_s status()
	{
		sim_guidance_status_s result{};
		EXPECT_TRUE(status_sub.copy(&result));
		return result;
	}
	trajectory_setpoint_s setpoint()
	{
		trajectory_setpoint_s result{};
		EXPECT_TRUE(setpoint_sub.copy(&result));
		return result;
	}
	void file(const char *name = "valid.traj", uint8_t axes = 4, float duration = 2.f,
		  float coefficient = 1.f, bool truncate = false, bool mismatch = false)
	{
		const std::string path = std::string(directory) + "/" + name;
		FILE *fp = fopen(path.c_str(), "wb");
		ASSERT_NE(fp, nullptr);
		traj_file_header_t header{2, 1, axes};
		ASSERT_EQ(fwrite(&header, sizeof(header), 1, fp), 1u);
		for (uint8_t axis = 0; axis < axes && !truncate; ++axis) {
			traj_file_data_t row{};
			row.i_dof = axis; row.t_int = duration + ((mismatch && axis == 1) ? 1.f : 0.f);
			row.coefs[1] = coefficient;
			ASSERT_EQ(fwrite(&row, sizeof(row), 1, fp), 1u);
		}
		fclose(fp);
	}
	void load(uint8_t axes = 4) { file("valid.traj", axes); ASSERT_EQ(guidance->set_src(directory, "valid.traj"), 0); }
	void execute() { command(true); command(false, true); guidance->update(); ASSERT_TRUE(status().executing); }
	void companion(bool finished = false, float x = 42.f)
	{
		debug_array_s message{};
		message.timestamp = hrt_absolute_time();
		message.data[0] = 1.f; message.data[1] = finished ? 1.f : 0.f;
		message.data[2] = 0.25f; message.data[3] = x; message.data[4] = 2.f; message.data[5] = -8.f;
		companion_pub.publish(message);
	}
};

TEST_F(GuidanceTest, YawAndDerivativesAreAddedOnce)
{
	load(); execute(); advance(500000); fresh(); guidance->update();
	auto sp = setpoint();
	EXPECT_NEAR(sp.position[0], 10.25f, 1e-5f);
	EXPECT_NEAR(sp.yaw, 2.356f, 1e-5f);
	EXPECT_NEAR(sp.velocity[0], .5f, 1e-5f);
	EXPECT_NEAR(sp.yawspeed, .5f, 1e-5f);
	EXPECT_FLOAT_EQ(sp.acceleration[0], 0.f);
}

TEST_F(GuidanceTest, StandaloneHomeIsAStationaryHold)
{
	load(3); command(false, false, false, false, true); guidance->update();
	auto sp = setpoint();
	EXPECT_FLOAT_EQ(sp.position[2], -9.f);
	EXPECT_FLOAT_EQ(sp.velocity[0], 0.f);
	EXPECT_FLOAT_EQ(sp.yawspeed, 0.f);
	EXPECT_NEAR(sp.yaw, position.heading, 1e-5f);
}

TEST_F(GuidanceTest, InvalidFilesPreserveThePreviousTrajectory)
{
	load();
	for (float duration : {0.f, -1.f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
		file("bad.traj", 4, duration);
		EXPECT_LT(guidance->set_src(directory, "bad.traj"), 0);
	}
	file("bad.traj", 4, 2.f, std::numeric_limits<float>::infinity());
	EXPECT_LT(guidance->set_src(directory, "bad.traj"), 0);
	file("bad.traj", 4, 2.f, 1.f, true);
	EXPECT_LT(guidance->set_src(directory, "bad.traj"), 0);
	file("bad.traj", 4, 2.f, 1.f, false, true);
	EXPECT_LT(guidance->set_src(directory, "bad.traj"), 0);
	EXPECT_STREQ(guidance->file_loader.get_file(), "valid.traj");
	execute(); EXPECT_NEAR(setpoint().yawspeed, .5f, 1e-5f);
}

TEST_F(GuidanceTest, RejectReloadAndHomeDuringExecution)
{
	load(); execute();
	EXPECT_LT(guidance->set_src(directory, "valid.traj"), 0);
	position.x = 100.f; fresh(); command(false, false, false, false, true);
	advance(250000); command(true); command(false, true); guidance->update();
	EXPECT_NEAR(setpoint().position[0], 10.125f, 1e-5f);
}

TEST_F(GuidanceTest, StopWinsAndDisableRequiresNewStart)
{
	load(); execute(); guidance->disable(); advance(1000000); fresh(); guidance->update();
	EXPECT_FALSE(status().started); EXPECT_FALSE(status().executing); EXPECT_FALSE(status().trajectory_valid);
	command(false, true); guidance->update(); EXPECT_FALSE(status().executing);
	command(true); command(false, true); command(false, false, false, true); guidance->update();
	EXPECT_TRUE(status().finished); EXPECT_FALSE(status().executing);
}

TEST_F(GuidanceTest, InvalidOrStaleStateCannotStart)
{
	load(); position.xy_valid = false; fresh(); command(true); guidance->update();
	EXPECT_FALSE(status().started); EXPECT_FALSE(status().trajectory_valid);
	position.xy_valid = true; fresh(); advance(600000); command(true); guidance->update();
	EXPECT_FALSE(status().started);
}

TEST_F(GuidanceTest, EndpointAndRepeatedRun)
{
	load(); execute(); advance(2000001); fresh(); guidance->update();
	EXPECT_TRUE(status().finished); EXPECT_FALSE(status().trajectory_valid);
	EXPECT_NEAR(setpoint().position[0], 11.f, 1e-5f);
	command(true); command(false, true); guidance->update();
	EXPECT_TRUE(status().executing); EXPECT_NEAR(setpoint().position[0], 10.f, 1e-5f);
}

TEST_F(GuidanceTest, EstimatorResetMovesBaseline)
{
	load(); execute();
	position.xy_reset_counter++; position.z_reset_counter++; position.heading_reset_counter++;
	position.delta_xy[0] = 5.f; position.delta_xy[1] = -2.f;
	position.delta_z = 3.f; position.delta_heading = .1f;
	fresh(); guidance->update();
	EXPECT_NEAR(setpoint().position[0], 15.f, 1e-5f);
	EXPECT_NEAR(setpoint().position[2], -6.f, 1e-5f);
	EXPECT_NEAR(setpoint().yaw, 2.206f, 1e-5f);
	guidance->update(); EXPECT_NEAR(setpoint().position[0], 15.f, 1e-5f);
	position.xy_reset_counter += 2; fresh(); guidance->update();
	EXPECT_TRUE(status().finished); EXPECT_FALSE(status().executing);
}

TEST_F(GuidanceTest, DebugArrayIncludesZAndHeading)
{
	load(3); guidance->configure(1, 3);
	debug_array_s input{}; input.timestamp = hrt_absolute_time();
	input.data[35] = 2.f; input.data[36] = 3.f; input.data[37] = -12.f;
	input.data[24] = cosf(.25f); input.data[27] = sinf(.25f);
	debug_pub.publish(input); command(true); guidance->update();
	EXPECT_NEAR(setpoint().position[2], -12.f, 1e-5f);
	EXPECT_NEAR(setpoint().yaw, .5f, 1e-5f);
}

TEST_F(GuidanceTest, CompanionHasThreeAxesWithoutLocalFileAndPublishesEndpoint)
{
	guidance->update(true); command(true); companion(); guidance->update(true);
	command(false, true); companion(); guidance->update(true);
	EXPECT_TRUE(status().executing);
	EXPECT_FLOAT_EQ(setpoint().position[0], 42.f);
	EXPECT_NEAR(setpoint().yaw, position.heading, 1e-5f);
	companion(true, 43.f); guidance->update(true);
	EXPECT_FLOAT_EQ(setpoint().position[0], 43.f); EXPECT_TRUE(status().finished);
}

TEST_F(GuidanceTest, CompanionTimeoutStopsExecution)
{
	load(); guidance->update(true); command(true); companion(); guidance->update(true);
	command(false, true); companion(); guidance->update(true);
	EXPECT_TRUE(status().executing);
	advance(1000001); fresh(); guidance->update(true);
	EXPECT_FALSE(status().loaded); EXPECT_FALSE(status().executing); EXPECT_FALSE(status().trajectory_valid);
}

TEST_F(GuidanceTest, GroundStartThenTakeoffHomeAndExecuteMatchesSimulinkModel)
{
	load();
	// EKF2's final magnetic alignment is deliberately incomplete while landed.
	position.z = -.1656723f;
	position.heading = 2.1122146f;
	position.heading_good_for_control = false;
	fresh();
	command(true, false, true); // Model enters offboard: reset + start.
	guidance->update();
	ASSERT_TRUE(status().started);
	EXPECT_TRUE(status().trajectory_valid);
	EXPECT_FALSE(status().executing);
	EXPECT_NEAR(setpoint().yaw, position.heading, 1e-5f);

	advance(1000000);
	position.z = -10.f;
	fresh();
	// Model completes takeoff: capture airborne home and begin execution together.
	command(false, true, false, false, true);
	guidance->update();
	ASSERT_TRUE(status().executing);
	EXPECT_NEAR(setpoint().position[2], -10.f, 1e-5f);
	advance(100000); fresh(); guidance->update();
	EXPECT_TRUE(status().executing);
	EXPECT_NEAR(setpoint().position[2], -9.95f, 1e-5f);
}
