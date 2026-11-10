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

#pragma once
#include "sensor_bridge.hpp"
#include <parameters/param.h>
#include <ardupilot/equipment/proximity_sensor/Proximity.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/sf45_sample.h>

class UavcanSF45Bridge : public UavcanSensorBridgeBase
{
public:
	UavcanSF45Bridge(uavcan::INode &node) :
		UavcanSensorBridgeBase("uavcan_sf45", ORB_ID(sf45_sample)), _subscriber(node) {}
	const char *get_name() const override { return "sf45"; }
	int init() override
	{
		param_get(param_find("SF45C_NODE"), &_source_node);
		param_get(param_find("SF45C_SENSOR"), &_sensor_id);
		return _subscriber.start(Callback(this, &UavcanSF45Bridge::receive));
	}
private:
	using Proximity = ardupilot::equipment::proximity_sensor::Proximity;
	void receive(const uavcan::ReceivedDataStructure<Proximity> &msg)
	{
		// Flag 1 is the paired AP_Periph sensor-frame SF45 extension.
		if (msg.getSrcNodeID().get() != _source_node || msg.sensor_id != _sensor_id
		    || msg.flags != 1 || msg.reading_type != Proximity::READING_TYPE_GOOD
		    || !PX4_ISFINITE(msg.yaw) || !PX4_ISFINITE(msg.distance) || !PX4_ISFINITE(msg.pitch)
		    || msg.yaw < -160.f || msg.yaw > 160.f || fabsf(msg.pitch) > 0.001f
		    || msg.distance < 0.f || msg.distance > 656.f) { return; }

		sf45_sample_s sample{};
		sample.timestamp = hrt_absolute_time();
		sample.yaw = msg.yaw;
		sample.distance = msg.distance;
		sample.node_id = msg.getSrcNodeID().get();
		sample.sensor_id = msg.sensor_id;
		_publication.publish(sample);
	}
	using Callback = uavcan::MethodBinder<UavcanSF45Bridge *,
	      void (UavcanSF45Bridge::*)(const uavcan::ReceivedDataStructure<Proximity> &)>;
	uavcan::Subscriber<Proximity, Callback> _subscriber;
	int32_t _source_node{0};
	int32_t _sensor_id{0};
	uORB::Publication<sf45_sample_s> _publication{ORB_ID(sf45_sample)};
};
