/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "UbicodersIpsDebug.hpp"

UbicodersIpsDebug::UbicodersIpsDebug() :
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

UbicodersIpsDebug::~UbicodersIpsDebug()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool UbicodersIpsDebug::init()
{
	// execute Run() on every vehicle_visual_odometry publication ...
	if (!_vehicle_visual_odometry_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	// ... and at FALLBACK_INTERVAL_US regardless, so the H-Flow and EKF2 blocks
	// are still visible while no IPS sample arrives (IPS off, cube hidden).
	ScheduleOnInterval(FALLBACK_INTERVAL_US);

	return true;
}

void UbicodersIpsDebug::Run()
{
	if (should_exit()) {
		_vehicle_visual_odometry_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	// A new IPS sample triggers a publication; otherwise the interval tick
	// publishes only if nothing went out for FALLBACK_INTERVAL_US.
	const bool new_odom = _vehicle_visual_odometry_sub.update(&_odom);
	const hrt_abstime now = hrt_absolute_time();

	if (new_odom) {
		_have_odom = true;
		++_ips_seq;

	} else if (now - _last_publish_us < FALLBACK_INTERVAL_US) {
		perf_end(_loop_perf);
		return;
	}

	{
		// latest sample of every other input; each is checked for staleness below
		_vehicle_optical_flow_sub.update(&_flow);
		_distance_sensor_sub.update(&_range);
		_estimator_status_flags_sub.update(&_flags);
		_vehicle_local_position_sub.update(&_lpos);

		ubicoders_ips_debug_s msg{};
		msg.timestamp = now;

		// 1. IPS as received: the last sample, with its age growing while no new
		// one arrives; NaN / 0 before the first sample.
		msg.ips_seq = _ips_seq;

		if (_have_odom) {
			msg.ips_timestamp = _odom.timestamp;
			msg.ips_timestamp_sample = _odom.timestamp_sample;

			const uint64_t age = (msg.timestamp > _odom.timestamp) ? (msg.timestamp - _odom.timestamp) : 0;
			msg.ips_age_us = (age > UINT32_MAX) ? UINT32_MAX : static_cast<uint32_t>(age);

			msg.ips_pose_frame = _odom.pose_frame;

			for (int i = 0; i < 3; i++) {
				msg.ips_position[i] = _odom.position[i];
				msg.ips_velocity[i] = _odom.velocity[i];
			}

			for (int i = 0; i < 4; i++) {
				msg.ips_q[i] = _odom.q[i];
			}

		} else {
			for (int i = 0; i < 3; i++) {
				msg.ips_position[i] = NAN;
				msg.ips_velocity[i] = NAN;
			}

			for (int i = 0; i < 4; i++) {
				msg.ips_q[i] = NAN;
			}
		}

		// 2. H-Flow raw: NaN / 0 when no sample arrived yet or the last one is
		// older than MAX_AGE_US (sensor off or unplugged).
		msg.flow_rate[0] = NAN;
		msg.flow_rate[1] = NAN;
		msg.flow_quality = 0;

		if (is_fresh(_flow.timestamp, msg.timestamp)) {
			if (_flow.integration_timespan_us > 0) {
				const float dt = static_cast<float>(_flow.integration_timespan_us) * 1e-6f;
				msg.flow_rate[0] = _flow.pixel_flow[0] / dt;
				msg.flow_rate[1] = _flow.pixel_flow[1] / dt;
			}

			msg.flow_quality = _flow.quality;
		}

		msg.range_m = NAN;
		msg.range_signal_quality = 0;

		if (is_fresh(_range.timestamp, msg.timestamp)) {
			msg.range_m = _range.current_distance;
			msg.range_signal_quality = (_range.signal_quality < 0) ? 0 : static_cast<uint8_t>(_range.signal_quality);
		}

		// Which EKF2 case is live: what EKF2 fuses, numbered like the EKF switch
		// (bit0 IPS, bit1 H-Flow). NONE when the flags are missing or stale.
		msg.ekf2_case = ubicoders_ips_debug_s::EKF2_CASE_NONE;

		if (is_fresh(_flags.timestamp, msg.timestamp)) {
			msg.cs_opt_flow = _flags.cs_opt_flow;
			msg.cs_rng_hgt = _flags.cs_rng_hgt;
			msg.cs_ev_pos = _flags.cs_ev_pos;
			msg.cs_ev_hgt = _flags.cs_ev_hgt;
			msg.cs_ev_yaw = _flags.cs_ev_yaw;

			uint8_t ekf2_case = ubicoders_ips_debug_s::EKF2_CASE_NONE;

			if (_flags.cs_ev_pos) {
				ekf2_case |= ubicoders_ips_debug_s::EKF2_CASE_IPS;
			}

			if (_flags.cs_opt_flow) {
				ekf2_case |= ubicoders_ips_debug_s::EKF2_CASE_HFLOW;
			}

			msg.ekf2_case = ekf2_case;
		}

		// 3.-5. EKF2 output: vehicle_local_position goes into the block of the
		// live case; the other two blocks (and all three when the position is
		// stale or no case is live) stay NaN / false.
		float *position[3] {msg.ekf2_hflow_position, msg.ekf2_ips_position, msg.ekf2_ips_hflow_position};
		float *velocity[3] {msg.ekf2_hflow_velocity, msg.ekf2_ips_velocity, msg.ekf2_ips_hflow_velocity};

		for (int b = 0; b < 3; b++) {
			for (int i = 0; i < 3; i++) {
				position[b][i] = NAN;
				velocity[b][i] = NAN;
			}
		}

		if (is_fresh(_lpos.timestamp, msg.timestamp)) {
			float *pos = nullptr;
			float *vel = nullptr;

			switch (msg.ekf2_case) {
			case ubicoders_ips_debug_s::EKF2_CASE_HFLOW:
				pos = msg.ekf2_hflow_position;
				vel = msg.ekf2_hflow_velocity;
				msg.ekf2_hflow_xy_valid = _lpos.xy_valid;
				msg.ekf2_hflow_v_xy_valid = _lpos.v_xy_valid;
				break;

			case ubicoders_ips_debug_s::EKF2_CASE_IPS:
				pos = msg.ekf2_ips_position;
				vel = msg.ekf2_ips_velocity;
				msg.ekf2_ips_xy_valid = _lpos.xy_valid;
				msg.ekf2_ips_v_xy_valid = _lpos.v_xy_valid;
				break;

			case ubicoders_ips_debug_s::EKF2_CASE_IPS_HFLOW:
				pos = msg.ekf2_ips_hflow_position;
				vel = msg.ekf2_ips_hflow_velocity;
				msg.ekf2_ips_hflow_xy_valid = _lpos.xy_valid;
				msg.ekf2_ips_hflow_v_xy_valid = _lpos.v_xy_valid;
				break;

			default:
				break;
			}

			if (pos && vel) {
				pos[0] = _lpos.x;
				pos[1] = _lpos.y;
				pos[2] = _lpos.z;
				vel[0] = _lpos.vx;
				vel[1] = _lpos.vy;
				vel[2] = _lpos.vz;
			}
		}

		_ubicoders_ips_debug_pub.publish(msg);
		_last_publish_us = now;

		_last_age_us = msg.ips_age_us;
		_last_pose_frame = msg.ips_pose_frame;
		_last_ekf2_case = msg.ekf2_case;

		for (int i = 0; i < 3; i++) {
			_last_position[i] = msg.ips_position[i];
		}
	}

	perf_end(_loop_perf);
}

int UbicodersIpsDebug::task_spawn(int argc, char *argv[])
{
	UbicodersIpsDebug *instance = new UbicodersIpsDebug();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

const char *UbicodersIpsDebug::case_name(uint8_t ekf2_case)
{
	switch (ekf2_case) {
	case ubicoders_ips_debug_s::EKF2_CASE_IPS: return "IPS only";

	case ubicoders_ips_debug_s::EKF2_CASE_HFLOW: return "H-Flow only";

	case ubicoders_ips_debug_s::EKF2_CASE_IPS_HFLOW: return "IPS + H-Flow";

	default: return "none";
	}
}

int UbicodersIpsDebug::print_status()
{
	PX4_INFO("vehicle_visual_odometry samples: %" PRIu32, _ips_seq);

	if (_ips_seq > 0) {
		PX4_INFO("last position: [%.3f, %.3f, %.3f] m, pose_frame %d, age %" PRIu32 " us",
			 (double)_last_position[0], (double)_last_position[1], (double)_last_position[2],
			 (int)_last_pose_frame, _last_age_us);
		PX4_INFO("ekf2_case: %d (%s)", (int)_last_ekf2_case, case_name(_last_ekf2_case));
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int UbicodersIpsDebug::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int UbicodersIpsDebug::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Debug view of the indoor-positioning fusion cases. On every
`vehicle_visual_odometry` update it publishes one `ubicoders_ips_debug` sample with:

1. the IPS pose as received (timestamps, a running sample counter, the input
   age, pose frame, position, attitude, velocity);
2. the raw H-Flow values (`vehicle_optical_flow`, `distance_sensor`);
3. `ekf2_case`, the fusion case EKF2 is running, derived from
   `estimator_status_flags` (bit0 `cs_ev_pos` = IPS, bit1 `cs_opt_flow` = H-Flow),
   plus the five source flags;
4. `vehicle_local_position` in the one block that matches `ekf2_case`
   (H-Flow only, IPS only, or IPS + H-Flow); the other blocks are NaN / false.

Any input with no sample or a sample older than 1 s reads as NaN / 0 / false
(for the flags: `ekf2_case` none). The uXRCE-DDS client streams the topic as
`/fmu/out/ubicoders_ips_debug`.

### Examples
```
ubicoders_ips_debug start
ubicoders_ips_debug status
listener ubicoders_ips_debug 5
```
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ubicoders_ips_debug", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ubicoders_ips_debug_main(int argc, char *argv[])
{
	return UbicodersIpsDebug::main(argc, argv);
}
