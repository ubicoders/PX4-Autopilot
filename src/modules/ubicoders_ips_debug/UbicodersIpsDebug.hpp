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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/estimator_status_flags.h>
#include <uORB/topics/ubicoders_ips_debug.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_optical_flow.h>

/**
 * On every vehicle_visual_odometry sample publishes one ubicoders_ips_debug: the IPS pose
 * as received, the raw H-Flow values, which EKF2 case is live
 * (estimator_status_flags) and the EKF2 output in the block of that case.
 */
class UbicodersIpsDebug : public ModuleBase<UbicodersIpsDebug>, public px4::ScheduledWorkItem
{
public:
	UbicodersIpsDebug();
	~UbicodersIpsDebug() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	// Publications
	uORB::Publication<ubicoders_ips_debug_s> _ubicoders_ips_debug_pub{ORB_ID(ubicoders_ips_debug)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _vehicle_visual_odometry_sub{this, ORB_ID(vehicle_visual_odometry)}; // schedules Run()

	// Latest samples, read on each vehicle_visual_odometry update
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_optical_flow_sub{ORB_ID(vehicle_optical_flow)};
	uORB::Subscription _distance_sensor_sub{ORB_ID(distance_sensor)};
	uORB::Subscription _estimator_status_flags_sub{ORB_ID(estimator_status_flags)};

	vehicle_optical_flow_s _flow{};
	distance_sensor_s _range{};
	estimator_status_flags_s _flags{};
	vehicle_local_position_s _lpos{};

	// last IPS sample (block 1) and the fallback publication timer
	vehicle_odometry_s _odom{};
	bool _have_odom{false};
	hrt_abstime _last_publish_us{0};
	static constexpr uint64_t FALLBACK_INTERVAL_US = 100'000; // 10 Hz while no IPS sample arrives

	// a sample older than this counts as missing (NaN / 0 / false in the output)
	static constexpr uint64_t MAX_AGE_US = 1'000'000;

	static bool is_fresh(uint64_t sample_timestamp, uint64_t now)
	{
		return sample_timestamp != 0 && (now < sample_timestamp || now - sample_timestamp < MAX_AGE_US);
	}

	static const char *case_name(uint8_t ekf2_case);

	uint32_t _ips_seq{0};
	uint32_t _last_age_us{0};
	uint8_t _last_pose_frame{0};
	float _last_position[3] {NAN, NAN, NAN};
	uint8_t _last_ekf2_case{ubicoders_ips_debug_s::EKF2_CASE_NONE};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
