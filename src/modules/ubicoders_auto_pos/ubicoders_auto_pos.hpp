#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>

#include <uORB/topics/ubicoders_msg_ips.h>
#include <uORB/topics/ubicoders_msg_auto_control_setpoint.h>

using namespace time_literals;

class UbicodersAutoPosModule : public ModuleBase<UbicodersAutoPosModule>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	UbicodersAutoPosModule();
	~UbicodersAutoPosModule() override;

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
	uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};
	uORB::Publication<orb_test_s> _auto_control_sp{ORB_ID(ubicoders_msg_auto_control_setpoint)};


	// Subscriptions
	
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)}; // subscription that schedules WorkItemExample when updated
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};  // subscription limited to 1 Hz updates
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};					  // regular subscription for additional data
	uORB::Subscription _ips_sub{ORB_ID(ubicoders_msg_ips)};

	ubicoders_msg_ips_s _ips {};

	// Performance (perf) counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};

	bool _armed{false};
};