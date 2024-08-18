#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <matrix/matrix/math.hpp>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/orb_test.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_attitude.h>

#include <uORB/topics/ubicoders_msg_ips.h>
#include <uORB/topics/ubicoders_msg_debug.h>
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

    void error_integrator();

    void clamp(float &val, float limit);

private:
	void Run() override;


	// Publications
	uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};
	uORB::Publication<ubicoders_msg_auto_control_setpoint_s> _auto_control_sp_pub{ORB_ID(ubicoders_msg_auto_control_setpoint)};
	uORB::Publication<ubicoders_msg_debug_s> _ubi_debug_pub{ORB_ID(ubicoders_msg_debug)};

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)}; // subscription that schedules WorkItemExample when updated
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};  // subscription limited to 1 Hz updates
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};					  // regular subscription for additional data
	uORB::Subscription _ips_sub{ORB_ID(ubicoders_msg_ips)};

	ubicoders_msg_ips_s _ips {};
	ubicoders_msg_debug_s _debug_msg {};
	ubicoders_msg_auto_control_setpoint_s _auto_ctrl_sp {};
    vehicle_attitude_s _vehicle_attitude {};
	
    
    float _pos[3] = {0, 0, 0};
    float _pos_err[3] = {0, 0, 0};
    float _pos_err_int[3] = {0, 0, 0};
    float _pos_err_limit[3] = {1, 1, 1};
    float _kp[3] = {0, 0, 0};
    float _ki[3] = {0, 0, 0};
    float _kd[3] = {0, 0, 0};
    float _vel_sp[3] = {0, 0, 0};




	// Performance (perf) counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};

	bool _armed{false};
};