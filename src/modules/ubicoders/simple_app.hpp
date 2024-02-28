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

//============================================================
#include <uORB/topics/vehicle_angular_velocity.h>
//============================================================

//==================================================================
#include <uORB/topics/ubicoders_msg_subs.h>
#include <uORB/topics/ubicoders_msg_pub.h>
//==================================================================

//=================================
#include <uORB/topics/ubicoders_msg_act_out.h>
//=================================

using namespace time_literals;

class UbicodersModule : public ModuleBase<UbicodersModule>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	UbicodersModule();
	~UbicodersModule() override;

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


	uORB::Publication<ubicoders_msg_act_out_s> _ubi_msg_act_out_publisher{ORB_ID(ubicoders_msg_act_out)};
	ubicoders_msg_act_out_s act_out_data{};

	//==================================================================
	uORB::Publication<ubicoders_msg_pub_s> _ubi_msg_pub{ORB_ID(ubicoders_msg_pub)};
	uORB::Subscription _ubi_msg_subsriber{ORB_ID(ubicoders_msg_subs)};
	//==================================================================

	// Publications
	uORB::Publication<orb_test_s> _orb_test_pub{ORB_ID(orb_test)};


	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _sensor_accel_sub{this, ORB_ID(sensor_accel)}; // subscription that schedules WorkItemExample when updated
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};  // subscription limited to 1 Hz updates
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};			  // regular subscription for additional data

	vehicle_status_s vcl_status;
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	vehicle_angular_velocity_s angular_velocity;

	// uORB::SubscriptionInterval _vehicle_status_sub_10hz{ORB_ID(vehicle_status), 100_ms};
	// vehicle_status_s vcl_status;

	// Performance (perf) counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};

	// Parameters
	int in_script_param1 =-1;
	float in_script_param2 = -0.1f;
	void parameters_update();
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::UBI_SYS_PARAM1>) _ubi_sys_param1,	 /**< example parameter */
		(ParamFloat<px4::params::UBI_SYS_PARAM2>) _ubi_sys_param2     /**< another parameter */
	)

	bool _armed{false};
};
