#include "ubicoders_auto_pos.hpp"

UbicodersAutoPosModule::UbicodersAutoPosModule() : ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}


UbicodersAutoPosModule::~UbicodersAutoPosModule()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool UbicodersAutoPosModule::init()
{
	ScheduleOnInterval(10000_us);
	return true;
}

void UbicodersAutoPosModule::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (!_ips_sub.updated()){
		perf_end(_loop_perf);
		return;
	}

	_ips_sub.copy(&_ips);




	// publish msg
	_auto_ctrl_sp.roll = 0.1;
	_auto_ctrl_sp.pitch = 0.2;
	_auto_ctrl_sp.yaw = 0.3;
	_auto_ctrl_sp.thrust = 0.4;
	_auto_control_sp_pub.publish(_auto_ctrl_sp);


	_debug_msg.pos_x = _ips.ips_x;
	_debug_msg.pos_y = _ips.ips_y;
	_debug_msg.pos_z = _ips.ips_z;
	_debug_msg.vdist = _ips.vdist;
	_ubi_debug_pub.publish(_debug_msg);


		


	// //  publish some data
	// orb_test_s data{};
	// data.val = 123456789;
	// data.timestamp = hrt_absolute_time();
	// _orb_test_pub.publish(data);

	perf_end(_loop_perf);
}

int UbicodersAutoPosModule::task_spawn(int argc, char *argv[])
{
	UbicodersAutoPosModule *instance = new UbicodersAutoPosModule();

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

int UbicodersAutoPosModule::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int UbicodersAutoPosModule::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int UbicodersAutoPosModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ubicoders_module", "ubicoders_module_category");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ubicoders_auto_pos_main(int argc, char *argv[])
{
	return UbicodersAutoPosModule::main(argc, argv);
}
