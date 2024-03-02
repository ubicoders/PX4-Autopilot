
#include "simple_app.hpp"

UbicodersModule::UbicodersModule() : ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::ubicoders_wq)
{
}


UbicodersModule::~UbicodersModule()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool UbicodersModule::init()
{

	ScheduleOnInterval(2500_us); // 2500 us interval, 400 Hz rate

	// Or, execute Run() on every sensor_accel publication
	// if (!_sensor_accel_sub.registerCallback()) {
	// 	PX4_ERR("callback registration failed");
	// 	return false;
	// }

	return true;
}

void UbicodersModule::parameters_update()
{
	in_script_param1 = (int) _ubi_sys_param1.get();
	in_script_param2 = 1.0f/_ubi_sys_param2.get();
}

void UbicodersModule::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	//PX4_INFO("i m here 1");

	// if (_vehicle_status_sub_10hz.updated()) {

	// 	_vehicle_status_sub_10hz.copy(&vcl_status);
	// 	PX4_INFO("vehicle status updated =================");
	// 	PX4_INFO("======================================== timestamp: %llu", vcl_status.timestamp);

	// }else {
	// 	PX4_INFO("vehicle status not updated");
	// }

	// if (_vehicle_status_sub.updated()) {

	// 	_vehicle_status_sub.copy(&vcl_status);
	// 	PX4_INFO("vehicle status updated =================");
	// 	PX4_INFO("======================================== timestamp: %llu", vcl_status.timestamp);

	// }else {
	// 	PX4_INFO("vehicle status not updated");
	// }

	// if (_vehicle_angular_velocity_sub.updated())
	// {
	// 	_vehicle_angular_velocity_sub.copy(&angular_velocity);
	// 	float angvel_x = angular_velocity.xyz[0];
	// 	PX4_INFO("angular vel x: %f", (double) angvel_x);
	// }
	// else {
	// 	PX4_INFO("vehicle ang.vel not updated");
	// }

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
		parameters_update();
	}
	// PX4_INFO("p1: %d", in_script_param1);
	// PX4_INFO("p2: %f", (double) in_script_param2);


	if (_ubi_msg_subsriber.updated()) {
		ubicoders_msg_subs_s ubi_msg;
		_ubi_msg_subsriber.copy(&ubi_msg);
		// PX4_INFO("i m here 2");

		// publish some data
		ubicoders_msg_pub_s data{};
		data.even_number_output = ubi_msg.odd_number_input + 1;
		data.timestamp = hrt_absolute_time();
		_ubi_msg_pub.publish(data);

	} else {
		//PX4_INFO("am i here");
		ubicoders_msg_pub_s data{};
		data.even_number_output = 999;
		data.timestamp = hrt_absolute_time();
		_ubi_msg_pub.publish(data);
	}

	// Example publish some data
	orb_test_s data{};
	data.val = 123456789;
	data.timestamp = hrt_absolute_time();
	_orb_test_pub.publish(data);


	// ubicoders act_out
	act_out_data.timestamp = hrt_absolute_time();
	act_out_data.pin0 = 1200;
	act_out_data.pin1 = 1300;
	_ubi_msg_act_out_publisher.publish(act_out_data);

	perf_end(_loop_perf);
}

int UbicodersModule::task_spawn(int argc, char *argv[])
{
	UbicodersModule *instance = new UbicodersModule();

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

int UbicodersModule::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int UbicodersModule::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int UbicodersModule::print_usage(const char *reason)
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

extern "C" __EXPORT int ubicoders_simple_app_main(int argc, char *argv[])
{
	return UbicodersModule::main(argc, argv);
}
