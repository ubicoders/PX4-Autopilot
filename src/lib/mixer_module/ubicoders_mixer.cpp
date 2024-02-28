#include "mixer_module.hpp"

UbicodersMixer::UbicodersMixer()
{
}

UbicodersMixer::~UbicodersMixer()
{
}

void UbicodersMixer::subsAndUpdate(){

	// If new actuator msg is available, update it
	if (_ubi_msg_act_out_subscriber.update(&_act_out_data)){
		servo_output[0] = _act_out_data.pin0;
		servo_output[1] = _act_out_data.pin1;
		servo_output[2] = _act_out_data.pin2;
		servo_output[3] = _act_out_data.pin3;
	}
}

void UbicodersMixer::update(bool isArmed){

	subsAndUpdate();
	if (!isArmed){
		servo_output[0] = 900;
		servo_output[1] = 900;
		servo_output[2] = 900;
		servo_output[3] = 900;
	}
}


void UbicodersMixer::update(bool isArmed, OutputModuleInterface &_interface){

	update(isArmed);
	_interface.updateOutputs(false, servo_output, 4, 0);
}
