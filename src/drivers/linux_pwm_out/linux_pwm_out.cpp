/****************************************************************************
 *
 *   Copyright (c) 2015-2017 PX4 Development Team. All rights reserved.
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

#include <stdint.h>

#include <px4_tasks.h>
#include <px4_getopt.h>
#include <px4_posix.h>
#include <errno.h>
#include <cmath>	// NAN

#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/rc_channels.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_mixer.h>
#include <systemlib/mixer/mixer.h>
#include <systemlib/mixer/mixer_load.h>
#include <systemlib/param/param.h>
#include <systemlib/pwm_limit/pwm_limit.h>

#include "common.h"
#include "navio_sysfs.h"
#include "PCA9685.h"
#include "ocpoc_mmap.h"


//这个是驱动上层的pwm输出，是用来给linux直接调用的，在linux_pwm_out目录中，其他的文件，都是基本的硬件驱动，但是没有业务逻辑，这个文件加入业务逻辑，可以单独运行。
namespace linux_pwm_out
{
static px4_task_t _task_handle = -1;
volatile bool _task_should_exit = false;
static bool _is_running = false;

static char _device[64] = "/sys/class/pwm/pwmchip0";
static char _protocol[64] = "navio";
static int _max_num_outputs = 8; ///< maximum number of outputs the driver should use  最大输出数量为8，也就是最大支持8轴输出
static char _mixer_filename[64] = "ROMFS/px4fmu_common/mixers/quad_x.main.mix";//混合器的文件地址，

// subscriptions
int     _controls_subs[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
int     _armed_sub = -1;

// publications
orb_advert_t    _outputs_pub = nullptr;
orb_advert_t    _rc_pub = nullptr;

// topic structures
actuator_controls_s _controls[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
orb_id_t 			_controls_topics[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];
actuator_outputs_s  _outputs;
actuator_armed_s    _armed;

// polling
uint8_t _poll_fds_num = 0;
px4_pollfd_struct_t _poll_fds[actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS];

// control groups related
uint32_t	_groups_required = 0;
uint32_t	_groups_subscribed = 0;

pwm_limit_t     _pwm_limit;

// esc parameters
int32_t _pwm_disarmed;
int32_t _pwm_min;
int32_t _pwm_max;

MixerGroup *_mixer_group = nullptr;

void usage();

void start();

void stop();

void task_main_trampoline(int argc, char *argv[]);

void subscribe();

void task_main(int argc, char *argv[]);

/* mixer initialization /混合器初始化*/
int initialize_mixer(const char *mixer_filename);
//回调
int mixer_control_callback(uintptr_t handle, uint8_t control_group, uint8_t control_index, float &input);


int mixer_control_callback(uintptr_t handle,
			   uint8_t control_group,
			   uint8_t control_index,
			   float &input)
{
	const actuator_controls_s *controls = (actuator_controls_s *)handle;
	input = controls[control_group].control[control_index];

	return 0;
}


int initialize_mixer(const char *mixer_filename)
{
	char buf[4096];
	unsigned buflen = sizeof(buf);
	memset(buf, '\0', buflen);

	_mixer_group = new MixerGroup(mixer_control_callback, (uintptr_t) &_controls);

	// PX4_INFO("Trying to initialize mixer from config file %s", mixer_filename);

	if (load_mixer_file(mixer_filename, buf, buflen) == 0) {
		if (_mixer_group->load_from_buf(buf, buflen) == 0) {
			PX4_INFO("Loaded mixer from file %s", mixer_filename);
			return 0;

		} else {
			PX4_ERR("Unable to parse from mixer config file %s", mixer_filename);
		}

	} else {
		PX4_ERR("Unable to load config file %s", mixer_filename);
	}

	if (_mixer_group->count() <= 0) {
		PX4_ERR("Mixer initialization failed");
		return -1;
	}

	return 0;
}


void subscribe()
{
	memset(_controls, 0, sizeof(_controls));
	memset(_poll_fds, 0, sizeof(_poll_fds));

	/* set up ORB topic names */
	_controls_topics[0] = ORB_ID(actuator_controls_0);
	_controls_topics[1] = ORB_ID(actuator_controls_1);
	_controls_topics[2] = ORB_ID(actuator_controls_2);
	_controls_topics[3] = ORB_ID(actuator_controls_3);

	// Subscribe for orb topics
	for (uint8_t i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_groups_required & (1 << i)) {
			PX4_DEBUG("subscribe to actuator_controls_%d", i);
			_controls_subs[i] = orb_subscribe(_controls_topics[i]);

		} else {
			_controls_subs[i] = -1;
		}

		if (_controls_subs[i] >= 0) {
			_poll_fds[_poll_fds_num].fd = _controls_subs[i];
			_poll_fds[_poll_fds_num].events = POLLIN;
			_poll_fds_num++;
		}

		_armed_sub = orb_subscribe(ORB_ID(actuator_armed));

	}
}
//pixhawk程序运行循环函数，是一个线程
void task_main(int argc, char *argv[])
{
	_is_running = true;//设置运行状态为true。

	// Set up mixer
	if (initialize_mixer(_mixer_filename) < 0) {
		PX4_ERR("Mixer initialization failed.");
		return;
	}

	PWMOutBase *pwm_out;

	//各种PWM输出的方式，PCA9685是16路12位pwm信号发生器，pixhawk硬件没有这个芯片
	if (strcmp(_protocol, "pca9685") == 0) {
		PX4_INFO("Starting PWM output in PCA9685 mode");
		pwm_out = new PCA9685();

	} else if (strcmp(_protocol, "ocpoc_mmap") == 0) {
		PX4_INFO("Starting PWM output in ocpoc_mmap mode");
		pwm_out = new OcpocMmapPWMOut(_max_num_outputs);

	} else { /* navio *///pixhawk默认是这个，使用stm32上面的引脚，输出PWM信号
		PX4_INFO("Starting PWM output in Navio mode");
		pwm_out = new NavioSysfsPWMOut(_device, _max_num_outputs);
	}

	if (pwm_out->init() != 0) {
		PX4_ERR("PWM output init failed");
		delete pwm_out;
		return;
	}

	_mixer_group->groups_required(_groups_required);
	// subscribe and set up polling
	subscribe();

	int rc_channels_sub = -1;

	// Start disarmed
	_armed.armed = false;
	_armed.prearmed = false;

	pwm_limit_init(&_pwm_limit);

	while (!_task_should_exit) {

		bool updated;
		orb_check(_armed_sub, &updated);

		if (updated) {
			orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);
		}

		int pret = px4_poll(_poll_fds, _poll_fds_num, 10);

		/* Timed out, do a periodic check for _task_should_exit. */
		if (pret == 0 && !_armed.in_esc_calibration_mode) {
			continue;
		}

		/* This is undesirable but not much we can do. */
		if (pret < 0) {
			PX4_WARN("poll error %d, %d", pret, errno);
			/* sleep a bit before next try */
			usleep(10000);
			continue;
		}

		/* get controls for required topics */
		unsigned poll_id = 0;

		for (uint8_t i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
			if (_controls_subs[i] >= 0) {
				if (_poll_fds[poll_id].revents & POLLIN) {
					orb_copy(_controls_topics[i], _controls_subs[i], &_controls[i]);
				}

				poll_id++;
			}
		}

		if (_armed.in_esc_calibration_mode) {
			if (rc_channels_sub == -1) {
				// only subscribe when really needed: esc calibration is not something we use regularily
				rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));
			}

			rc_channels_s rc_channels;
			int ret = orb_copy(ORB_ID(rc_channels), rc_channels_sub, &rc_channels);
			_controls[0].control[0] = 0.f;
			_controls[0].control[1] = 0.f;
			_controls[0].control[2] = 0.f;
			int channel = rc_channels.function[rc_channels_s::RC_CHANNELS_FUNCTION_THROTTLE];

			if (ret == 0 && channel >= 0 && channel < (int)(sizeof(rc_channels.channels) / sizeof(rc_channels.channels[0]))) {
				_controls[0].control[3] = rc_channels.channels[channel];

			} else {
				_controls[0].control[3] = 1.f;
			}

			/* Switch off the PWM limit ramp for the calibration. */
			_pwm_limit.state = PWM_LIMIT_STATE_ON;
		}

		if (_mixer_group != nullptr) {
			_outputs.timestamp = hrt_absolute_time();
			/* do mixing */
			_outputs.noutputs = _mixer_group->mix(_outputs.output,
							      actuator_outputs_s::NUM_ACTUATOR_OUTPUTS,
							      NULL);

			/* disable unused ports by setting their output to NaN */
			for (size_t i = _outputs.noutputs; i < _outputs.NUM_ACTUATOR_OUTPUTS; i++) {
				_outputs.output[i] = NAN;
			}

			const uint16_t reverse_mask = 0;
			uint16_t disarmed_pwm[actuator_outputs_s::NUM_ACTUATOR_OUTPUTS];
			uint16_t min_pwm[actuator_outputs_s::NUM_ACTUATOR_OUTPUTS];
			uint16_t max_pwm[actuator_outputs_s::NUM_ACTUATOR_OUTPUTS];

			for (unsigned int i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
				disarmed_pwm[i] = _pwm_disarmed;
				min_pwm[i] = _pwm_min;
				max_pwm[i] = _pwm_max;
			}

			uint16_t pwm[actuator_outputs_s::NUM_ACTUATOR_OUTPUTS];

			// TODO FIXME: pre-armed seems broken
			pwm_limit_calc(_armed.armed,
				       false/*_armed.prearmed*/,
				       _outputs.noutputs,
				       reverse_mask,
				       disarmed_pwm,
				       min_pwm,
				       max_pwm,
				       _outputs.output,
				       pwm,
				       &_pwm_limit);

			if (_armed.lockdown || _armed.manual_lockdown) {
				pwm_out->send_output_pwm(disarmed_pwm, _outputs.noutputs);

			} else if (_armed.in_esc_calibration_mode) {

				uint16_t pwm_value;

				if (_controls[0].control[3] > 0.5f) { // use throttle to decide which value to use
					pwm_value = _pwm_max;

				} else {
					pwm_value = _pwm_min;
				}

				for (uint32_t i = 0; i < _outputs.noutputs; ++i) {
					pwm[i] = pwm_value;
				}

				pwm_out->send_output_pwm(pwm, _outputs.noutputs);

			} else {
				pwm_out->send_output_pwm(pwm, _outputs.noutputs);
			}

			if (_outputs_pub != nullptr) {
				orb_publish(ORB_ID(actuator_outputs), _outputs_pub, &_outputs);

			} else {
				_outputs_pub = orb_advertise(ORB_ID(actuator_outputs), &_outputs);
			}

		} else {
			PX4_ERR("Could not mix output! Exiting...");
			_task_should_exit = true;
		}
	}

	delete pwm_out;

	for (uint8_t i = 0; i < actuator_controls_s::NUM_ACTUATOR_CONTROL_GROUPS; i++) {
		if (_controls_subs[i] >= 0) {
			orb_unsubscribe(_controls_subs[i]);
		}
	}

	if (_armed_sub != -1) {
		orb_unsubscribe(_armed_sub);
		_armed_sub = -1;
	}

	if (rc_channels_sub != -1) {
		orb_unsubscribe(rc_channels_sub);
	}

	_is_running = false;

}

void task_main_trampoline(int argc, char *argv[])
{
	task_main(argc, argv);
}

void start()
{
	ASSERT(_task_handle == -1);

	_task_should_exit = false;

	/* start the task */
	_task_handle = px4_task_spawn_cmd("pwm_out_main",
					  SCHED_DEFAULT,
					  SCHED_PRIORITY_MAX,
					  1500,
					  (px4_main_t)&task_main_trampoline,
					  nullptr);

	if (_task_handle < 0) {
		warn("task start failed");
		return;
	}

}

void stop()
{
	_task_should_exit = true;

	while (_is_running) {
		usleep(200000);
		PX4_INFO(".");
	}

	_task_handle = -1;
}

void usage()
{
	PX4_INFO("usage: pwm_out start [-d pwmdevice] [-m mixerfile] [-p protocol]");
	PX4_INFO("       -d pwmdevice : sysfs device for pwm generation (only for Navio)");
	PX4_INFO("                       (default /sys/class/pwm/pwmchip0)");
	PX4_INFO("       -m mixerfile : path to mixerfile");
	PX4_INFO("                       (default ROMFS/px4fmu_common/mixers/quad_x.main.mix)");
	PX4_INFO("       -p protocol : driver output protocol (navio|pca9685|ocpoc_mmap)");
	PX4_INFO("                       (default is navio)");
	PX4_INFO("       -n num_outputs : maximum number of outputs the driver should use");
	PX4_INFO("                       (default is 8)");
	PX4_INFO("       pwm_out stop");
	PX4_INFO("       pwm_out status");
}

} // namespace linux_pwm_out

/* driver 'main' command 命令的执行主程序*/
extern "C" __EXPORT int linux_pwm_out_main(int argc, char *argv[]);

int linux_pwm_out_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	char *verb = nullptr;

	if (argc >= 2) {
		verb = argv[1];

	} else {
		return 1;
	}

	while ((ch = px4_getopt(argc, argv, "d:m:p:n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			strncpy(linux_pwm_out::_device, myoptarg, sizeof(linux_pwm_out::_device));
			break;

		case 'm':
			strncpy(linux_pwm_out::_mixer_filename, myoptarg, sizeof(linux_pwm_out::_mixer_filename));
			break;

		case 'p':
			strncpy(linux_pwm_out::_protocol, myoptarg, sizeof(linux_pwm_out::_protocol));
			break;

		case 'n': {
				unsigned long max_num = strtoul(myoptarg, nullptr, 10);

				if (max_num <= 0) {
					max_num = 8;
				}

				if (max_num > actuator_outputs_s::NUM_ACTUATOR_OUTPUTS) {
					max_num = actuator_outputs_s::NUM_ACTUATOR_OUTPUTS;
				}

				linux_pwm_out::_max_num_outputs = max_num;
			}
			break;
		}
	}

	/** gets the parameters for the esc's pwm */
	param_get(param_find("PWM_DISARMED"), &linux_pwm_out::_pwm_disarmed);
	param_get(param_find("PWM_MIN"), &linux_pwm_out::_pwm_min);
	param_get(param_find("PWM_MAX"), &linux_pwm_out::_pwm_max);

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		if (linux_pwm_out::_is_running) {
			PX4_WARN("pwm_out already running");
			return 1;
		}

		linux_pwm_out::start();
	}

	else if (!strcmp(verb, "stop")) {
		if (!linux_pwm_out::_is_running) {
			PX4_WARN("pwm_out is not running");
			return 1;
		}

		linux_pwm_out::stop();
	}

	else if (!strcmp(verb, "status")) {
		PX4_WARN("pwm_out is %s", linux_pwm_out::_is_running ? "running" : "not running");
		return 0;

	} else {
		linux_pwm_out::usage();
		return 1;
	}

	return 0;
}
