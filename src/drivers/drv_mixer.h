/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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

/**
 * @file drv_mixer.h
 *
 * Mixer ioctl interfaces.
 *
 * Normal workflow is:
 *
 * - open mixer device
 * - add mixer(s)
 * loop:
 *  - mix actuators to array
 *
 * Each client has its own configuration.
 *
 * When mixing, outputs are produced by mixers in the order they are
 * added.  A simple mixer produces one output; a multirotor mixer will
 * produce several outputs, etc.
 */

#ifndef _DRV_MIXER_H
#define _DRV_MIXER_H

#include <px4_defines.h>
#include <stdint.h>
#include <sys/ioctl.h>

#define MIXER0_DEVICE_PATH		"/dev/mixer0"

/*
 * ioctl() definitions
 */
#define _MIXERIOCBASE		(0x2500)
#define _MIXERIOC(_n)		(_PX4_IOC(_MIXERIOCBASE, _n))

/** get the number of mixable outputs */
#define MIXERIOCGETOUTPUTCOUNT	_MIXERIOC(0)

/** reset (clear) the mixer configuration */
#define MIXERIOCRESET		_MIXERIOC(1)

/** simple channel scaler */
struct mixer_scaler_s {
	float			negative_scale;;//负向缩放, MIX文件中 O: 后面的第1个整数/10000.0f
	float			positive_scale;//正向缩放, MIX文件中 O: 后面的第2个整数/10000.0f
	float			offset; //偏移 , MIX文件中 O: 后面的第3个整数/10000.0f
	float			min_output;//最小输出值 , MIX文件中 O: 后面的第4个整数/10000.0f
	float			max_output;//最大输出值 , MIX文件中 O: 后面的第5个整数/10000.0f
};//该结构定义了单个控制量的结构

/** mixer input */
struct mixer_control_s {
	uint8_t			control_group;	/**< group from which the input reads */
	uint8_t			control_index;	/**< index within the control group */
	struct mixer_scaler_s 	scaler;		/**< scaling applied to the input before use */
};//定义输入量的结构

/** simple mixer */
struct mixer_simple_s {
	uint8_t			control_count;	/**< number of inputs 23.
	//因为一个mixer只有一个输出，可以有0到多个输入，所以control_count指明了这个mixer所需要的输入信号数量，而具体的信号都存放在数组controls[0]中。*/
	struct mixer_scaler_s	output_scaler;	/**< scaling for the output 输出则由output_scaler来控制. */
	struct mixer_control_s	controls[0];	/**< actual size of the array is set by control_count */
};//定义了一个控制实体的控制体，包括输入的信号数量，输入信号控制集，输出信号控制。

#define MIXER_SIMPLE_SIZE(_icount)	(sizeof(struct mixer_simple_s) + (_icount) * sizeof(struct mixer_control_s))

/**
 * add a simple mixer in (struct mixer_simple_s *)arg
 */
#define MIXERIOCADDSIMPLE	_MIXERIOC(2)

/* _MIXERIOC(3) was deprecated */
/* _MIXERIOC(4) was deprecated */

/**
 * Add mixer(s) from the buffer in (const char *)arg
 */
#define MIXERIOCLOADBUF		_MIXERIOC(5)

/*
 * XXX Thoughts for additional operations:
 *
 * - get/set output scale, for tuning center/limit values.
 * - save/serialise for saving tuned mixers.
 */

#endif /* _DRV_ACCEL_H */
