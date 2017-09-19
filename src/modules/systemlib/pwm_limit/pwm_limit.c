/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file pwm_limit.c
 *
 * Library for PWM output limiting
 *
 * @author Julian Oes <julian@px4.io>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "pwm_limit.h"
#include <math.h>
#include <stdbool.h>
#include <drivers/drv_hrt.h>
#include <stdio.h>

#define PROGRESS_INT_SCALING	10000
//pwm限制初始化
void pwm_limit_init(pwm_limit_t *limit)
{
	limit->state = PWM_LIMIT_STATE_INIT;
	limit->time_armed = 0;
}

/**
 * pwm限制计算
 * //input：armed锁定，pre_armed预锁定，num_channels一共多少个通道，reverse_mask，disarmed_pwm解锁后的pwm，min_pwm最小pwm，max_pwm最大pwm，
 * ，output输出，effective_pwm有效pwm，limit限制
 *
 * 说明，armed置1位解锁电机，pre-armed置1为开始解锁电机
 */
void pwm_limit_calc(const bool armed, const bool pre_armed, const unsigned num_channels, const uint16_t reverse_mask,
		    const uint16_t *disarmed_pwm, const uint16_t *min_pwm, const uint16_t *max_pwm,
		    const float *output, uint16_t *effective_pwm, pwm_limit_t *limit)
{

	/* first evaluate state changes 首先评估状态改变*/
	switch (limit->state) {
	case PWM_LIMIT_STATE_INIT:

		if (armed) {

			/* set arming time for the first call */
			if (limit->time_armed == 0) {//第一次调用的时候，会记录初试时间，
				limit->time_armed = hrt_absolute_time();
			}

			if (hrt_elapsed_time(&limit->time_armed) >= INIT_TIME_US) {//如果解锁的时间间隔大于初始运行时间设定
				limit->state = PWM_LIMIT_STATE_OFF;//那么久置状态为off
			}
		}

		break;

	case PWM_LIMIT_STATE_OFF:
		if (armed) {
			limit->state = PWM_LIMIT_STATE_RAMP;

			/* reset arming time, used for ramp timing */
			limit->time_armed = hrt_absolute_time();//重置解锁开始时间为当前绝对时间
		}

		break;

	case PWM_LIMIT_STATE_RAMP://这个状态是启动到怠速，运算时间，程序叫RAMP时间，平滑时间
		if (!armed) {
			limit->state = PWM_LIMIT_STATE_OFF;//如果被锁定，置位off

		} else if (hrt_elapsed_time(&limit->time_armed) >= RAMP_TIME_US) {
			limit->state = PWM_LIMIT_STATE_ON;//超过平滑时间后，置位ON正常运转状态
		}

		break;

	case PWM_LIMIT_STATE_ON:
		if (!armed) {
			limit->state = PWM_LIMIT_STATE_OFF;//如果被锁定，置位off
		}

		break;

	default:
		break;
	}

	/* if the system is pre-armed, the limit state is temporarily on,
	 * as some outputs are valid and the non-valid outputs have been
	 * set to NaN. This is not stored in the state machine though,
	 * as the throttle channels need to go through the ramp at
	 * regular arming time.
	 *
如果系统是预制的，那么限制状态是暂时的，
*由于某些输出是有效的，而非有效输出已经存在
*将NaN。这不是存储在状态机中，
*当油门通道需要通过斜坡时
*定期施法时间。
	 *
	 */
	unsigned local_limit_state = limit->state;

	if (pre_armed) {
		local_limit_state = PWM_LIMIT_STATE_ON;
	}

	unsigned progress;

	/* then set effective_pwm based on state */
	switch (local_limit_state) {
	case PWM_LIMIT_STATE_OFF:
	case PWM_LIMIT_STATE_INIT:
		for (unsigned i = 0; i < num_channels; i++) {
			effective_pwm[i] = disarmed_pwm[i];
		}
		break;
	case PWM_LIMIT_STATE_RAMP: {//一下是对平滑过程的处理
			hrt_abstime diff = hrt_elapsed_time(&limit->time_armed);

			progress = diff * PROGRESS_INT_SCALING / RAMP_TIME_US;

			if (progress > PROGRESS_INT_SCALING) {
				progress = PROGRESS_INT_SCALING;
			}

			for (unsigned i = 0; i < num_channels; i++) {

				float control_value = output[i];//将上面计算到的数值，再算一遍，即使是启动，都要把姿态情况放进去运算。

				/* check for invalid / disabled channels */
				if (!isfinite(control_value)) {
					effective_pwm[i] = disarmed_pwm[i];
					continue;
				}
				uint16_t ramp_min_pwm;
				/* if a disarmed pwm value was set, blend between disarmed and min */
				if (disarmed_pwm[i] > 0) {
					/* safeguard against overflows */
					unsigned disarmed = disarmed_pwm[i];
					if (disarmed > min_pwm[i]) {
						disarmed = min_pwm[i];
					}
					unsigned disarmed_min_diff = min_pwm[i] - disarmed;
					ramp_min_pwm = disarmed + (disarmed_min_diff * progress) / PROGRESS_INT_SCALING;
				} else {
					/* no disarmed pwm value set, choose min pwm */
					ramp_min_pwm = min_pwm[i];
				}

				if (reverse_mask & (1 << i)) {
					control_value = -1.0f * control_value;
				}

				effective_pwm[i] = control_value * (max_pwm[i] - ramp_min_pwm) / 2 + (max_pwm[i] + ramp_min_pwm) / 2;

				/* last line of defense against invalid inputs */
				if (effective_pwm[i] < ramp_min_pwm) {
					effective_pwm[i] = ramp_min_pwm;

				} else if (effective_pwm[i] > max_pwm[i]) {
					effective_pwm[i] = max_pwm[i];
				}
			}
		}
		break;
	case PWM_LIMIT_STATE_ON:
		for (unsigned i = 0; i < num_channels; i++) {//对每一个通道做处理
			float control_value = output[i];
			/* check for invalid / disabled channels */
			if (!isfinite(control_value)) {
				effective_pwm[i] = disarmed_pwm[i];
				continue;
			}

			if (reverse_mask & (1 << i)) {
				control_value = -1.0f * control_value;
			}
//这个也是精华，
			effective_pwm[i] = control_value * (max_pwm[i] - min_pwm[i]) / 2 + (max_pwm[i] + min_pwm[i]) / 2;

			/* last line of defense against invalid inputs 限制非法值*/
			if (effective_pwm[i] < min_pwm[i]) {
				effective_pwm[i] = min_pwm[i];

			} else if (effective_pwm[i] > max_pwm[i]) {
				effective_pwm[i] = max_pwm[i];
			}

		}

		break;

	default:
		break;
	}

}
