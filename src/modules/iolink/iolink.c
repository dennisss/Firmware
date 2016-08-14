/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file iolink.c
 * Minimal application example for PX4 autopilot
 *
 * @author Dennis Shtatnov <densht@gmail.com>
 */

#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <nuttx/fs/ioctl.h>

#include "drivers/drv_pwm_output.h"
//#include <drivers/drv_gpio.h>


#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>



static int _iolink_task;


__EXPORT int iolink_main(int argc, char *argv[]);

int iolink_thread_main(int argc, char *argv[]);

int iolink_main(int argc, char *argv[])
{

	PX4_INFO("Starting daemon...");

	_iolink_task = px4_task_spawn_cmd(
		"iolink",
		SCHED_DEFAULT,
		SCHED_PRIORITY_DEFAULT, // + 40,
		3600,
		iolink_thread_main,
		NULL
		//(char * const *)&argv[0]
	);

	PX4_INFO("exiting");

	return 0;
}


int iolink_thread_main(int argc, char *argv[]) {
	int ret;
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;


	/* Open PWM driver */
	int fd = open(dev, 0);
	if (fd < 0) {
		err(1, "can't open %s", dev);
	}

	/* Open the GPIO driver */
	//int fd_gpio = open(PX4FMU_DEVICE_PATH, 0);
	//if (fd < 0) {
	//	err(1, "can't open %s", dev);
	//}


	/* Set PWM range to 0% to 100% duty cycle for 50Hz */
	struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));
	pwm_values.channel_count = 6; // servo_count;
	pwm_values.values[4] = 90;
	pwm_values.values[5] = 90;
	ret = ioctl(fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);
	if (ret != OK) {
		errx(ret, "failed setting min values");
	}

	pwm_values.values[4] = 20000;
	pwm_values.values[5] = 20000;
	ret = ioctl(fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values);
	if (ret != OK) {
		errx(ret, "failed setting max values");
	}


	/* Subscribe to commands */
	int vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));


	px4_pollfd_struct_t fds[] = {
		{ .fd = vehicle_command_sub,   .events = POLLIN }
	};

	int error_counter = 0;

	while (true) {
		/* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
		int poll_ret = px4_poll(fds, 1, 1000);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			//PX4_ERR("[px4_simple_app] Got no data within a second");

		} else if (poll_ret < 0) {
			/* this is seriously bad - should be an emergency */
			if (error_counter < 10 || error_counter % 50 == 0) {
				/* use a counter to prevent flooding (and slowing us down) */
				PX4_ERR("[iolink] ERROR return value from poll(): %d"
					, poll_ret);
			}

			error_counter++;

		} else {

			if (fds[0].revents & POLLIN) {

				struct vehicle_command_s cmd;

				orb_copy(ORB_ID(vehicle_command), vehicle_command_sub, &cmd);

				if (cmd.command == VEHICLE_CMD_DO_SET_SERVO) {
					PX4_INFO("Setting Servo");


					unsigned val1 = ((float)cmd.param1) * 20000,
							 val2 = ((float)cmd.param2) * 20000;

					PX4_INFO("%d %d", val1, val2);

					if(val1 > 20000) val1 = 20000;
					else if(val1 < 90) val1 = 90;

					if(val2 > 20000) val2 = 20000;
					else if(val2 < 90) val2 = 90;

					//bool val3 = ((int)cmd.param3) != 0;

					ret = ioctl(fd, PWM_SERVO_SET(4), val1);
					if (ret != OK) {
						err(1, "PWM_SERVO_SET(%d)", 4);
					}

					ret = ioctl(fd, PWM_SERVO_SET(5), val2);
					if (ret != OK) {
						err(1, "PWM_SERVO_SET(%d)", 5);
					}


					//if()
					//ioctl(priv->gpio_fd, GPIO_SET, priv->pin);

				}
			}

		}
	}
}
