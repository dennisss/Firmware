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
#include <px4_defines.h>

#include <termios.h>
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

// PE1 is TX/Out, PE0 is RX/In
#define GPIO_ACTIVE_IR (GPIO_OUTPUT|GPIO_SPEED_2MHz|GPIO_PORTE|GPIO_PIN0)

// Pixie on GPS port
#define PIXIE_DEVICE_PATH "/dev/ttyS3"

// We use FrSky UART on Pixracer for the Pixie
//"/dev/ttyS6"
#define PIXIE_CHAIN_LEN 2

#define VEHICLE_CMD_USER_1 31010
#define VEHICLE_CMD_BEACON VEHICLE_CMD_USER_1

#define VEHICLE_CMD_USER_2 31011
#define VEHICLE_CMD_RGBLED VEHICLE_CMD_USER_2

static int _iolink_task;


__EXPORT int iolink_main(int argc, char *argv[]);

int iolink_thread_main(int argc, char *argv[]);

void pixie_write(int fd, int rgb);

int
open_serial(const char *dev);


void pixie_write(int fd, int rgb) {

	char r = rgb;
	char g = rgb >> 8;
	char b = rgb >> 16;

	char buf[PIXIE_CHAIN_LEN*3];
	for(int i = 0; i < PIXIE_CHAIN_LEN; i++) {
		buf[i*3] = r;
		buf[i*3 + 1] = g;
		buf[i*3 + 2] = b;
	}

	write(fd, buf, sizeof(buf));

}


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



int
open_serial(const char *dev)
{

	int rate = B115200;

	// open uart
	int fd = px4_open(dev, O_RDWR | O_NOCTTY);
	int termios_state = -1;

	if (fd < 0) {
		PX4_ERR("failed to open uart device!");
		return -1;
	}

	// set baud rate
	struct termios config;
	tcgetattr(fd, &config);

	// clear ONLCR flag (which appends a CR for every LF)
	config.c_oflag &= ~ONLCR;

	// Disable hardware flow control
	config.c_cflag &= ~CRTSCTS;


	/* Set baud rate */
	if (cfsetispeed(&config, rate) < 0 || cfsetospeed(&config, rate) < 0) {
		warnx("ERR SET BAUD %s: %d\n", dev, termios_state);
		px4_close(fd);
		return -1;
	}

	if ((termios_state = tcsetattr(fd, TCSANOW, &config)) < 0) {
		PX4_WARN("ERR SET CONF %s\n", dev);
		px4_close(fd);
		return -1;
	}

	return fd;
}



int iolink_thread_main(int argc, char *argv[]) {
	int ret;
	const char *dev = PWM_OUTPUT0_DEVICE_PATH;


	/* Open PWM driver */
	int pwm_fd = open(dev, 0);
	if (pwm_fd < 0) {
		err(1, "can't open %s", dev);
	}


	int pixie_fd = open_serial(PIXIE_DEVICE_PATH);
	int pixie_color = 0; // Default color is off
	if (pixie_fd < 0) {
		err(1, "can't open %s", PIXIE_DEVICE_PATH);
	}

	/* Configure gpio for active IR */
	px4_arch_configgpio(GPIO_ACTIVE_IR);
	px4_arch_gpiowrite(GPIO_ACTIVE_IR, true);


	/* Set PWM range to 0% to 100% duty cycle for 50Hz */
	struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));
	pwm_values.channel_count = 6; // servo_count;
	pwm_values.values[4] = 2;
	pwm_values.values[5] = 2;
	ret = ioctl(pwm_fd, PWM_SERVO_SET_MIN_PWM, (long unsigned int)&pwm_values);
	if (ret != OK) {
		errx(ret, "failed setting min values");
	}

	pwm_values.values[4] = 2500;
	pwm_values.values[5] = 2500;
	ret = ioctl(pwm_fd, PWM_SERVO_SET_MAX_PWM, (long unsigned int)&pwm_values);
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
		int poll_ret = px4_poll(fds, 1, 500);

		/* handle the poll result */
		if (poll_ret == 0) {
			/* this means none of our providers is giving us data */
			//PX4_ERR("[px4_simple_app] Got no data within a second");

			// Ensure that the LEDs don't time out
			pixie_write(pixie_fd, pixie_color);

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


					unsigned val1 = ((float)cmd.param1) * 2500,
							 val2 = ((float)cmd.param2) * 2500;

					PX4_INFO("%d %d", val1, val2);

					if(val1 > 2500) val1 = 2500;
					//else if(val1 < 2) val1 = 2;

					if(val2 > 2500) val2 = 2500;
					//else if(val2 < 2) val2 = 2;

					//bool val3 = ((int)cmd.param3) != 0;

					ret = ioctl(pwm_fd, PWM_SERVO_SET(4), val1);
					if (ret != OK) {
						err(1, "PWM_SERVO_SET(%d)", 4);
					}

					ret = ioctl(pwm_fd, PWM_SERVO_SET(5), val2);
					if (ret != OK) {
						err(1, "PWM_SERVO_SET(%d)", 5);
					}


					//if()
					//ioctl(priv->gpio_fd, GPIO_SET, priv->pin);

				} else if (cmd.command == VEHICLE_CMD_BEACON) {
					bool on = (int)cmd.param1? false : true;
					px4_arch_gpiowrite(GPIO_ACTIVE_IR, on);

				} else if (cmd.command == VEHICLE_CMD_RGBLED) {
					pixie_color = (int) cmd.param1;
					pixie_write(pixie_fd, pixie_color);
				}

			}

		}
	}
}
