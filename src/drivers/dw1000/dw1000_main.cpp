/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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


#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/spi.h>
#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>

#include "dw1000.h"


#define DW_DEVICE_PATH		"/dev/dw1000"
#define DW_DEVICE_PATH_EXT		"/dev/dw1000_ext"


/** driver 'main' command */
extern "C" { __EXPORT int dw1000_main(int argc, char *argv[]); }

/**
 * Local functions in support of the shell command.
 */
namespace dw1000
{

DW1000	*g_dev_int; // on internal bus
DW1000	*g_dev_ext; // on external bus

void	start(bool);
void	stop(bool);
void	test(bool);
void	reset(bool);
void	info(bool);
void	regdump(bool);
void	usage();

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(bool external_bus)
{
	//int fd;
	DW1000 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;
	const char *path = external_bus ? DW_DEVICE_PATH_EXT : DW_DEVICE_PATH;

	if (*g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
	{
		errx(0, "already started");
	}

	/* create the driver */
	if (external_bus) {
#ifdef PX4_SPI_BUS_EXT
		*g_dev_ptr = new DW1000(PX4_SPI_BUS_EXT, path, (spi_dev_e)PX4_SPIDEV_DW); //PX4_SPIDEV_EXT_DW);
#else
		errx(0, "External SPI not available");
#endif

	} else {
		// Internal
	}

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}

	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	/*
	fd = open(path_accel, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	close(fd);
	*/


	exit(0);
fail: // TODO: Currently this isn't used

	if (*g_dev_ptr != nullptr) {
		delete(*g_dev_ptr);
		*g_dev_ptr = nullptr;
	}

	errx(1, "driver start failed");
}

void
stop(bool external_bus)
{
	DW1000 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr != nullptr) {
		delete *g_dev_ptr;
		*g_dev_ptr = nullptr;

	} else {
		/* warn, but not an error */
		warnx("already stopped.");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test(bool external_bus)
{
	const char *path = external_bus ? DW_DEVICE_PATH_EXT : DW_DEVICE_PATH;

	//ssize_t sz;

	/* get the driver */
	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'd start')", path);
	}



	/* reset to manual polling */
	//if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_MANUAL) < 0) {
	//	err(1, "reset to manual polling");
	//}

	/* do a simple demand read */
	//sz = read(fd, &a_report, sizeof(a_report));

	//if (sz != sizeof(a_report)) {
	//	warnx("ret: %d, expected: %d", sz, sizeof(a_report));
	//	err(1, "immediate acc read failed");
	//}

	// warnx("single read");


	close(fd);

	/* XXX add poll-rate tests here too */

	reset(external_bus);
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset(bool external_bus)
{
	const char *path = external_bus ? DW_DEVICE_PATH_EXT : DW_DEVICE_PATH_EXT;
	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}
/*
	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}
*/
	close(fd);

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info(bool external_bus)
{
	DW1000 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

	//(*g_dev_ptr)->print_info();

	exit(0);
}

/**
 * Dump the register information
 */
void
regdump(bool external_bus)
{
	DW1000 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		errx(1, "driver not running");
	}

	//(*g_dev_ptr)->print_registers();

	exit(0);
}

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'stop',\n'reset', 'regdump'");
	warnx("options:");
	warnx("    -X    (external bus)");
}

} // namespace

int
dw1000_main(int argc, char *argv[])
{
	bool external_bus = false;
	int ch;

	/* jump over start/off/etc and look at options first */
	while ((ch = getopt(argc, argv, "X")) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		default:
			dw1000::usage();
			exit(0);
		}
	}

	const char *verb = argv[optind];

	/*
	* Start/load the driver.
	*/
	if (!strcmp(verb, "start")) {
		dw1000::start(external_bus);
	}

	if (!strcmp(verb, "stop")) {
		dw1000::stop(external_bus);
	}

	/*
	* Test the driver/device.
	*/
	if (!strcmp(verb, "test")) {
		dw1000::test(external_bus);
	}

	/*
	* Reset the driver.
	*/
	if (!strcmp(verb, "reset")) {
		dw1000::reset(external_bus);
	}

	/*
	* Print driver information.
	*/
	if (!strcmp(verb, "info")) {
		dw1000::info(external_bus);
	}

	/*
	* Print register information.
	*/
	if (!strcmp(verb, "regdump")) {
		dw1000::regdump(external_bus);
	}

	dw1000::usage();
	exit(1);
}
