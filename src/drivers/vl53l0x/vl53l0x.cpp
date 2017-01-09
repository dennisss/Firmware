/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file vl53l0x.cpp
 *
 * @author Dennis Shtatnov <densht@gmail.com>
 *
 * Driver for the VL53L0X Time of Flight Sensor
 */


#include "vl53l0x.h"



/* Configuration Constants */
#define VL53L0X_BUS 		PX4_I2C_BUS_EXPANSION
#define VL53L0X_BASEADDR 	0x29
#define VL53L0X_DEVICE_PATH	"/dev/vl53l0x"


#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int vl53l0x_main(int argc, char *argv[]);

VL53L0X::VL53L0X(const char * path, int bus, int address) :
	I2C("VL53L0X", path, bus, address, 400000),
	_min_distance(-1.0f),
	_max_distance(-1.0f),
	_conversion_interval(-1),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "vl53l0x_read")),
	_comms_errors(perf_alloc(PC_COUNT, "vl53l0x_com_err")),
	_buffer_overflows(perf_alloc(PC_COUNT, "vl53l0x_buf_of"))

{
	/* enable debug() calls */
	_debug_enabled = false;

	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

VL53L0X::~VL53L0X()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_distance_sensor_topic != nullptr) {
		orb_unadvertise(_distance_sensor_topic);
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
}

int
VL53L0X::init()
{
	int ret = PX4_ERROR;
	int hw_model;
	param_get(param_find("SENS_EN_VL53L0X"), &hw_model);

	switch (hw_model) {
	case 0:
		DEVICE_LOG("disabled.");
		return ret;

	case 1:
		/* (2m 33Hz) */
		_min_distance = 0.05f;
		_max_distance = 2.0f;
		_conversion_interval = 33000;
		break;

	default:
		DEVICE_LOG("invalid HW model %d.", hw_model);
		return ret;
	}

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ret;
	}



	/* Perform device setup */
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	//FixPoint1616_t LimitCheckCurrent;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	uint8_t VhvSettings;
	uint8_t PhaseCal;

	// Initialize Comms
	_device.arg = this;

	status = VL53L0X_DataInit(&_device); // Data initialization

	status = VL53L0X_GetDeviceInfo(&_device, &_deviceInfo);
	if (status != VL53L0X_ERROR_NONE)
		return false;

	if (status == VL53L0X_ERROR_NONE)  {
		/*
		Serial.println(F("VL53L0X_GetDeviceInfo:"));
		Serial.print(F("Device Name : ")); Serial.println(DeviceInfo.Name);
		Serial.print(F("Device Type : ")); Serial.println(DeviceInfo.Type);
		Serial.print(F("Device ID : ")); Serial.println(DeviceInfo.ProductId);
		Serial.print(F("ProductRevisionMajor : ")); Serial.println(DeviceInfo.ProductRevisionMajor);
		Serial.print(F("ProductRevisionMinor : ")); Serial.println(DeviceInfo.ProductRevisionMinor);
		*/
		if ((_deviceInfo.ProductRevisionMinor != 1) && (_deviceInfo.ProductRevisionMinor != 1)) {
			/*
			Serial.print(F("Error expected cut 1.1 but found cut "));
			Serial.print(DeviceInfo.ProductRevisionMajor);
			Serial.print('.');
			Serial.println(DeviceInfo.ProductRevisionMinor);
			*/
			status = VL53L0X_ERROR_NOT_SUPPORTED;
			return false;
		}
	}

	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_StaticInit(&_device);
	}


	// Calibrate
	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_PerformRefSpadManagement(&_device, &refSpadCount, &isApertureSpads);
	}
	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_PerformRefCalibration(&_device, &VhvSettings, &PhaseCal);
	}


	// Setup long range profile
	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_SetLimitCheckValue(&_device, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.1*65536));
	}
	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_SetLimitCheckValue(&_device, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(60*65536));
	}
	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&_device, 33000);
	}
	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_SetVcselPulsePeriod(&_device, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	}
	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_SetVcselPulsePeriod(&_device, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);
	}


	// Set to single ranging mode
	if (status == VL53L0X_ERROR_NONE) {
		status = VL53L0X_SetDeviceMode(&_device, VL53L0X_DEVICEMODE_SINGLE_RANGING);
	}




	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	if (_reports == nullptr) {
		return ret;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	struct distance_sensor_s ds_report = {};

	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
				 &_orb_class_instance, ORB_PRIO_HIGH);

	if (_distance_sensor_topic == nullptr) {
		DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
	}

	/*
	// Select altitude register
	int ret2 = measure();

	if (ret2 == 0) {
		ret = OK;
		_sensor_ok = true;
		DEVICE_LOG("(%dm %dHz) with address %d found", (int)_max_distance,
			   (int)(1e6f / _conversion_interval), SF1XX_BASEADDR);
	}
	*/




	return ret;
}

int
VL53L0X::probe()
{
	return OK;
}

void
VL53L0X::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
VL53L0X::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
VL53L0X::get_minimum_distance()
{
	return _min_distance;
}

float
VL53L0X::get_maximum_distance()
{
	return _max_distance;
}

int
VL53L0X::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_conversion_interval);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();

					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_conversion_interval)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	case RANGEFINDERIOCSETMINIUMDISTANCE: {
			set_minimum_distance(*(float *)arg);
			return 0;
		}
		break;

	case RANGEFINDERIOCSETMAXIUMDISTANCE: {
			set_maximum_distance(*(float *)arg);
			return 0;
		}
		break;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
VL53L0X::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(_conversion_interval);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int
VL53L0X::read_register(unsigned addr, void *data, unsigned count)
{
	uint8_t cmd = addr;
	return transfer(&cmd, 1, (uint8_t *)data, count);
}


int
VL53L0X::write_register(unsigned addr, void *data, unsigned count)
{
	uint8_t cmd[64];

	if (sizeof(cmd) < (count + 1)) {
		return -EIO;
	}

	cmd[0] = addr;
	cmd[1] = *(uint8_t *)data;
	return transfer(&cmd[0], count + 1, nullptr, 0);
}

// This triggers the start of a measurement
int
VL53L0X::measure()
{
	if(VL53L0X_StartMeasurement(&_device) != VL53L0X_ERROR_NONE) {
		return -EIO;
	}

	return OK;
}

// This reads the measurement
int
VL53L0X::collect()
{
	int	ret = -EIO;
	VL53L0X_Error status = VL53L0X_ERROR_NONE;

	VL53L0X_RangingMeasurementData_t val;
	perf_begin(_sample_perf);

	/* check if data was measured */
	// TODO

	status = VL53L0X_GetRangingMeasurementData(&_device, &val);

	if (status != VL53L0X_ERROR_NONE) {
		DEVICE_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	float distance_m = float(val.RangeMilliMeter) * 1e-3f;

	struct distance_sensor_s report;
	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	report.orientation = 8;
	report.current_distance = distance_m;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.covariance = 0.0f;
	/* TODO: set proper ID */
	report.id = 0;

	/* publish it, if we are the primary */
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}

	if (_reports->force(&report)) {
		perf_count(_buffer_overflows);
	}

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
VL53L0X::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	/* set register to '0' */
	measure();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&VL53L0X::cycle_trampoline, this, USEC2TICK(_conversion_interval));
}

void
VL53L0X::stop()
{
	work_cancel(HPWORK, &_work);
}

void
VL53L0X::cycle_trampoline(void *arg)
{
	VL53L0X *dev = (VL53L0X *)arg;

	dev->cycle();
}

void
VL53L0X::cycle()
{

	/* Collect results */
	if (OK != collect()) {
		DEVICE_DEBUG("collection error");
		/* if error restart the measurement state machine */
		start();
		return;
	}

	// TODO: We should perform a measurement right here
	measure();

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&VL53L0X::cycle_trampoline,
		   this,
		   USEC2TICK(_conversion_interval));

}

void
VL53L0X::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace vl53l0x
{

VL53L0X	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd = -1;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/* create the driver */
	g_dev = new VL53L0X(VL53L0X_DEVICE_PATH, VL53L0X_BUS, VL53L0X_BASEADDR);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(VL53L0X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		::close(fd);
		goto fail;
	}

	::close(fd);
	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct distance_sensor_s report;
	ssize_t sz;
	int ret;

	int fd = open(VL53L0X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'vl53l0x start' if the driver is not running", VL53L0X_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %0.2f m", (double)report.current_distance);
	warnx("time:        %llu", report.timestamp);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("valid %u", (float)report.current_distance > report.min_distance
		      && (float)report.current_distance < report.max_distance ? 1 : 0);
		warnx("measurement: %0.3f", (double)report.current_distance);
		warnx("time:        %llu", report.timestamp);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	::close(fd);
	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(VL53L0X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	::close(fd);
	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	exit(0);
}

} /* namespace */

int
vl53l0x_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		vl53l0x::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		vl53l0x::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		vl53l0x::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		vl53l0x::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		vl53l0x::info();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
