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


#include "dw1000.h"


/*
	TODO: Support both TOA and TDOA

	For TOA, LPE needs
	- Beacon location, and distance to beacon

	For TDOA, we need to give LPE messages like:
	- Beacon location, receive time

*/

// Mode defined by configuration of GPIO5 and GPIO6 on the board. (not connected/low defaults to MODE0)
DW1000::DW1000(int bus, const char *path, spi_dev_e device) :
	SPI("DW1000", path, bus, device, SPIDEV_MODE0, DW1000_SPI_BUS_SPEED)
{
	//_device_id.devid_s.devtype =  DRV_ACC_DEVTYPE_MPU9250;
}

DW1000::~DW1000()
{

}

int
DW1000::init()
{

	// TODO: Need to setup reset pin

	int ret;

	ret = SPI::init();

	if (ret != OK) {
		DEVICE_DEBUG("SPI init failed");
		return -EIO;
	}

	ret = probe();

	if (ret != OK) {
		DEVICE_DEBUG("DW1000 probe failed");
		return ret;
	}


	uint64_t t = 0;
	read_register(DW1000_SYS_TIME, 0, &t, 5);
	PX4_INFO("%llu", t);
	usleep(10);

	read_register(DW1000_SYS_TIME, 0, &t, 5);
	PX4_INFO("%llu", t);


	PX4_INFO("hello");





	//Hard setting GPIO0 to high

	uint8_t val = 0;
	write(DW1000_GPIO_CTRL, 0, &val, 1);

	read(DW1000_GPIO_CTRL, &val, 1);
	PX4_INFO("%02X", val);

	val = 1 << 4; // mask
	write(DW1000_GPIO_CTRL, 0x08, &val, 1);


	val = 0xFF; //1 | (1 << 1);
	write(DW1000_GPIO_CTRL, 0x0C, &val, 1);


	setup_leds();


	// trigger a test receive
	uint32_t ctrl = 2; //1 << 8;
	write(DW1000_SYS_CTRL, &ctrl, 4);

	return OK;
}

void
DW1000::reset()
{


}



#define DIR_READ 0
#define DIR_WRITE (1 << 7)
#define SUB_INDEX (1 << 6)
#define EXT_INDEX (1 << 7)

/*
	Inserts the SPI header at the beginning of the given buffer
	returns the size in bytes of the header
*/
inline unsigned
dw1000_spi_header(unsigned dir, unsigned reg, unsigned sub, void *data)
{
	reg = reg & 0x3F;
	sub = sub & 0x7FFF;

	unsigned n = 1;
	if (sub > 0) {
		n++;

		if (sub > 0x7F) {
			n++;
			cmd[2] = sub >> 7;
		}

		cmd[1] = (n > 2? EXT_INDEX : 0) | (sub & 0x7F);
	}

	cmd[0] = (writing? DIR_WRITE : DIR_READ) | (n > 1? SUB_INDEX : 0) | reg;

	return n;
}


int DW1000::write_register(uint8_t reg, uint16_t sub, const char *buffer, int length)
{
	uint8_t cmd[128];//MPU_MAX_WRITE_BUFFER_SIZE];

	if (sizeof(cmd) < (count + 2)) {
		return -EIO;
	}

	int n = dw1000_spi_header(DIR_WRITE, reg, sub, cmd);
	memcpy(&cmd[n], data, count);

	set_frequency(20*1000*1000);

	return transfer(cmd, cmd, count + n);
}


int
DW1000::read_register(uint8_t reg, uint16_t sub, char *buffer, int length)
{
	uint8_t cmd[128];

	int n = dw1000_spi_header(DIR_READ, reg, sub, cmd);

	set_frequency(20*1000*1000);

	int ret = transfer(cmd, cmd, count + n);

	if (ret == OK) {
		memcpy(data, &cmd[n], count);
	}

	return ret;
}

int
DW1000::probe()
{
	uint32_t whoami = 0;
	uint32_t expected = DW1000_DEV_ID_RESPONSE;
	int r = (read_register(DW1000_DEV_ID, 0, &whoami, 4) == OK && (whoami == expected)) ? 0 : -EIO;

	PX4_INFO("%08X", whoami);

	return r;
}

void
DW1000::transmit(const char *buf, unsigned len, bool delay = false)
{
	uint32_t fc = 0;
	write_register(DW1000_TX_FCTRL, 0, &fc, 4);

	write_register(DW_1000_TX_BUFFER, 0, buf, len);

	uint32_t ctrl = DW1000_TXSTRT | (delay? DW1000_TXDLYS : 0);
	write_register(DW1000_SYS_CTRL, 0, &ctrl, 4);
}

void
DW1000::set_addr(uint16_t pan, uint16_t addr)
{
	uint32_t val;
	memcpy(&val, addr, 2);
	memcpy(((char *)&val) + 2. pan, 2);

	write_register(DW1000_PANADR, 0, &val, 4);
}

dw1000_time_t
DW1000::get_system_time()
{
	dw1000_time_t t;
	read_register(DW1000_SYS_TIME, 0, &t, 5);
	return t;
}

void
DW1000::set_delayed_time(dw1000_time_t t)
{
	write_register(DW1000_DX_TIME, 0, &t, 5);
}

dw1000_time_t
DW1000::get_receive_time()
{
	dw1000_time_t t;
	read_register(DW1000_RX_TIME, 0, &t, 5);
	return t;
}

dw1000_time_t
DW1000::get_transmit_time()
{
	dw1000_time_t t;
	read_register(DW1000_TX_TIME, 0, &t, 5);
	return t;
}

void
DW1000::setup_leds()
{
	/* Configure GPIO0-3 to act as alternate I/O indicator LEDs */
	uint32_t val = (1 << 6) | (1 << 8) | (1 << 10) | (1 << 12);
	// Sub-register GPIO_MODE 0
	write(DW1000_GPIO_CTRL, 0, &val, 4);
}



/*
In order to transmit, the host controller must write data for transmission to Register file: 0x09 – Transmit Data Buffer. The desired selections for preamble length, data rate and PRF must also be written to Register file: 0x08 – Transmit Frame Control. Transmitter configuration is carried out in the IDLE state, but frame configurations may be carried out during active transmit as described in section 3.5 – High Speed Transmission. Assuming all other relevant configurations have already been made, the host controller initiates the transmission by setting the TXSTRT control bit in Register file: 0x0D – System Control Register. After transmission has been requested, the DW1000 automatically sends the complete frame; preamble, SFD, PHR and data. The FCS (CRC) is automatically appended to the message as an aid to the MAC layer framing.

*/
