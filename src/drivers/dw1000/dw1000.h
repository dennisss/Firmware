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

/*
The maximum SPI frequency is 20 MHz when the CLKPLL is locked, otherwise the maximum SPI frequency is 3 MHz.
SPICSn select asserted low to valid slave output data

Bring RSTn pin positive to power on the device

*/

#include <drivers/device/spi.h>

/* Register map */
#define DW1000_DEV_ID 0x00 // Device Identifier – includes device type and revision info
#define DW1000_EUI 0x01 // Extended Unique Identifier
#define DW1000_PANADR 0x03 // PAN Identifier and Short Address
#define DW1000_SYS_CFG 0x04 // System Configuration bitmap
#  define DW1000_FFEN  (1 << 0)
#  define DW1000_FFBC  (1 << 1)
#  define DW1000_FFAB  (1 << 2)
#  define DW1000_FFAD  (1 << 3)
#  define DW1000_FFAA  (1 << 4)
#  define DW1000_FFAM  (1 << 5)
#  define DW1000_FFAR  (1 << 6)
#  define DW1000_FFA4  (1 << 7)
#  define DW1000_FFA5  (1 << 8)
#  define DW1000_HIRQ_POL  (1 << 9)
#  define DW1000_SPI_EDGE  (1 << 10)
#  define DW1000_DIS_FCE  (1 << 11)
#  define DW1000_DIS_DRXB  (1 << 12)
#  define DW1000_DIS_PHE  (1 << 13)
#  define DW1000_DIS_RSDE  (1 << 14)
#  define DW1000_FCS_INIT2F  (1 << 0)
#  define DW1000_  (1 << 0)
#  define DW1000_  (1 << 0)
#  define DW1000_  (1 << 0)
#  define DW1000_  (1 << 0)
#  define DW1000_  (1 << 0)
#  define DW1000_  (1 << 0)
#  define DW1000_  (1 << 0)
#  define DW1000_  (1 << 0)

#define DW1000_SYS_TIME 0x06 // System Time Counter (40-bit)
#define DW1000_TX_FCTRL 0x08 // Transmit Frame Control
#define DW1000_TX_BUFFER 0x09 // Transmit Data Buffer
#define DW1000_DX_TIME 0x0A // Delayed Send or Receive Time (40-bit)
#define DW1000_RX_FWTO 0x0C // Receive Frame Wait Timeout Period
#define DW1000_SYS_CTRL 0x0D // System Control Register
#  define DW1000_SFCST  (1 << 0)
#  define DW1000_TXSTRT  (1 << 1)
#  define DW1000_TXDLYS  (1 << 2)
#  define DW1000_CANSFCS  (1 << 3)
#  define DW1000_TRXOFF  (1 << 6)
#  define DW1000_WAIT4RESP  (1 << 7)
#  define DW1000_RXENAB  (1 << 8)
#  define DW1000_RXDLYE  (1 << 9)
#  define DW1000_HRBPT  (1 << 24)
#define DW1000_SYS_MASK 0x0E // System Event Mask Register
#define DW1000_SYS_STATUS 0x0F // System Event Status Register
#define DW1000_RX_FINFO 0x10 // RX Frame Information (in double buffer set)
#define DW1000_RX_BUFFER 0x11 // Receive Data (in double buffer set)
#define DW1000_RX_FQUAL 0x12 // Rx Frame Quality information (in double buffer set)
#define DW1000_RX_TTCKI 0x13 // Receiver Time Tracking Interval (in double buffer set)
#define DW1000_RX_TTCKO 0x14 // Receiver Time Tracking Offset (in double buffer set)
#define DW1000_RX_TIME 0x15 // Receive Message Time of Arrival (in double buffer set)
#define DW1000_TX_TIME 0x17 // Transmit Message Time of Sending
#define DW1000_TX_ANTD 0x18 // 16-bit Delay from Transmit to Antenna
#define DW1000_SYS_STATE 0x19 // System State information
#define DW1000_ACK_RESP_T 0x1A // Acknowledgement Time and Response Time
#define DW1000_RX_SNIFF 0x1D // Pulsed Preamble Reception Configuration
#define DW1000_TX_POWER 0x1E // TX Power Control
#define DW1000_CHAN_CTRL 0x1F // Channel Control
#define DW1000_USR_SFD 0x21 // User-specified short/long TX/RX SFD sequences
#define DW1000_AGC_CTRL 0x23 // Automatic Gain Control configuration
#define DW1000_EXT_SYNC 0x24 // External synchronisation control.
#define DW1000_ACC_MEM 0x25 // Read access to accumulator data
#define DW1000_GPIO_CTRL 0x26 // Peripheral register bus 1 access – GPIO control
#  define DW1000_GPIO_MODE 0x00
#  define DW1000_GPIO_DIR 0x08
#  define DW1000_GPIO_DOUT 0x0C
#  define DW1000_GPIO_IRQE 0x10
#  define DW1000_GPIO_ISEN 0x14
#  define DW1000_GPIO_IMODE 0x18
#  define DW1000_GPIO_IBES 0x1C
#  define DW1000_GPIO_ICLR 0x20
#  define DW1000_GPIO_IDBE 0x24
#  define DW1000_GPIO_RAW 0x28
#define DW1000_DRX_CONF 0x27 // Digital Receiver configuration
#  define DW1000_DRX_TUNE0b 0x02
#  define DW1000_DRX_TUNE1a 0x04
#  define DW1000_DRX_TUNE1b 0x06
#  define DW1000_DRX_TUNE2 0x08
#  define DW1000_DRX_SFDTOC 0x20
#  define DW1000_DRX_PRETOC 0x24
#  define DW1000_DRX_TUNE4H 0x26
#  define DW1000_RXPACC_NOSAT 0x2C
#define DW1000_RF_CONF 0x28 // Analog RF Configuration
#define DW1000_TX_CAL 0x2A // Transmitter calibration block
#define DW1000_FS_CTRL 0x2B // Frequency synthesiser control block
#define DW1000_AON 0x2C // Always-On register set
#define DW1000_OTP_IF 0x2D // One Time Programmable Memory Interface
#define DW1000_LDE_CTRL 0x2E // Leading edge detection control block
#define DW1000_DIG_DIAG 0x2F // Digital Diagnostics Interface
#define DW1000_PMSC 0x36 // Power Management System Control Block

#define DW1000_DEV_ID_RESPONSE 0xDECA0130


// XXX: This can go up to 20Mhz
#define DW1000_SPI_BUS_SPEED 20*1000*1000 /* will be rounded to 10.4 MHz, within margins for MPU9250 */



class DW1000 : public device::SPI
{
public:
	DW1000(int bus, const char *path, spi_dev_e device);
	virtual ~DW1000();

	virtual int	init();

	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);

	int write(unsigned address, unsigned sub, void *data, unsigned count);

//	virtual int	ioctl(unsigned operation, unsigned &arg);



	void setup_leds();

	void reset();

	// TODO: We want to avoid copying, so buffer should be big enough to hold the index as well
	int read_register(uint8_t id, uint16_t index, char *buffer, int length);
	int write_register(uint8_t id, uint16_t index, char *buffer, int length);


protected:
	virtual int probe();


};
