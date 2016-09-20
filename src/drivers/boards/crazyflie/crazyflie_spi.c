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

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>
#include <unistd.h>

#include <nuttx/spi.h>
#include <arch/board/board.h>

#include <up_arch.h>
#include <chip.h>
#include <stm32.h>
#include "board_config.h"
#include <systemlib/err.h>

__EXPORT void stm32_spiinitialize(void)
{
#ifdef CONFIG_STM32_SPI1
	px4_arch_configgpio(GPIO_SPI_CS_DW1000);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	px4_arch_gpiowrite(GPIO_SPI_CS_DW1000, 1);

	px4_arch_configgpio(GPIO_DRDY_DW1000);
#endif

}

__EXPORT void stm32_spi1select(FAR struct spi_dev_s *dev, enum spi_dev_e devid, bool selected)
{
	/* SPI select is active low, so write !selected to select the device */

	switch (devid) {
	case PX4_SPIDEV_DW:
		px4_arch_gpiowrite(GPIO_SPI_CS_DW1000, !selected);
		break;

	default:
		break;
	}
}

__EXPORT uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, enum spi_dev_e devid)
{
	return SPI_STATUS_PRESENT;
}

__EXPORT void board_spi_reset(int ms)
{
	/* disable SPI bus */
	px4_arch_configgpio(GPIO_SPI_CS_OFF_DW1000);

	px4_arch_gpiowrite(GPIO_SPI_CS_OFF_DW1000, 0);

	px4_arch_configgpio(GPIO_SPI1_SCK_OFF);
	px4_arch_configgpio(GPIO_SPI1_MISO_OFF);
	px4_arch_configgpio(GPIO_SPI1_MOSI_OFF);

	px4_arch_gpiowrite(GPIO_SPI1_SCK_OFF, 0);
	px4_arch_gpiowrite(GPIO_SPI1_MISO_OFF, 0);
	px4_arch_gpiowrite(GPIO_SPI1_MOSI_OFF, 0);

	px4_arch_configgpio(GPIO_DRDY_OFF_DW1000);

	px4_arch_gpiowrite(GPIO_DRDY_OFF_DW1000, 0);


	/* wait a bit before starting SPI, different times didn't influence results */
	usleep(100);

	/* reconfigure the SPI pins */
#ifdef CONFIG_STM32_SPI1
	px4_arch_configgpio(GPIO_SPI_CS_DW1000);

	/* De-activate all peripherals,
	 * required for some peripheral
	 * state machines
	 */
	px4_arch_gpiowrite(GPIO_SPI_CS_DW1000, 1);

	px4_arch_configgpio(GPIO_SPI1_SCK);
	px4_arch_configgpio(GPIO_SPI1_MISO);
	px4_arch_configgpio(GPIO_SPI1_MOSI);

	// // XXX bring up the EXTI pins again
	// px4_arch_configgpio(GPIO_GYRO_DRDY);
	// px4_arch_configgpio(GPIO_MAG_DRDY);
	// px4_arch_configgpio(GPIO_ACCEL_DRDY);
	// px4_arch_configgpio(GPIO_EXTI_MPU_DRDY);

#endif

}
