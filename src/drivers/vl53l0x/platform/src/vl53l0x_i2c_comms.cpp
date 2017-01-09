
#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_def.h"
#include "../vl53l0x.h"


int32_t VL53L0X_write_multi(void *arg, uint8_t index, uint8_t  *pdata, int32_t count) {

	VL53L0X *dev = (VL53L0X *) arg;
	if (dev->write_register(index, pdata, count) < 0) {
		return 1;
	}
	return VL53L0X_ERROR_NONE;
}

int32_t VL53L0X_read_multi(void *arg, uint8_t index, uint8_t *pdata, int32_t count) {

	VL53L0X *dev = (VL53L0X *) arg;
	if (dev->read_register(index, pdata, count) < 0) {
		return 1;
	}
	return VL53L0X_ERROR_NONE;
}

int32_t VL53L0X_write_byte(void *arg, uint8_t index, uint8_t data) {
	return VL53L0X_write_multi(arg, index, &data, 1);
}


int32_t VL53L0X_write_word(void *arg, uint8_t index, uint16_t data) {
	uint8_t buff[2];
	buff[1] = data & 0xFF;
	buff[0] = data >> 8;
	return VL53L0X_write_multi(arg, index, buff, 2);
}

int32_t VL53L0X_write_dword(void *arg, uint8_t index, uint32_t data) {
	uint8_t buff[4];

	buff[3] = data & 0xFF;
	buff[2] = data >> 8;
	buff[1] = data >> 16;
	buff[0] = data >> 24;

	return VL53L0X_write_multi(arg, index, buff, 4);
}

int32_t VL53L0X_read_byte(void *arg, uint8_t index, uint8_t  *pdata) {
	return VL53L0X_read_multi(arg, index, data, 1);
}

int32_t VL53L0X_read_word(void *arg, uint8_t index, uint16_t *pdata) {
	uint8_t buff[2];
	int r = VL53L0X_read_multi(arg, index, buff, 2);

	uint16_t tmp;
	tmp = buff[0];
	tmp <<= 8;
	tmp |= buff[1];
	*pdata = tmp;

	return r;
}


int32_t VL53L0X_read_dword(void *arg, uint8_t index, uint32_t *pdata) {
	uint8_t buff[4];
	int r = VL53L0X_read_multi(arg, index, buff, 4);

	uint32_t tmp;
	tmp = buff[0];
	tmp <<= 8;
	tmp |= buff[1];
	tmp <<= 8;
	tmp |= buff[2];
	tmp <<= 8;
	tmp |= buff[3];

	*pdata = tmp;

	return r;
}
