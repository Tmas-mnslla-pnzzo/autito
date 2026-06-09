/*
 * soft_i2c.h
 *
 *  Created on: 3 jun 2026
 *      Author: tomi
 */

#ifndef INC_SOFT_I2C_H_
#define INC_SOFT_I2C_H_

#include <stdint.h>

uint8_t soft_i2c_write(uint8_t addr, const uint8_t *data, uint8_t len);

#endif /* INC_SOFT_I2C_H_ */
