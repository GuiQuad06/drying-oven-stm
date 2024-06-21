/*
 * sensor_callbacks.h
 *
 *  Created on: Jun 21, 2024
 *      Author: gucd
 */

#ifndef INC_SENSOR_CALLBACKS_H_
#define INC_SENSOR_CALLBACKS_H_

#include <stdbool.h>
#include <stdint.h>

void delay(uint16_t time);
void chipselect_cb(bool enable);
uint8_t spi_trx_cb(uint8_t data);
void charge_time_delay_cb(void);
void conversion_time_delay_cb(void);
void threshold_fault(void);
void gpio_input_dir(void);
void gpio_write(bool state);
uint8_t gpio_read(void);

#endif /* INC_SENSOR_CALLBACKS_H_ */