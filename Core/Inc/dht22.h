/*
 * dht22.h
 *
 *  Created on: Jun 20, 2024
 *      Author: gucd
 */

#ifndef INC_DHT22_H_
#define INC_DHT22_H_

#include <stdbool.h>
#include <stdint.h>

#define DHT22_FRAME (2u)

typedef void (*fptr_t)(void);
typedef void (*fptr_b_t)(bool);
typedef void (*fptr_u16_t)(uint16_t);
typedef uint8_t (*u8_fptr_t)(void);

typedef enum
{
    DHT22_OK    = 0,
    DHT22_ERROR = 1
} dht22_status_t;

typedef struct
{
    fptr_t input_cfg;
    fptr_b_t output_state;
    fptr_u16_t delay_sensor;
    u8_fptr_t input_state;
    fptr_t output_cfg;

} dht22_t;

void dht22_init(dht22_t *device,
                fptr_t input_cfg_cb,
                fptr_b_t output_state_cb,
                fptr_u16_t delay_cb,
                u8_fptr_t input_state_cb,
                fptr_t output_cfg_cb);
dht22_status_t dht22_start(dht22_t *device);
uint8_t dht22_read_data(dht22_t *device, uint16_t *data, uint16_t size);

#endif /* INC_DHT22_H_ */
