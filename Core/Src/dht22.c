/*
 * dht22.c
 *
 *  Created on: Jun 20, 2024
 *      Author: gucd
 */
#include "dht22.h"

#define BYTE_LEN (8u)

void dht22_init(dht22_t *device,
                fptr_t input_cfg_cb,
                fptr_b_t output_state_cb,
                fptr_u16_t delay_cb,
                u8_fptr_t input_state_cb)
{
    // Object config
    device->input_cfg    = input_cfg_cb;
    device->output_state = output_state_cb;
    device->delay_sensor = delay_cb;
    device->input_state  = input_state_cb;
}

dht22_status_t dht22_start(dht22_t *device)
{
    dht22_status_t res = DHT22_OK;

    device->output_state(false);
    device->delay_sensor(1200);

    device->output_state(true);
    device->delay_sensor(30);

    device->input_cfg();

    // Check answer
    device->delay_sensor(40);

    if (!device->input_state())
    {
        device->delay_sensor(80);
        if (device->input_state())
        {
            res = DHT22_OK;
        }
        else
        {
            res = DHT22_ERROR;
        }
    }
    // Wait until the pin goes low
    while (device->input_state())
        ;
    return res;
}

uint8_t dht22_read_byte(dht22_t *device)
{
    uint8_t res;

    for (uint8_t i = 0; i < BYTE_LEN; i++)
    {
        // Wait until the pin goes high
        while (!device->input_state())
            ;

        device->delay_sensor(40);

        if (!device->input_state())
        {
            res &= ~(1 << (BYTE_LEN - 1 - i));
        }
        else
        {
            res |= (1 << (BYTE_LEN - 1 - i));
        }
        // Wait until the pin goes low
        while (device->input_state())
            ;
    }
    return res;
}
