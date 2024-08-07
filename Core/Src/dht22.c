/*
 * dht22.c
 *
 *  Created on: Jun 20, 2024
 *      Author: gucd
 */
#include "dht22.h"

#define BYTE_LEN (8u)
#define WORD_LEN (16u)

static uint16_t dht22_read_word(dht22_t *device);
static uint8_t dht22_read_byte(dht22_t *device);

void dht22_init(dht22_t *device,
                fptr_t input_cfg_cb,
                fptr_b_t output_state_cb,
                fptr_u16_t delay_cb,
                u8_fptr_t input_state_cb,
                fptr_t output_cfg_cb)
{
    // Object config
    device->input_cfg    = input_cfg_cb;
    device->output_state = output_state_cb;
    device->delay_sensor = delay_cb;
    device->input_state  = input_state_cb;
    device->output_cfg   = output_cfg_cb;
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

uint8_t dht22_read_data(dht22_t *device, uint16_t *data, uint16_t size)
{
    uint8_t checksum = 0x00;
    uint16_t acc     = 0x0000;

    for (uint8_t i = 0; i < size; i++)
    {
        data[i] = dht22_read_word(device);
    }
    // Read the Checksum
    checksum = dht22_read_byte(device);

    // Verify the checksum
    acc = (data[0] >> 8) + (data[0] & 0xFF) + (data[1] >> 8) + (data[1] & 0xFF);
    if (checksum == (acc & 0xFF))
    {
        checksum = 0x00;
    }

    // Reset the pin as output for the next MCU request
    device->output_cfg();

    return checksum;
}

// PRIVATE FUNCTIONS

static uint16_t dht22_read_word(dht22_t *device)
{
    uint16_t res = 0x0000;

    for (uint16_t i = 0; i < WORD_LEN; i++)
    {
        // Wait until the pin goes high
        while (!device->input_state())
            ;

        device->delay_sensor(40);

        if (!device->input_state())
        {
            res &= ~(1 << (WORD_LEN - 1 - i));
        }
        else
        {
            res |= (1 << (WORD_LEN - 1 - i));
        }
        // Wait until the pin goes low
        while (device->input_state())
            ;
    }
    return res;
}

static uint8_t dht22_read_byte(dht22_t *device)
{
    uint8_t res = 0x00;

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
