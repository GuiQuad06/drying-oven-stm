/*
 * sensor_callbacks.c
 *
 *  Created on: Jun 21, 2024
 *      Author: gucd
 */
#include "sensor_callbacks.h"

#include "main.h"

#include <string.h>

#define CHARGE_TIME_DELAY_US     (10000u)
#define CONVERSION_TIME_DELAY_US (65200u)

/**
 * @brief      Delay function in ms
 * @param[in]  time : time in ms
 */
void delay_ms(uint16_t time)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    while (__HAL_TIM_GET_COUNTER(&htim3) < time)
        ;
}

/**
 * @brief      Delay function in us
 * @param[in]  time : time in us
 */
void delay_us(uint16_t time)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    while (__HAL_TIM_GET_COUNTER(&htim4) < time)
        ;
}

/**
 * @brief      Callback for chipselect pin state handle
 * @param[in]  enable : expected slave state
 * @return     None
 */
void chipselect_cb(bool enable)
{
    if (enable)
    { // device selected
        // pull chip select pin low
        GPIOB->BSRR = GPIO_PIN_12 << 16;
    }
    else
    { // device not selected
        // pull chip select pin high
        GPIOB->BSRR = GPIO_PIN_12;
    }
}

/**
 * @brief      Callback for transcieving SPI data on full duplex
 * @param[in]  data : the data to be send via SPI
 * @return     uint8_t : the returned response to the sent data
 */
uint8_t spi_trx_cb(uint8_t data)
{
    uint8_t rx_data[1];

    if (HAL_SPI_TransmitReceive(&hspi2, &data, &rx_data[0], 1, 10) != HAL_OK)
    {
        return 0;
    }
    return *rx_data;
}

/**
 * @brief      Delay callback for charge time
 */
void charge_time_delay_cb(void)
{
    delay_us(CHARGE_TIME_DELAY_US);
}

/**
 * @brief      Delay callback for conversion time
 */
void conversion_time_delay_cb(void)
{
    delay_us(CONVERSION_TIME_DELAY_US);
}

/**
 * @brief      Callback for threshold fault handling
 */
void threshold_fault(void)
{
    // TODO handle threshold fault
}

/**
 * @brief      Set GPIO pin to input mode for DHT22
 */
void gpio_input_dir(void)
{
    // MODE: 00 (input)
    GPIOA->CRL &= ~(GPIO_CRL_MODE6);

    // CFG: 01 (floating)
    GPIOA->CRL |= GPIO_CRL_CNF6_0;
}

/**
 * @brief      Set GPIO pin to output mode for DHT22
 */
void gpio_output_dir(void)
{
    // MODE: 01 (output)
    GPIOA->CRL |= GPIO_CRL_MODE6_0;

    // CFG: 00 (push-pull)
    GPIOA->CRL &= ~(GPIO_CRL_CNF6);
}

/**
 * @brief      Write (set or reset) GPIO pin
 * @param[in]  state : the state to be set
 */
void gpio_write(bool state)
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
 * @brief      Read GPIO pin
 * @return     uint8_t : the state of the pin
 */
uint8_t gpio_read(void)
{
    return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6);
}

/**
 * @brief      Send a message over UART interface
 * @param[in]  msg : the message to be sent
 * @param[in]  timeout : the timeout for the message to be sent
 * @return     uint8_t : the status of the message sending
 */
uint8_t send_message(const char *msg, uint16_t timeout)
{
    return HAL_UART_Transmit(&huart1, (uint8_t *) msg, strlen(msg), timeout) == HAL_OK ? 0 : 1;
}
