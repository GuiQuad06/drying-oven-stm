/*
 * esp8266.c
 *
 *  Created on: Jun 26, 2024
 *      Author: gucd
 */

#include "esp8266.h"

#include <string.h>

#define MAX_CHAR_CWJAP (100u)

void esp_8266_set_credentials(esp8266_t *esp8266, char *ssid, char *password)
{
    memcpy(esp8266->wifi_credentials.ssid, ssid, strlen(ssid) + 1);
    memcpy(esp8266->wifi_credentials.password, password, strlen(password) + 1);
}

void esp8266_init(esp8266_t *esp8266, fptr_u16_t delay_ms, fptr_pc_u16_t send)
{
    // Callbacks setup
    esp8266->delay_cb = delay_ms;
    esp8266->send_cb  = send;
}

esp8266_status_t esp8266_connect(esp8266_t *esp8266)
{
    char cwjap_cmd[MAX_CHAR_CWJAP];
    uint8_t uart_sts = 0x00;

    memset(cwjap_cmd, 0, MAX_CHAR_CWJAP);

    strcat(cwjap_cmd, "AT+CWJAP=\"");
    strcat(cwjap_cmd, esp8266->wifi_credentials.ssid);
    strcat(cwjap_cmd, "\",\"");
    strcat(cwjap_cmd, esp8266->wifi_credentials.password);
    strcat(cwjap_cmd, "\"\r\n");

    // Init the module
    uart_sts = esp8266->send_cb("AT\r\n", 2000u);
    uart_sts |= esp8266->send_cb("AT+GMR\r\n", 2000u);
    uart_sts |= esp8266->send_cb("AT+CIPSERVER=0\r\n", 2000u);
    uart_sts |= esp8266->send_cb("AT+RESTORE\r\n", 2000u);

    if (0x00 != uart_sts)
    {
        return ESP8266_HAL_ERROR;
    }
    esp8266->delay_cb(5000u);

    // Set mode to station
    uart_sts = esp8266->send_cb("AT+CWMODE=1\r\n", 1500u);
    if (0x00 != uart_sts)
    {
        return ESP8266_HAL_ERROR;
    }
    esp8266->delay_cb(1500u);

    // Connect to wifi
    uart_sts = esp8266->send_cb(cwjap_cmd, 2000u);
    if (0x00 != uart_sts)
    {
        return ESP8266_HAL_ERROR;
    }
    esp8266->delay_cb(5000u);

    // Connect to server
    uart_sts = esp8266->send_cb("AT+CIFSR\r\n", 1500u);
    esp8266->delay_cb(1500u);
    uart_sts |= esp8266->send_cb("AT+CIPMUX=1\r\n", 1500u);
    esp8266->delay_cb(1500u);
    uart_sts |= esp8266->send_cb("AT+CIPSERVER=1,80\r\n", 1500u);
    if (0x00 != uart_sts)
    {
        return ESP8266_HAL_ERROR;
    }
    else
    {
        return ESP8266_OK;
    }
}
