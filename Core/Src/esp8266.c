/*
 * esp8266.c
 *
 *  Created on: Jun 26, 2024
 *      Author: gucd
 */

#include "esp8266.h"

#include <stdio.h>
#include <string.h>

#define MAX_CHAR_CWJAP (100u)
#define CONN_ID_OFFSET (5u)

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

esp8266_status_t http_send_data(esp8266_t *esp8266,
                                char *feedback,
                                uint16_t size_fb,
                                const char *msg,
                                uint16_t size_msg)
{
    char cip_send_cmd[MAX_BUFFER_LEN];
    char cip_close_cmd[MAX_BUFFER_LEN];
    char size_msg_str[4];
    uint8_t uart_sts = 0x00;

    const char *begin = strstr(feedback, "+IPD");
    if (NULL == begin)
    {
        return ESP8266_CONNECTION_ERROR;
    }

    memset(cip_send_cmd, 0, MAX_BUFFER_LEN);

    strcat(cip_send_cmd, "AT+CIPSEND=");
    strncat(cip_send_cmd, begin + CONN_ID_OFFSET, 1);
    strcat(cip_send_cmd, ",");
    snprintf(size_msg_str, 4, "%u", size_msg);
    strcat(cip_send_cmd, size_msg_str);
    strcat(cip_send_cmd, "\r\n");

    uart_sts = esp8266->send_cb(cip_send_cmd, 1000u);
    if (0x00 != uart_sts)
    {
        return ESP8266_HAL_ERROR;
    }

    esp8266->delay_cb(1000u);
    uart_sts = esp8266->send_cb(msg, 1000u);
    if (0x00 != uart_sts)
    {
        return ESP8266_HAL_ERROR;
    }

    esp8266->delay_cb(1000u);
    memset(cip_close_cmd, 0, MAX_BUFFER_LEN);

    strcat(cip_close_cmd, "AT+CIPCLOSE=");
    strncat(cip_close_cmd, begin + 5, 1);
    strcat(cip_close_cmd, "\r\n");

    uart_sts = esp8266->send_cb(cip_close_cmd, 3000u);
    if (0x00 != uart_sts)
    {
        return ESP8266_HAL_ERROR;
    }
    else
    {
        return ESP8266_OK;
    }
}
