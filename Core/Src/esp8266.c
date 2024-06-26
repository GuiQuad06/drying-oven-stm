/*
 * esp8266.c
 *
 *  Created on: Jun 26, 2024
 *      Author: gucd
 */

#include "esp8266.h"

#include <string.h>

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

void esp8266_connect(esp8266_t *esp8266)
{
    // Connect to wifi
    esp8266->send_cb("AT+RST\r\n", 2000u);
    esp8266->send_cb("AT+CWJAP=\"", 500u);
    esp8266->send_cb(esp8266->wifi_credentials.ssid, 500u);
    esp8266->send_cb("\",\"", 500u);
    esp8266->send_cb(esp8266->wifi_credentials.password, 500u);
    esp8266->send_cb("\"\r\n", 500u);
    esp8266->delay_cb(3000u);

    // Connect to server
    esp8266->send_cb("AT+CIPMODE=1\r\n", 1500u);
    esp8266->delay_cb(1500u);
    esp8266->send_cb("AT+CIFSR\r\n", 1500u);
    esp8266->delay_cb(1500u);
    esp8266->send_cb("AT+CIPMUX=1\r\n", 1500u);
    esp8266->delay_cb(1500u);
    esp8266->send_cb("AT+CIPSERVER=1,80\r\n", 1500u);
}