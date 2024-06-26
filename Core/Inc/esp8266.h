/*
 * esp8266.h
 *
 *  Created on: Jun 26, 2024
 *      Author: gucd
 */

#ifndef INC_ESP8266_H_
#define INC_ESP8266_H_

#include <stdint.h>

typedef void (*fptr_pc_u16_t)(char *, uint16_t);
typedef void (*fptr_u16_t)(uint16_t);

typedef struct
{
    char *ssid;
    char *password;
} wifi_credentials_t;

typedef struct
{
    wifi_credentials_t wifi_credentials;
    fptr_u16_t delay_cb;
    fptr_pc_u16_t send_cb;
} esp8266_t;

void esp_8266_set_credentials(esp8266_t *esp8266, char *ssid, char *password);
void esp8266_init(esp8266_t *esp8266, fptr_u16_t delay_ms, fptr_pc_u16_t send);
void esp8266_connect(esp8266_t *esp8266);

#endif /* INC_ESP8266_H_ */
