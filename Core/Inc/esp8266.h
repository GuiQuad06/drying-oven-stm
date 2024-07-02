/*
 * esp8266.h
 *
 *  Created on: Jun 26, 2024
 *      Author: gucd
 */

#ifndef INC_ESP8266_H_
#define INC_ESP8266_H_

#include <stdint.h>

#define MAX_CHAR_SSID  (50u)
#define MAX_CHAR_PWD   (50u)
#define MAX_BUFFER_LEN (50u)

typedef uint8_t (*fptr_pc_u16_t)(const char *, uint16_t);
typedef void (*fptr_u16_t)(uint16_t);

typedef enum
{
    ESP8266_OK               = 0x00,
    ESP8266_ERROR            = 0x01,
    ESP8266_HAL_ERROR        = 0x02,
    ESP8266_CONNECTION_ERROR = 0x03
} esp8266_status_t;

typedef struct
{
    char ssid[MAX_CHAR_SSID];
    char password[MAX_CHAR_PWD];
} wifi_credentials_t;

typedef struct
{
    wifi_credentials_t wifi_credentials;
    fptr_u16_t delay_cb;
    fptr_pc_u16_t send_cb;
} esp8266_t;

void esp_8266_set_credentials(esp8266_t *esp8266, char *ssid, char *password);
void esp8266_init(esp8266_t *esp8266, fptr_u16_t delay_ms, fptr_pc_u16_t send);
esp8266_status_t esp8266_connect(esp8266_t *esp8266);
esp8266_status_t http_send_data(esp8266_t *esp8266,
                                char *feedback,
                                uint16_t size_fb,
                                const char *msg,
                                uint16_t size_msg);

#endif /* INC_ESP8266_H_ */
