/*
 * cli.c
 *
 *  Created on: Jun 14, 2024
 *      Author: gucd
 */

#include "cli.h"

#include "main.h"

#include <stdarg.h>
#include <string.h>

#define MAX_ARGC (2u)

extern int __io_putchar(int ch) __attribute__((weak));

typedef struct
{
    const char *title;
    cmd_callback_t callback;
    const char *desc;
} cli_menu_t;

cli_menu_t cli_menu[] = {{"pt100", cmd_read_pt100, "Read the PT100 temperature"},
                         {"led", cmd_led, "Set or reset the LED (on/off)"},
                         {"dht22", cmd_dht22, "Read the DHT22 temperature and humidity"},
                         {0, 0, 0}};

str_status_t str_status[] = {{STATUS_OK, "Tout va bien"},
                             {STATUS_KO, "Oups, erreur..."},
                             {STATUS_TOO_MANY_ARGS, "Trop de parametres..."},
                             {STATUS_PARAMETER_MISSING, "Parametre manquant!"}};

void print_cli_menu(void)
{
    cli_menu_t *ptr;
    ptr       = cli_menu;
    uint8_t n = 1;

    while (ptr->title != 0)
    {
        PRINTF("%d) %s : %s\n", n, ptr->title, ptr->desc);

        ptr++;
        n++;
    }
    PRINTF("\n");
}

cli_status_t cli_input(char *cli)
{
    uint8_t argc = 0;
    char *ptr, *end, *argv[MAX_ARGC + 1];
    cli_menu_t *ptr_cli_menu;

    ptr = cli;

    while ((end = strchr(ptr, ' ')))
    {
        // When found, replace it by \0
        *end = 0;

        argv[argc] = ptr;

        argc++;

        // Start after the space for the next search
        ptr = end + 1;

        if (MAX_ARGC < argc)
        {
            return STATUS_TOO_MANY_ARGS;
        }
    }

    // Command without parameters case
    if (!argc)
    {
        argv[0] = ptr;
        argc    = 1;
    }
    // Command with parameters case
    else
    {
        argv[argc] = ptr;
    }

    ptr_cli_menu = &cli_menu[0];

    // Lookup for the Command callback to call :
    while (ptr_cli_menu->callback)
    {
        if (!strncmp(ptr_cli_menu->title, argv[0], strlen(ptr_cli_menu->title)))
        {
            return (ptr_cli_menu->callback(argc, argv));
        }
        ptr_cli_menu++;
    }

    return STATUS_KO;
}

void print_feedback(cli_status_t sts)
{
    PRINTF("%s\n", str_status[sts].str);
}

cli_status_t cmd_read_pt100(int argc, char **argv)
{
    cli_status_t status = STATUS_OK;

    PRINTF("PT100 temperature is:%u\n", (uint16_t) max31865_readCelsius(&pt100_TempSensor));

    return status;
}

cli_status_t cmd_led(int argc, char **argv)
{
    if (argc < 1)
    {
        return STATUS_PARAMETER_MISSING;
    }
    else
    {
        if (strcmp(argv[1], "on") && strcmp(argv[1], "off"))
        {
            return STATUS_PARAMETER_UNKNOWN;
        }
        else if (!strcmp(argv[1], "off"))
        {
            // Light off
            GPIOA->BSRR = GPIO_PIN_5 << 16;
        }
        else if (!strcmp(argv[1], "on"))
        {
            // Light on
            GPIOA->BSRR = GPIO_PIN_5;
        }
    }

    return STATUS_OK;
}

cli_status_t cmd_dht22(int argc, char **argv)
{
    cli_status_t status        = STATUS_OK;
    uint16_t data[DHT22_FRAME] = {0};
    uint8_t cs                 = 0;
    dht22_status_t sts         = DHT22_OK;

    sts = dht22_start(&dht22);
    if (DHT22_ERROR == sts)
    {
        return STATUS_KO;
    }
    else
    {
        cs = dht22_read_data(&dht22, data, DHT22_FRAME);

        if (0x00 == cs)
        {
            return STATUS_WRONG_CHECKSUM;
        }
        else
        {
            PRINTF("Temperature: %u\n", (uint16_t) (data[1] / 10.0));
            PRINTF("Humidity: %u\n", (uint16_t) (data[0] / 10.0));
        }
    }

    return status;
}

void my_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    char buffer[256];
    int buffer_idx = 0;

    // Iterate over the format string
    for (const char *p = fmt; *p != '\0'; p++)
    {
        if (*p == '%' && *(p + 1) != '\0')
        {
            p++; // Skip the '%' character
            switch (*p)
            {
                case 'd':
                {
                    int i = va_arg(args, int);
                    // Convert integer to string manually
                    char temp[10];
                    int temp_idx = 0;
                    if (i == 0)
                    {
                        temp[temp_idx++] = '0';
                    }
                    else
                    {
                        if (i < 0)
                        {
                            buffer[buffer_idx++] = '-';
                            i                    = -i;
                        }
                        while (i > 0)
                        {
                            temp[temp_idx++] = (i % 10) + '0';
                            i                /= 10;
                        }
                        // Reverse the string
                        for (int j = temp_idx - 1; j >= 0; j--)
                        {
                            buffer[buffer_idx++] = temp[j];
                        }
                    }
                    break;
                }
                case 's':
                {
                    const char *s = va_arg(args, const char *);
                    while (*s)
                    {
                        buffer[buffer_idx++] = *s++;
                    }
                    break;
                }
                // Add more cases as needed for other format specifiers
                default:
                    buffer[buffer_idx++] = '%';
                    buffer[buffer_idx++] = *p;
                    break;
            }
        }
        else
        {
            buffer[buffer_idx++] = *p;
        }
    }

    buffer[buffer_idx] = '\0'; // Null-terminate the buffer
    va_end(args);

    // Output the buffer using __io_putchar
    for (int i = 0; i < buffer_idx; i++)
    {
        __io_putchar(buffer[i]);
    }
}

/**
 * @brief Retarget PRINTF to UART2
 * @param ch: character to be printed
 * @retval character printed
 */
int __io_putchar(int ch)
{
    uint8_t c[1];
    c[0] = ch & 0x00FF;
    HAL_UART_Transmit(&huart3, &*c, 1, 10);

    return ch;
}

/**
 * @brief Function to convert string to integer
 *
 * @return int
 */
int atoi(char *str)
{
    // Initialize result
    int res = 0;

    // Iterate through all characters
    // of input string and update result
    // take ASCII character of corresponding digit and
    // subtract the code from '0' to get numerical
    // value and multiply res by 10 to shuffle
    // digits left to update running total
    for (int i = 0; str[i] != '\0'; ++i)
        res = res * 10 + str[i] - '0';

    // return result.
    return res;
}
