/*
 * cli.c
 *
 *  Created on: Jun 14, 2024
 *      Author: gucd
 */

#include "cli.h"

#include "main.h"

#include <string.h>

#define MAX_ARGC (2u)

typedef struct
{
    const char *title;
    cmd_callback_t callback;
    const char *desc;
} cli_menu_t;

cli_menu_t cli_menu[] = {{"pt100", cmd_read_pt100, "Read the PT100 temperature"},
                         {"led", cmd_led, "Set or reset the LED (on/off)"},
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
        printf("%d) %s : %s\n", n, ptr->title, ptr->desc);

        ptr++;
        n++;
    }
    printf("\n");
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
    printf("%s\n", str_status[sts].str);
}

cli_status_t cmd_read_pt100(int argc, char **argv)
{
    cli_status_t status = STATUS_OK;

    printf("PT100 temperature is:%u\n", (uint16_t) max31865_readCelsius(&pt100_TempSensor));

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
