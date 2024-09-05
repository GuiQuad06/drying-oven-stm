/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cli.h"
#include "dht22.h"
#include "esp8266.h"
#include "sensor_callbacks.h"

#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WANT_SEGGER_SYSVIEW (0u)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart3_rx;

osThreadId defaultTaskHandle;
uint32_t defaultTaskBuffer[128];
osStaticThreadDef_t defaultTaskControlBlock;
osThreadId sensorTaskHandle;
uint32_t sensorTaskBuffer[128];
osStaticThreadDef_t sensorTaskControlBlock;
osThreadId httpTaskHandle;
uint32_t httpTaskBuffer[128];
osStaticThreadDef_t httpTaskControlBlock;
osThreadId cliTaskHandle;
uint32_t cliTaskBuffer[128];
osStaticThreadDef_t cliTaskControlBlock;
osMutexId dataMutexHandle;
osStaticMutexDef_t dataMutexControlBlock;
/* USER CODE BEGIN PV */
dht22_t dht22;
esp8266_t esp8266;
max31865_t pt100_TempSensor;
static const char fw_version[] = "0.0.1";

const char *hello_http      = "<h1>Hello World!</h1>";
volatile uint8_t cpt_addr   = 0;
volatile uint16_t prev_size = 0;

static uint8_t rx_buffer[INPUT_BUF_SIZE];
static char cli_buffer[INPUT_BUF_SIZE]; /** Input buffer for the command line interpreter. */
static uint8_t esp_buffer[MAX_BUFFER_LEN];
static char esp_freeze_buffer[MAX_BUFFER_LEN];

volatile uint8_t cli_flag = 0;

typedef union
{
    uint64_t data;

    struct __attribute__((__packed__))
    {
        uint16_t oven_temperature;
        uint16_t int_temp_celsius;
        uint16_t int_humidity;
    } unwrapped;
} sensor_values_t;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(const void *argument);
void StartSensorTask(const void *argument);
void StartHttpTask(const void *argument);
void StartCliTask(const void *argument);

/* USER CODE BEGIN PFP */
static void ask_user_credentials(esp8266_t *esp8266);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief This function is called to increment  a global variable "uwTick"
 *        used as application time base.
 * @note We need to increment tick by different value depending on
 *        the configTICK_RATE_HZ constant
 * @note This function will overload the weak version
 * @retval None
 */
void HAL_IncTick(void)
{
    switch (configTICK_RATE_HZ)
    {
        case 1000:
            uwTick += HAL_TICK_FREQ_1KHZ;
            break;
        case 100:
            uwTick += HAL_TICK_FREQ_100HZ;
            break;
        case 10:
            uwTick += HAL_TICK_FREQ_10HZ;
            break;
        default:
            uwTick += HAL_TICK_FREQ_DEFAULT;
    }
}

/**
 * @brief      UART Rx callback function
 * @param[in]  huart UART handle
 * @param[in]  Size Number of bytes received
 * @retval     None
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BaseType_t is_higher_priority_task_woken = pdFALSE;

    if (USART3 == huart->Instance)
    {
        /** Copy RxBuffer to cli_input buffer for processing*/
        memcpy(cli_buffer, rx_buffer, Size);

        /** Set New message available for processing */
        // TODO This is a temporary solution to avoid Notify a task while scheduler has not yet started
        // Best solution could be to use a InitTask to ask_user_credentials and kill it after
        // so that we won't need anymore cli_flag
        if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
        {
            cli_flag = 1;
            return;
        }
        vTaskNotifyGiveFromISR(cliTaskHandle, &is_higher_priority_task_woken);
    }
    else if (USART1 == huart->Instance)
    {
        // Do something with the ESP8266 received data
        memcpy(esp_freeze_buffer, esp_buffer, Size);

        /** Set New message available for processing*/
        if (MAX_BUFFER_LEN == Size)
        {
            vTaskNotifyGiveFromISR(httpTaskHandle, &is_higher_priority_task_woken);
        }
        else
        {
            /* start the DMA while entire message has not been received */
            HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) esp_buffer, MAX_BUFFER_LEN);
            __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
        }
    }
    else
    {
        return;
    }
    portYIELD_FROM_ISR(is_higher_priority_task_woken);
}

static void ask_user_credentials(esp8266_t *esp8266)
{
    char ssid[MAX_CHAR_SSID];
    char password[MAX_CHAR_PWD];

    PRINTF("Enter SSID for wifi connection:\n");

    while (!cli_flag)
    {
    }
    memcpy(ssid, cli_buffer, strlen(cli_buffer) + 1);

    /* start the DMA again */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *) rx_buffer, INPUT_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

    cli_flag = 0;

    PRINTF("Enter password for wifi connection:\n");

    while (!cli_flag)
    {
    }
    memcpy(password, cli_buffer, strlen(cli_buffer) + 1);

    /* start the DMA again */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *) rx_buffer, INPUT_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

    cli_flag = 0;

    esp_8266_set_credentials(esp8266, ssid, password);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    esp8266_status_t esp_sts = ESP8266_OK;

    dht22_init(&dht22, gpio_input_dir, gpio_write, delay_us, gpio_read, gpio_output_dir);

    esp8266_init(&esp8266, delay_ms, send_message);

    max31865_init(&pt100_TempSensor,
                  chipselect_cb,
                  spi_trx_cb,
                  charge_time_delay_cb,
                  conversion_time_delay_cb,
                  threshold_fault,
                  threshold_fault,
                  100,    // RTD resistance
                  432,    // Rref resistance
                  0,      // lower fault threshold
                  0x7fff, // higher fault threshold
                  false,  // 2 wire
                  false); // 50Hz filter

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_SPI2_Init();
    MX_USART1_UART_Init();
    MX_TIM4_Init();
    MX_TIM3_Init();
    MX_USART3_UART_Init();
    /* USER CODE BEGIN 2 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rx_buffer, INPUT_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

    PRINTF("Coucou Hibou\nSoftware Version %s\n", fw_version);

    ask_user_credentials(&esp8266);
    esp_sts = esp8266_connect(&esp8266);
    PRINTF("%s\n", (esp_sts == ESP8266_OK) ? "Connected to wifi" : "Failed to connect to wifi");

    print_cli_menu();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, esp_buffer, MAX_BUFFER_LEN);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    // Enabling the CM3 Cycle Counter
    DWT->CTRL |= (1U << 0);
    SEGGER_SYSVIEW_Conf();
    /* USER CODE END 2 */

    /* Create the mutex(es) */
    /* definition and creation of dataMutex */
    osMutexStaticDef(dataMutex, &dataMutexControlBlock);
    dataMutexHandle = osMutexCreate(osMutex(dataMutex));

    /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
    /* USER CODE END RTOS_MUTEX */

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
    /* USER CODE END RTOS_TIMERS */

    /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
    /* USER CODE END RTOS_QUEUES */

    /* Create the thread(s) */
    /* definition and creation of defaultTask */
    osThreadStaticDef(defaultTask,
                      StartDefaultTask,
                      osPriorityIdle,
                      0,
                      128,
                      defaultTaskBuffer,
                      &defaultTaskControlBlock);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* definition and creation of sensorTask */
    osThreadStaticDef(sensorTask, StartSensorTask, osPriorityNormal, 0, 128, sensorTaskBuffer, &sensorTaskControlBlock);
    sensorTaskHandle = osThreadCreate(osThread(sensorTask), NULL);

    /* definition and creation of httpTask */
    osThreadStaticDef(httpTask, StartHttpTask, osPriorityAboveNormal, 0, 128, httpTaskBuffer, &httpTaskControlBlock);
    httpTaskHandle = osThreadCreate(osThread(httpTask), NULL);

    /* definition and creation of cliTask */
    osThreadStaticDef(cliTask, StartCliTask, osPriorityHigh, 0, 128, cliTaskBuffer, &cliTaskControlBlock);
    cliTaskHandle = osThreadCreate(osThread(cliTask), NULL);

    /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
    /* USER CODE END RTOS_THREADS */

    /* Start scheduler */
    osKernelStart();

    /* We should never get here as control is now taken by the scheduler */
    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void)
{
    /* USER CODE BEGIN SPI2_Init 0 */

    /* USER CODE END SPI2_Init 0 */

    /* USER CODE BEGIN SPI2_Init 1 */

    /* USER CODE END SPI2_Init 1 */
    /* SPI2 parameter configuration*/
    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_HIGH;
    hspi2.Init.CLKPhase          = SPI_PHASE_2EDGE;
    hspi2.Init.NSS               = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    hspi2.Init.CRCPolynomial     = 10;
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI2_Init 2 */

    /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{
    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 64000 - 1;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */
    __HAL_TIM_ENABLE(&htim3);
    /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{
    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig     = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 64 - 1;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 65535;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */
    __HAL_TIM_ENABLE(&htim4);
    /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{
    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{
    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance          = USART3;
    huart3.Init.BaudRate     = 115200;
    huart3.Init.WordLength   = UART_WORDLENGTH_8B;
    huart3.Init.StopBits     = UART_STOPBITS_1;
    huart3.Init.Parity       = UART_PARITY_NONE;
    huart3.Init.Mode         = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel3_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    /* DMA1_Channel5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, LD2_Pin | DHT_22_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin : B1_Pin */
    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
    GPIO_InitStruct.Pin   = USART_TX_Pin | USART_RX_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : LD2_Pin DHT_22_Pin */
    GPIO_InitStruct.Pin   = LD2_Pin | DHT_22_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : CS_Pin */
    GPIO_InitStruct.Pin   = CS_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    // Set chip select to high level
    GPIOB->BSRR = GPIO_PIN_12;
    // Set DHT22 pin to high level
    GPIOA->BSRR = GPIO_PIN_6;
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(const void *argument)
{
    /* USER CODE BEGIN 5 */
    const TickType_t xDelay = 20000 / portTICK_PERIOD_MS;
    bool segger_started     = false;
    /* Infinite loop */
    for (;;)
    {
        if (!segger_started && WANT_SEGGER_SYSVIEW)
        {
            SEGGER_SYSVIEW_Start();
            segger_started = true;
        }

        vTaskDelay(xDelay);
    }
    /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartSensorTask */
/**
 * @brief Function implementing the sensorTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(const void *argument)
{
    /* USER CODE BEGIN StartSensorTask */
    sensor_values_t sensor_values = {.data = 0};

    TickType_t xLastWakeTime;
    uint8_t cs                              = 0xFF;
    uint16_t int_sensor_values[DHT22_FRAME] = {0};
    // Run the task every 10 seconds
    const TickType_t xDelay                 = 10000 / portTICK_PERIOD_MS;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();
    /* Infinite loop */
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xDelay);

        sensor_values.unwrapped.oven_temperature = (uint16_t) max31865_readCelsius(&pt100_TempSensor);

        (void) dht22_start(&dht22);
        cs = dht22_read_data(&dht22, int_sensor_values, DHT22_FRAME);
        if (0x00 == cs)
        {
            sensor_values.unwrapped.int_temp_celsius = (uint16_t) (int_sensor_values[1] / 10.0);
            sensor_values.unwrapped.int_humidity     = (uint16_t) (int_sensor_values[0] / 10.0);
        }
        if (dataQueueHandle != NULL)
        {
            if (xQueueSend(dataQueueHandle, (void *) &sensor_values.data, (TickType_t) 10) != pdPASS)
            {
                PRINTF("Failed to send sensor data to the queue\n");
            }
        }
    }
    /* USER CODE END StartSensorTask */
}

/* USER CODE BEGIN Header_StartHttpTask */
/**
 * @brief Function implementing the httpTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartHttpTask */
void StartHttpTask(const void *argument)
{
    /* USER CODE BEGIN StartHttpTask */
    sensor_values_t rx_data;

    char buffer[50]; // Adjust the size as needed
    char *html_start = "<h1>Temp :";
    char *html_end   = "</h1>";
    char temp_str[4]; // Assuming temperature value won't exceed 999999999
    /* Infinite loop */
    for (;;)
    {
        // Blocked task until the notification has been received
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (dataQueueHandle != NULL)
        {
            if (xQueueReceive(dataQueueHandle, &rx_data.data, portMAX_DELAY) != pdPASS)
            {
                PRINTF("Failed to receive int temperature data from the queue\n");
            }
        }

        // Convert temperature to string
        int temp_value = rx_data.unwrapped.int_temp_celsius;
        int i          = 0;
        while (temp_value > 0)
        {
            temp_str[i++] = (temp_value % 10) + '0';
            temp_value    /= 10;
        }
        temp_str[i] = '\0';

        // Reverse the temp_str
        for (int start = 0, end = i - 1; start < end; start++, end--)
        {
            char temp       = temp_str[start];
            temp_str[start] = temp_str[end];
            temp_str[end]   = temp;
        }

        // Manually copy the strings into the buffer
        int j = 0;
        for (i = 0; html_start[i] != '\0'; i++, j++)
        {
            buffer[j] = html_start[i];
        }
        for (i = 0; temp_str[i] != '\0'; i++, j++)
        {
            buffer[j] = temp_str[i];
        }
        for (i = 0; html_end[i] != '\0'; i++, j++)
        {
            buffer[j] = html_end[i];
        }
        buffer[j] = '\0';

        http_send_data(&esp8266, esp_freeze_buffer, strlen(esp_freeze_buffer), buffer, strlen(buffer));

        /** Clear buffer */
        memset(esp_freeze_buffer, 0, MAX_BUFFER_LEN);
        /* start the DMA again */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t *) esp_buffer, MAX_BUFFER_LEN);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
    /* USER CODE END StartHttpTask */
}

/* USER CODE BEGIN Header_StartCliTask */
/**
 * @brief Function implementing the cliTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCliTask */
void StartCliTask(const void *argument)
{
    /* USER CODE BEGIN StartCliTask */
    /* Infinite loop */
    for (;;)
    {
        // Blocked task until the notification has been received
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        cli_input(cli_buffer);

        /** Clear buffer */
        memset(cli_buffer, 0, INPUT_BUF_SIZE);
        /* start the DMA again */
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, (uint8_t *) rx_buffer, INPUT_BUF_SIZE);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
    /* USER CODE END StartCliTask */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
