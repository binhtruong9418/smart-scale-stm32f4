/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdarg.h>
#include <math.h>
#include <stdio.h>			
#include <string.h>			
#include "tm_stm32f4_mfrc522.h"
#include "7seg.h"
#include "HX711.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint8_t uid[5];      // UID card
    char name[100];      // name of card
    float   weight;      // weight
    uint32_t timestamp;  // timestamp (ms from system start)
} HistoryEntry;

typedef struct {
    float current_weight;
    float previous_weight;
    uint8_t weight_stable_count;
    uint8_t system_ready;
    uint32_t last_measurement_time;
    uint32_t last_display_time;
    float weight_threshold;
} scale_state_t;

typedef struct {
    float history[50];  // use MAX_WEIGHT_HISTORY #define
    int head;           // Next pointer
    int count;          // Total valid data
    float saved_weight; // Stable weight last scale
} WeightHistory;

typedef struct {
    uint8_t uid[5]; 
    WeightHistory weight_data;
    char name[100];
} CardData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Config save history
#define MAX_WEIGHT_HISTORY 5    // Start save weight when have 5 stable weight
#define MAX_REGISTERED_CARDS 20 // Save 20 card max
#define MAX_WEIGHING_HISTORY 50 // Save latest 50 cards history

// Configuration constants
#define WEIGHT_STABILITY_THRESHOLD  0.01f  // 10g threshold for stability
#define WEIGHT_STABILITY_COUNT      5      // Number of stable readings required
#define MEASUREMENT_INTERVAL        100    // ms between measurements
#define DISPLAY_UPDATE_INTERVAL     500    // ms between display updates
#define WEIGHT_FILTER_ALPHA         0.3f   // Low-pass filter coefficient

//Config loadcell
#define HX711_DT_PORT     GPIOA
#define HX711_DT_PIN      GPIO_PIN_11
#define HX711_SCK_PORT    GPIOA
#define HX711_SCK_PIN     GPIO_PIN_12
#define SCALE_FACTOR      83000.0f  // Calibration factor (83000 for 10kg and 44000 for 20kg load cell)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

scale_state_t scale_state = {0};
HistoryEntry weighing_history[MAX_WEIGHING_HISTORY];

int history_count = 0; // Số mục hợp lệ trong lịch sử
int history_head = 0;  // Vị trí tiếp theo để ghi (đầu của bộ đệm)

hx711_t hx711;
char uart_buffer[128];
CardData card_database[MAX_REGISTERED_CARDS];
int registered_card_count = 0;

// Buffer for receive from UART
uint8_t uart_rx_buffer[64];
uint8_t uart_rx_data;
uint8_t uart_rx_index = 0;
volatile uint8_t uart_command_ready = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

void scale_init(void);
void scale_calibrate(void);
float scale_read_weight(void);
float scale_filter_weight(float raw_weight);
uint8_t scale_is_weight_stable(float weight);
void scale_display_weight(float weight);
void scale_process_rfid(float weight);
void scale_send_uart_data(const char* format, ...);
void scale_handle_error(const char* error_msg);
void add_to_weighing_history(CardData* card, float weight);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_SPI4_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6);

  // Initialize scale system
  scale_init();

  //RFID
  TM_MFRC522_Init();
  scale_send_uart_data("Smart Scale System Initialized\r\n");

  HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);

  HAL_Delay(1000);
  scale_state.system_ready = 1;
  scale_state.last_measurement_time = HAL_GetTick();
  scale_state.last_display_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (uart_command_ready) {
		   process_uart_command();
	   }

	  uint32_t current_time = HAL_GetTick();
	  // Periodic weight measurement
	  if (current_time - scale_state.last_measurement_time >= MEASUREMENT_INTERVAL) {
		  scale_state.last_measurement_time = current_time;
		  // Read and filter weight
		  float raw_weight = scale_read_weight();
		  float filtered_weight = scale_filter_weight(raw_weight);
		  scale_state.current_weight = filtered_weight;

		  // Check weight stability
		  if (scale_is_weight_stable(filtered_weight)) {
			  if (current_time - scale_state.last_display_time >= DISPLAY_UPDATE_INTERVAL) {
				  scale_state.last_display_time = current_time;
			  }
		  } else {
			  // Weight is not stable, but still update display more frequently for debugging
			  if (current_time - scale_state.last_display_time >= (DISPLAY_UPDATE_INTERVAL / 2)) {
				  scale_state.last_display_time = current_time;
				  scale_display_weight(scale_state.current_weight);
				  scale_process_rfid(scale_state.current_weight);
			  }
		  }
	  }
	  // Small delay to prevent excessive CPU usage
	  HAL_Delay(10);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE4 PE8 PE9 PE10
                           PE11 PE12 PE13 PE14
                           PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PG2 PG3 PG13 PG14 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

CardData* find_or_register_card(uint8_t* card_uid) {
    // 1. Find existing card
    for (int i = 0; i < registered_card_count; i++) {
        if (memcmp(card_database[i].uid, card_uid, 5) == 0) {
            return &card_database[i];
        }
    }

    // 2. If no card found, register new card
    if (registered_card_count < MAX_REGISTERED_CARDS) {
        CardData* new_card = &card_database[registered_card_count];
        memcpy(new_card->uid, card_uid, 5);
        // Initialize weight history
        memset(&new_card->weight_data, 0, sizeof(WeightHistory));
        new_card->name[0] = '\0';
        registered_card_count++;
        scale_send_uart_data("New card registered. Total: %d\r\n", registered_card_count);
        return new_card;
    }

    // 3. If database is full
    scale_send_uart_data("ERROR: Card database is full. Cannot register new card.\r\n");
    return NULL;
}

void push_weight_to_history(WeightHistory* history, float weight) {
    history->history[history->head] = weight;
    history->head = (history->head + 1) % MAX_WEIGHT_HISTORY;
    if (history->count < MAX_WEIGHT_HISTORY) {
        history->count++;
    }
}

uint8_t are_all_weights_stable(WeightHistory* history) {
    if (history->count < MAX_WEIGHT_HISTORY) {
        return 0;
    }

    float first_weight = history->history[0];
    for (int i = 1; i < MAX_WEIGHT_HISTORY; i++) {
        if (fabsf(history->history[i] - first_weight) > WEIGHT_STABILITY_THRESHOLD) {
            return 0;
        }
    }

    return 1;
}

void process_uart_command(void) {
    uart_rx_buffer[strlen((char*)uart_rx_buffer) - 1] = 0; // Set the last character = NULL
    char* command = (char*)uart_rx_buffer;
	
    if (strcmp(command, "LIST") == 0) {
        scale_send_uart_data("\r\n--- Registered Card List ---\r\n");
        if (registered_card_count == 0) {
            scale_send_uart_data("No cards registered yet.\r\n");
        } else {
            for (int i = 0; i < registered_card_count; i++) {
                const char* display_name = (card_database[i].name[0] == '\0') ? "(not set)" : card_database[i].name;
            	scale_send_uart_data(
			"Card %d | Name: %-15s | UID: %02X%02X%02X%02X%02X | Saved Weight: %.1f kg\r\n",
			 i + 1,
		 	display_name,
		 	card_database[i].uid[0], card_database[i].uid[1], card_database[i].uid[2], card_database[i].uid[3], card_database[i].uid[4],
		 	card_database[i].weight_data.saved_weight
		);
            }
        }
        scale_send_uart_data("----------------------------\r\n");
    } else if (strncmp(command, "SET ", 4) == 0) {
        char name_to_set[100];
        int card_index;
        if (sscanf(command, "SET %s %d", name_to_set, &card_index) == 2) { 
            // Validate card index input
            if (card_index > 0 && card_index <= registered_card_count) {
                // Transform user input to card index
                CardData* card = &card_database[card_index - 1];

                // Copy user input card name to card name
                strncpy(card->name, name_to_set, sizeof(card->name) - 1);
                card->name[sizeof(card->name) - 1] = '\0';

                scale_send_uart_data("Success: Name '%s' has been set for Card %d.\r\n", card->name, card_index);
            } else {
                scale_send_uart_data("Error: Invalid card index. Please use an index from 1 to %d.\r\n", registered_card_count);
            }
        } else {
            scale_send_uart_data("Error: Invalid command format. Use: SET <name> <index>\r\n");
        }
    } else if (strcmp(command, "HISTORY") == 0) {
        scale_send_uart_data("\r\n--- Weighing History (Oldest to Newest) ---\r\n");
        if (history_count == 0) {
            scale_send_uart_data("History is empty.\r\n");
        } else {
            int start_index = 0;
            if (history_count == MAX_WEIGHING_HISTORY) {
                start_index = history_head;
            }

            for (int i = 0; i < history_count; i++) {
                int current_index = (start_index + i) % MAX_WEIGHING_HISTORY;
                HistoryEntry* entry = &weighing_history[current_index];
                const char* display_name = (entry->name[0] == '\0') ? "(not set)" : entry->name;

                scale_send_uart_data(
			"#%d | Time: %lu ms | Name: %-15s | Weight: %.1f kg\r\n",
		     	i + 1,
		     	entry->timestamp,
		     	display_name,
		     	entry->weight
		);
            }
        }
        scale_send_uart_data("--------------------------------------------\r\n");
    }
    else {
        scale_send_uart_data("Unknown command: %s\r\n", uart_rx_buffer);
    }

    // Reset buffer and flag
    memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));
    uart_rx_index = 0;
    uart_command_ready = 0;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        if (uart_rx_data == '\n' || uart_rx_data == '\r' || uart_rx_index >= sizeof(uart_rx_buffer) - 1) {
            uart_rx_buffer[uart_rx_index] = '\0'; // End of buffer
            if (uart_rx_index > 0) {
                uart_command_ready = 1; // Turn on flag
            }
            uart_rx_index = 0;
        } else {
            uart_rx_buffer[uart_rx_index++] = uart_rx_data;
        }
        // Kích hoạt lại ngắt nhận UART cho byte tiếp theo
        HAL_UART_Receive_IT(&huart1, &uart_rx_data, 1);
    }
}

void save_weight_to_rfid_card(uint8_t* card_uid, float weight) {
    scale_send_uart_data("------ SAVING TO CARD ------\r\n");
    scale_send_uart_data("Card UID: %02X%02X%02X%02X%02X\r\n",
                         card_uid[0], card_uid[1], card_uid[2], card_uid[3], card_uid[4]);
    scale_send_uart_data("Saving Weight: %.1f kg\r\n", weight);
    scale_send_uart_data("--------------------------\r\n");
}

void scale_init(void)
{
    scale_send_uart_data("Initializing HX711 load cell...\r\n");
    hx711_init(&hx711, HX711_SCK_PORT, HX711_SCK_PIN, HX711_DT_PORT, HX711_DT_PIN);
    set_gain(&hx711, 128, 32);
	
    if (is_ready(&hx711)) {
	scale_send_uart_data("HX711 is ready!\r\n");
    } else {
	scale_send_uart_data("HX711 is NOT ready!\r\n");
    }
	
    set_scale(&hx711, SCALE_FACTOR, SCALE_FACTOR);
    scale_calibrate();

    // Initialize state variables
    scale_state.current_weight = 0.0f;
    scale_state.previous_weight = 0.0f;
    scale_state.weight_stable_count = 0;
    scale_state.weight_threshold = WEIGHT_STABILITY_THRESHOLD;

    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_14, GPIO_PIN_SET); // Turn on green LED when scale initialization complete
    scale_send_uart_data("Scale initialization complete\r\n");
}

void scale_calibrate(void)
{
    scale_send_uart_data("Calibrating scale... Please ensure scale is empty\r\n");
    HAL_Delay(2000);
    tare_all(&hx711, 10);
    scale_send_uart_data("Scale calibration complete\r\n");
}

float scale_read_weight(void)
{
    if (!is_ready(&hx711)) return 0.0f;
    float weight = get_weight(&hx711, 10, CHANNEL_A);
    return (weight < 0.0f) ? 0.0f : weight;
}

float scale_filter_weight(float raw_weight)
{
    static float filtered_weight = 0.0f;
    static uint8_t first_reading = 1;

    if (first_reading) {
        filtered_weight = raw_weight;
        first_reading = 0;
    } else {
        // Simple low-pass filter: y[n] = α * x[n] + (1-α) * y[n-1]
        filtered_weight = WEIGHT_FILTER_ALPHA * raw_weight + (1.0f - WEIGHT_FILTER_ALPHA) * filtered_weight;
    }

    return filtered_weight;
}

uint8_t scale_is_weight_stable(float weight)
{
    float weight_diff = fabsf(weight - scale_state.previous_weight);

    if (weight_diff < scale_state.weight_threshold) {
        scale_state.weight_stable_count++;
        if (scale_state.weight_stable_count >= WEIGHT_STABILITY_COUNT) {
            scale_state.weight_stable_count = WEIGHT_STABILITY_COUNT;  // Cap the counter
            scale_state.previous_weight = weight;
            return 1;
        }
    } else {
        scale_state.weight_stable_count = 0;
        scale_state.previous_weight = weight;
    }

    return 0;
}

void scale_display_weight(float weight)
{
    if (weight < 0.0f) weight = 0.0f;
    if (weight > 9.9f) weight = 9.9f;

    int display_value = (int)(weight * 10 + 0.5f);
    Set7SegDisplayWithDecimal(display_value, 1);
    Run7SegDisplay();
    scale_send_uart_data("Weight: %d (display_value) = %.3f kg\r\n", display_value, weight);
}

void scale_process_rfid(float weight)
{
    uint8_t CardUID[5];

    if (TM_MFRC522_Check(CardUID) == MI_OK) {
        CardData* current_card = find_or_register_card(CardUID);
	if (current_card == NULL) {
		scale_send_uart_data("Full database\r\n");
		// Error: Database if full
		return;
	}

	push_weight_to_history(&current_card->weight_data, weight);

	if (are_all_weights_stable(&current_card->weight_data)) {
		float stable_weight = current_card->weight_data.history[0]; // Get stable value

		// Only save new stable value if it different from with old one
		if (fabsf(stable_weight - current_card->weight_data.saved_weight) >= 0.1f) {
			save_weight_to_rfid_card(current_card->uid, stable_weight);
			current_card->weight_data.saved_weight = stable_weight;
			add_to_weighing_history(current_card, stable_weight);
		}
	}

	scale_send_uart_data("Card: %02X%02X%02X%02X%02X | Weight: %.1f kg | History Cnt: %d\r\n",
			CardUID[0], CardUID[1], CardUID[2], CardUID[3], CardUID[4], weight, current_card->weight_data.count);
    }
}

void add_to_weighing_history(CardData* card, float weight) {
    HistoryEntry* new_entry = &weighing_history[history_head];

    memcpy(new_entry->uid, card->uid, 5);
    strncpy(new_entry->name, card->name, sizeof(new_entry->name) - 1);
    new_entry->name[sizeof(new_entry->name) - 1] = '\0';
    new_entry->weight = weight;
    new_entry->timestamp = HAL_GetTick();
    history_head = (history_head + 1) % MAX_WEIGHING_HISTORY;
    if (history_count < MAX_WEIGHING_HISTORY) {
        history_count++;
    }

    scale_send_uart_data("OK: New entry added to weighing history.\r\n");
}

void scale_send_uart_data(const char* format, ...)
{
    va_list args;
    va_start(args, format);

    int len = vsnprintf(uart_buffer, sizeof(uart_buffer), format, args);

    va_end(args);

    if (len > 0 && len < sizeof(uart_buffer)) {
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, len, 1000);
    }
}

void scale_handle_error(const char* error_msg)
{
    scale_send_uart_data("ERROR: %s\r\n", error_msg);

    // Flash both LEDs to indicate error
    for (int i = 0; i < 5; i++) {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_Delay(200);
    }
}

/* USER CODE END 4 */

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

#ifdef  USE_FULL_ASSERT
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
