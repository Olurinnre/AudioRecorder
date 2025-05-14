#include <stdio.h>
#include <string.h>
#include "stm32f4xx.h"       // Include header for STM32F4xx microcontroller
#include "stm32f4xx_hal.h" // Include header for HAL library
#include "ff.h"            // Include header for FatFs file system
#include "diskio.h"        // Include header for disk I/O functions

// --- Configuration ---
// Sample Rate: How many audio samples we take per second
#define AUDIO_SAMPLE_RATE 16000 // Hz (e.g., 8000, 16000, 44100 are common)
// ADC and DMA settings
#define ADC_BUFFER_SIZE 1024    // Size of the buffer to store audio samples from ADC
//SD Card
#define SD_CARD_CS_PIN GPIO_PIN_12 // Chip Select pin for the SD card
#define SD_CARD_CS_PORT GPIOB      // Port where the SD card CS pin is located

// --- Hardware resources ---
ADC_HandleTypeDef hadc1;  // Handle for the ADC (Analog-to-Digital Converter)
DMA_HandleTypeDef hdma_adc1; // Handle for the DMA (Direct Memory Access)
TIM_HandleTypeDef htim2;  // Handle for Timer 2 (used to trigger ADC conversions)

// --- File system ---
FATFS fs;    // File system object (for FatFs)
FIL file;    // File object (for FatFs)

// --- Buffers ---
uint16_t adc_buffer[ADC_BUFFER_SIZE];       // Buffer to store ADC samples (16-bit)
uint8_t audio_buffer[ADC_BUFFER_SIZE * 2]; // Buffer to store audio data as 16-bit PCM

// Flags to indicate when data is ready
volatile uint8_t dma_half_transfer_complete = 0; // Flag for DMA half-transfer complete
volatile uint8_t dma_transfer_complete = 0;   // Flag for DMA transfer complete
volatile uint8_t sd_write_pending = 0;         // Flag to indicate when data is ready to be written to SD card

// --- Function prototypes ---
void SystemClock_Config(void);   // Function to configure the system clock
static void MX_GPIO_Init(void);     // Function to initialize GPIO pins
static void MX_DMA_Init(void);      // Function to initialize DMA
static void MX_ADC1_Init(void);     // Function to initialize ADC1
static void MX_TIM2_Init(void);     // Function to initialize Timer 2
void Error_Handler(void);          // Function to handle errors
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc); // Callback for DMA half-transfer
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);   // Callback for DMA transfer complete
uint32_t convert_to_pcm16(uint16_t adc_value);             // Function to convert ADC value to PCM
static uint8_t SD_Init(void);                             // Function to initialize the SD card
static uint8_t SD_Write(uint8_t *data, uint32_t len);      // Function to write data to the SD card
static void SD_Error_Handler(void);                       // Function to handle SD card errors

// --- Global variables ---
static char filename[32];       // Array to store the filename
static uint32_t file_index = 0; // Counter to generate unique filenames
static uint8_t sd_card_status = 0; // 0: Not initialized, 1: Initialized, 2: Error

int main(void) {
  // --- Initialization ---
  HAL_Init();             // Initialize the HAL library
  SystemClock_Config();   // Configure the system clock
  MX_GPIO_Init();         // Initialize GPIO pins
  MX_DMA_Init();          // Initialize DMA
  MX_ADC1_Init();         // Initialize ADC1
  MX_TIM2_Init();         // Initialize Timer 2 (before starting ADC)

  // Mount the file system (initialize SD card communication)
  if (f_mount(&fs, "", 0) != FR_OK) {
    Error_Handler(); // Handle file system error
  }

  sd_card_status = SD_Init(); // Initialize the SD card
  if (sd_card_status != 1) {
    Error_Handler(); // Stop if SD card init fails
  }

  // --- Main loop ---
  while (1) {
    // Check if data is ready to be written to the SD card
    if (sd_write_pending) {
      // Write the audio data to the SD card
      if (SD_Write(audio_buffer, sizeof(audio_buffer)) != FR_OK) {
        SD_Error_Handler(); // Handle SD card write error
      }
      sd_write_pending = 0; // Clear the flag
    }
  }
}

/**
  * @brief  System Clock Configuration
  * Configures the system clock source, frequency, and dividers.
  * This is crucial for setting the speed of the microcontroller.
  * @param  None
  * @retval None
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  // Enable power control clock
  __HAL_RCC_PWR_CLK_ENABLE();
  // Configure the main voltage regulator
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Initialize the RCC Oscillators
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE; // Use external high-speed oscillator (HSE)
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                 // Enable HSE
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                // Enable PLL (Phase Locked Loop)
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;       // Use HSE as PLL source
  RCC_OscInitStruct.PLL.PLLM = 8;                          // PLLM divider
  RCC_OscInitStruct.PLL.PLLN = 336;                         // PLLN multiplier
  RCC_OscInitStruct.PLL.PLLP = RCC_PLL_PLLP_DIV4;            // PLLP divider
  RCC_OscInitStruct.PLL.PLLQ = 7;                          // PLLQ divider
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler(); // Handle oscillator configuration error
  }

  // Initialize the CPU, AHB, and APB buses clocks
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Use PLL as system clock source
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;         // AHB clock divider
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;         // APB1 clock divider
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         // APB2 clock divider
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler(); // Handle clock configuration error
  }

  // Enable the SYSCFG clock
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  // Configure the voltage scaling
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  // Configure the Systick interrupt time (for HAL_Delay() function)
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
  // Configure the Systick
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
  // Set Systick interrupt priority
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief ADC1 Initialization Function
  * Configures ADC1 to read the microphone input.
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};

  // Enable the ADC1 clock
  __HAL_RCC_ADC1_CLK_ENABLE();

  hadc1.Instance = ADC1;                             // Select ADC1
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4; // Set clock prescaler
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;         // Set resolution to 12 bits
  hadc1.Init.ScanConvMode = DISABLE;                  // Disable scan mode
  hadc1.Init.ContinuousConvMode = DISABLE;            // Disable continuous conversion
  hadc1.Init.DiscontinuousConvMode = DISABLE;         // Disable discontinuous mode
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING; // Trigger on rising edge
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;    // Trigger from Timer 2
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;           // Align data to the right
  hadc1.Init.NbrOfConversion = 1;                     // Number of conversions per sequence
  hadc1.Init.DMAContinuousRequests = ENABLE;          // Enable DMA continuous requests
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE;            // End of conversion selection
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler(); // Handle ADC initialization error
  }

  // Configure ADC channel
  sConfig.Channel = ADC_CHANNEL_0;       // Select ADC channel 0 (PA0 pin)
  sConfig.Rank = 1;                    // Rank in the conversion sequence
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES; // Sampling time (adjust for your microphone)
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler(); // Handle ADC channel configuration error
  }
}

/**
  * @brief TIM2 Initialization Function
  * Configures Timer 2 to generate the ADC trigger signal.
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  // Enable the TIM2 clock
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = (SystemCoreClock / AUDIO_SAMPLE_RATE) - 1; // Calculate prescaler for desired sample rate
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;       // Counter mode: up counting
  htim2.Init.Period = 0;                             // Timer period (0 for continuous triggering)
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // Clock division
  htim2.Init.RepetitionCounter = 0;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler(); // Handle Timer 2 initialization error
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL; // Use internal clock source
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler(); // Handle Timer 2 clock source config error
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE; // Trigger ADC on timer update event
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler(); // Handle Timer 2 master config error
  }
}

/**
  * @brief DMA Initialization Function
  * Configures DMA to transfer ADC data to memory.
  * @param None
  * @retval None
  */
static void MX_DMA_Init(void) {
  // Enable the DMA2 clock
  __HAL_RCC_DMA2_CLK_ENABLE();

  // Configure DMA for ADC1
  hdma_adc1.Instance = DMA2_Stream0;
  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;       // Transfer from peripheral to memory
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;         // Peripheral address increment disabled
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;           // Memory address increment enabled
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD; // Peripheral data size: 16 bits
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;    // Memory data size: 16 bits
  hdma_adc1.Init.Mode = DMA_CIRCULAR;                  // Circular buffer mode
  hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;           // High priority
  hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;         // Disable FIFO mode
  hdma_adc1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_HALFFULL;
  hdma_adc1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_adc1.Init.PeriphBurst = DMA_PBURST_SINGLE;
  if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
    Error_Handler(); // Handle DMA initialization error
  }

  // Link DMA handle to ADC handle
  __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

  // Enable DMA interrupts
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);     // Set interrupt priority
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);             // Enable interrupt
}

/**
  * @brief GPIO Initialization Function
  * Configures the GPIO pins used in the application.
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // Enable GPIO clocks
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Configure SD card Chip Select pin (PB12)
  HAL_GPIO_WritePin(SD_CARD_CS_PORT, SD_CARD_CS_PIN, GPIO_PIN_SET); // Set CS pin high (inactive)
  GPIO_InitStruct.Pin = SD_CARD_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       // Output mode, push-pull
  GPIO_InitStruct.Pull = GPIO_PULLUP;             // Pull-up resistor enabled
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;    // High-speed output
  HAL_GPIO_Init(SD_CARD_CS_PORT, &GPIO_InitStruct); // Initialize the GPIO pin

  // Configure Microphone input pin (PA0)
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;       // Analog input mode
  GPIO_InitStruct.Pull = GPIO_NOPULL;           // No pull-up or pull-down resistor
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);     // Initialize the GPIO pin
}

/**
  * @brief  ADC conversion half complete callback
  * This function is called when half of the ADC buffer is filled.
  * It processes the first half of the data.
  * @param  hadc: pointer to ADC handle
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) {
  UNUSED(hadc); // Prevent unused parameter warning
  uint32_t i;

  // Convert ADC data to 16-bit PCM format and store it in the audio buffer
  for (i = 0; i < ADC_BUFFER_SIZE / 2; i++) {
    uint32_t pcm_value = convert_to_pcm16(adc_buffer[i]);         // Convert ADC value to PCM
    audio_buffer[i * 2] = pcm_value & 0xFF;         // Store low byte of PCM value
    audio_buffer[i * 2 + 1] = (pcm_value >> 8) & 0xFF; // Store high byte of PCM value
  }
  dma_half_transfer_complete = 1; // Set the flag
  sd_write_pending = 1;         // Set flag to write to SD card
}

/**
  * @brief  ADC conversion complete callback
  * This function is called when the entire ADC buffer is filled.
  * It processes the second half of the data.
  * @param  hadc: pointer to ADC handle
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  UNUSED(hadc); // Prevent unused parameter warning
  uint32_t i;

  // Convert ADC data to 16-bit PCM format and store it in the audio buffer
  for (i = ADC_BUFFER_SIZE / 2; i < ADC_BUFFER_SIZE; i++) {
    uint32_t pcm_value = convert_to_pcm16(adc_buffer[i]);         // Convert ADC value to PCM
    audio_buffer[i * 2] = pcm_value & 0xFF;         // Store low byte of PCM value
    audio_buffer[i * 2 + 1] = (pcm_value >> 8) & 0xFF; // Store high byte of PCM value
  }
  dma_transfer_complete = 1;   // Set the flag
  sd_write_pending = 1;         // Set flag to write to SD card
}

/**
  * @brief  Convert 12-bit ADC value to 16-bit PCM value
  * This function scales the 12-bit ADC value to a 16-bit PCM value.
  * @param  adc_value: 12-bit ADC value (0-4095)
  * @retval 16-bit PCM value (0-65535)
  */
uint32_t convert_to_pcm16(uint16_t adc_value) {
  return (adc_value << 4); // Left shift by 4 bits to expand to 16 bits
}

/**
  * @brief  Initialize the SD card
  * This function initializes the SD card and mounts the file system.
  * @retval 0: Error, 1: OK
  */
static uint8_t SD_Init(void) {
  // Check if the SD card is deselected
  if (HAL_GPIO_ReadPin(SD_CARD_CS_PORT, SD_CARD_CS_PIN) == GPIO_PIN_RESET) {
    return 0; // SD card not deselected.
  }
  // Mount file system
  if (f_mount(&fs, "", 0) != FR_OK) {
    return 0;
  }
  return 1;
}

/**
  * @brief  Write data to the SD card
  * This function writes the audio data to a new file on the SD card.
  * @param  data: pointer to data buffer
  * @param  len: data length in bytes
  * @retval FR_OK: success, other values: error
  */
static uint8_t SD_Write(uint8_t *data, uint32_t len) {
  FRESULT res; // Result of file operations
  UINT bw;     // Bytes written

  // Create a new file for each recording
  sprintf(filename, "audio_%04lu.wav", file_index++); // Generate filename
  res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE); // Open/create file
  if (res != FR_OK) {
    return res; // Error opening file
  }

  // Write the data to the file
  res = f_write(&file, data, len, &bw); // Write data to the file
  if (res != FR_OK || bw != len) {       // Check for write error and if all bytes were written
    f_close(&file);                 // Close the file
    return res;                   // Return the error code
  }

  // Close the file
  res = f_close(&file);
  if (res != FR_OK) {
    return res;
  }
  return FR_OK; // Success
}

/**
  * @brief  SD Card Error Handler
  * This function handles errors that occur during SD card operations.

  */
static void SD_Error_Handler(void) {
  // Handle SD card errors
  sd_card_status = 2; // Set SD card status to error
  Error_Handler();     // Call the general error handler
}

/**
  * It provides a basic error indication (blinking LED).
  */
void Error_Handler(void) {
  // Error handling
  __disable_irq(); // Disable interrupts
  while (1) {
    // Blink an LED to indicate an error (assuming LED is connected to GPIOD pin 13)
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(500);
  }
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line) {
  while (1) {
  }
}
#endif 
