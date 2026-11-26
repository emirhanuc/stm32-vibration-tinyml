/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : PM Edge Node V2 - real-time vibration fault classification
  *                   on STM32F407G-DISC1.
  *
  * Pipeline:
  *   TIM2 @ 800 Hz -> LIS3DSH (SPI1) -> 1024-sample double-buffered window
  *   -> 51 time/frequency features -> Random Forest (150 trees, 6 classes)
  *   -> result + raw window streamed over USB CDC (virtual COM port).
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "lis3dsh_driver.h"
#include "sampling.h"
#include "pm_inference.h"
#include "usbd_def.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USB_TX_TIMEOUT_MS   3000U
#define USB_MSG_SIZE        128U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* USB device handle is defined in USB_DEVICE/App/usb_device.c. */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* Static buffers: keeps the large arrays off the Cortex-M4 stack. */
static int16_t ai_xyz[PM_WINDOW * 3];
static float   ai_features[PM_FEATURE_COUNT];
static float   ai_probabilities[PM_CLASS_COUNT];

static uint32_t ai_window_counter  = 0U;
static uint32_t ai_inference_ms    = 0U;
static int      ai_last_prediction = -1;
static int      ai_last_status     = -1;

static LIS3DSH_Status_t        sensor_status;
static const SamplingWindow_t *ready_window = NULL;
static SamplingStats_t         sampling_stats;
static char                    usb_message[USB_MSG_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static int USB_SendText(const char *text, uint32_t timeout_ms);
static int AI_RunWindow(const SamplingWindow_t *window);
static void Report_Window(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Send a NUL-terminated string over USB CDC with a bounded wait.
  *         Waits until the transfer has completed so the caller may reuse
  *         the buffer. Never blocks forever if the host disconnects.
  * @note   Call from main context only, never from an interrupt.
  * @retval 0 on success, -1 on error/timeout.
  */
static int USB_SendText(const char *text, uint32_t timeout_ms)
{
  if (text == NULL)
  {
    return -1;
  }

  size_t n = strlen(text);
  if ((n == 0U) || (n > 0xFFFFU))
  {
    return -1;
  }

  uint32_t started = HAL_GetTick();
  uint8_t status;

  do
  {
    status = CDC_Transmit_FS((uint8_t *)text, (uint16_t)n);
    if (status == USBD_OK)
    {
      break;
    }
    if (status != USBD_BUSY)
    {
      return -1;
    }
    HAL_Delay(1U);
  } while ((uint32_t)(HAL_GetTick() - started) < timeout_ms);

  if (status != USBD_OK)
  {
    return -1;
  }

  /* CDC_Transmit_FS is asynchronous: wait until the IN transfer is done. */
  while ((uint32_t)(HAL_GetTick() - started) < timeout_ms)
  {
    if (hUsbDeviceFS.pClassData == NULL)
    {
      return -1;
    }

    const USBD_CDC_HandleTypeDef *cdc =
        (const USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    if (cdc->TxState == 0U)
    {
      return 0;
    }
    HAL_Delay(1U);
  }

  return -1;
}

/**
  * @brief  Run feature extraction + Random Forest on one window.
  *         Samples are interleaved [x0,y0,z0,x1,...] exactly as in Python.
  * @retval Predicted class index (0..PM_CLASS_COUNT-1) or -1 on error.
  */
static int AI_RunWindow(const SamplingWindow_t *window)
{
  if (window == NULL)
  {
    return -1;
  }

  for (uint32_t i = 0U; i < PM_WINDOW; ++i)
  {
    ai_xyz[(3U * i) + 0U] = window->x[i];
    ai_xyz[(3U * i) + 1U] = window->y[i];
    ai_xyz[(3U * i) + 2U] = window->z[i];
  }

  if (pm_extract_features(ai_xyz, ai_features) != 0)
  {
    return -1;
  }

  int prediction = pm_predict_rf(ai_features, ai_probabilities);
  if ((prediction < 0) || (prediction >= PM_CLASS_COUNT))
  {
    return -1;
  }

  ai_window_counter++;
  return prediction;
}

/**
  * @brief  Stream the ready window and the inference result over USB CDC.
  *
  * Output format (parsed by the Python collector):
  *   WINDOW <n> | samples=1024 | missed=<m> | dropped=<d>
  *   <index>,<x>,<y>,<z>          (1024 lines, raw LIS3DSH counts)
  *   AI_RESULT window=<n> class=1.<c> inference_ms=<t>
  *   or AI_ERROR window=<n> inference_ms=<t>
  */
static void Report_Window(void)
{
  int length;
  int usb_ok;

  Sampling_GetStats(&sampling_stats);

  length = snprintf(usb_message, sizeof(usb_message),
                    "\r\nWINDOW %lu | samples=%lu | missed=%lu | dropped=%lu\r\n",
                    (unsigned long)sampling_stats.completed_windows,
                    (unsigned long)SAMPLING_WINDOW_SIZE,
                    (unsigned long)sampling_stats.missed_samples,
                    (unsigned long)sampling_stats.dropped_windows);

  usb_ok = ((length > 0) && ((size_t)length < sizeof(usb_message)))
               ? USB_SendText(usb_message, USB_TX_TIMEOUT_MS)
               : -1;

  if (usb_ok == 0)
  {
    for (uint32_t i = 0U; i < SAMPLING_WINDOW_SIZE; ++i)
    {
      length = snprintf(usb_message, sizeof(usb_message),
                        "%lu,%d,%d,%d\r\n",
                        (unsigned long)i,
                        (int)ready_window->x[i],
                        (int)ready_window->y[i],
                        (int)ready_window->z[i]);

      if ((length <= 0) || ((size_t)length >= sizeof(usb_message)) ||
          (USB_SendText(usb_message, USB_TX_TIMEOUT_MS) != 0))
      {
        usb_ok = -1;
        break;
      }
    }
  }

  /* Metadata line is sent separately, never between indexed CSV rows. */
  if (usb_ok == 0)
  {
    if (ai_last_status == 0)
    {
      length = snprintf(usb_message, sizeof(usb_message),
                        "AI_RESULT window=%lu class=1.%d inference_ms=%lu\r\n",
                        (unsigned long)sampling_stats.completed_windows,
                        ai_last_prediction + 1,
                        (unsigned long)ai_inference_ms);
    }
    else
    {
      length = snprintf(usb_message, sizeof(usb_message),
                        "AI_ERROR window=%lu inference_ms=%lu\r\n",
                        (unsigned long)sampling_stats.completed_windows,
                        (unsigned long)ai_inference_ms);
    }

    if ((length > 0) && ((size_t)length < sizeof(usb_message)))
    {
      (void)USB_SendText(usb_message, USB_TX_TIMEOUT_MS);
    }
  }
}

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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  sensor_status = LIS3DSH_Init(&hspi1);
  if (sensor_status != LIS3DSH_STATUS_OK)
  {
    Error_Handler();
  }

  Sampling_Init();

  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Sampling_Process();

    ready_window = Sampling_GetReadyWindow();
    if (ready_window != NULL)
    {
      /* Sampling is paused while the reference DFT and USB transfer run. */
      if (HAL_TIM_Base_Stop_IT(&htim2) != HAL_OK)
      {
        Error_Handler();
      }

      uint32_t ai_started = HAL_GetTick();
      ai_last_prediction = AI_RunWindow(ready_window);
      ai_inference_ms = (uint32_t)(HAL_GetTick() - ai_started);
      ai_last_status = (ai_last_prediction >= 0) ? 0 : -1;

      Report_Window();

      Sampling_ReleaseWindow();

      if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
      {
        Error_Handler();
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration (HSE 8 MHz -> PLL -> SYSCLK 168 MHz,
  *        48 MHz for USB OTG FS).
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function (LIS3DSH, SPI mode 0, 5.25 MHz)
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function (84 MHz / 84 / 1250 = 800 Hz tick)
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1249;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level (LIS3DSH CS idles high) */
  HAL_GPIO_WritePin(LIS3DSH_CS_GPIO_Port, LIS3DSH_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LIS3DSH_CS_Pin */
  GPIO_InitStruct.Pin = LIS3DSH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LIS3DSH_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  TIM2 update callback: 800 Hz sampling tick.
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    Sampling_TimerTickISR();
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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
