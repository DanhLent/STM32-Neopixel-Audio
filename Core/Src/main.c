/* USER CODE BEGIN Header */
/**
  **************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  **************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "ili9341.h"
#include "fonts.h"
#include "colors.h"
#include <stdbool.h>
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Dinh nghia cac trang thai (che do) hoat dong chinh cua chuong trinh.
// Giup quan ly chuong trinh theo kieu "May trang thai" (State Machine).
typedef enum {
    STATE_MENU,                 // Trang thai dang hien thi menu lua chon.
    STATE_EFFECT_SOUND_SYNC,    // Trang thai dang chay hieu ung dong bo am thanh.
    STATE_EFFECT_RAINBOW        // Trang thai dang chay hieu ung cau vong.
} ProgramState;                 // Dat ten cho kieu du lieu moi la "ProgramState".

// Dinh nghia cac ket qua tra ve khi doc trang thai cua rotary encoder.
// Giup code de doc hon thay vi tra ve cac con so 0, 1, 2.
typedef enum {
    ENCODER_NO_ROTATION,        // Encoder khong xoay.
    ENCODER_CW_ROTATION,        // Encoder xoay theo chieu kim dong ho (Clockwise).
    ENCODER_CCW_ROTATION        // Encoder xoay nguoc chieu kim dong ho (Counter-Clockwise).
} Encoder_Rotation;             // Dat ten cho kieu du lieu moi la "Encoder_Rotation".

// Dinh nghia mot cau truc (struct) de luu tru gia tri cua mot mau RGB.
// Giup gom 3 gia tri mau vao mot bien duy nhat, de dang truyen qua lai giua cac ham.
typedef struct {
    uint8_t r;  // Luu gia tri kenh Mau Do (Red), tu 0-255.
    uint8_t g;  // Luu gia tri kenh Xanh la (Green), tu 0-255.
    uint8_t b;  // Luu gia tri kenh Xanh duong (Blue), tu 0-255.
} RGB_t;        // Dat ten cho kieu du lieu moi la "RGB_t".
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_LED 50
// Cong tac de bat (1) hoac tat (0) chuc nang dieu chinh do sang phuc tap.
#define USE_BRIGHTNESS 1
// So PI
#ifndef PI
#define PI 3.14159265358979323846f

// Thong so cho viec xu ly am thanh
// Tan so lay mau am thanh (don vi: Hz). Quyet dinh so lan ADC se doc gia tri trong mot giay.
#define SAMPLING_RATE 8000.0f
// Kich thuoc cua moi khoi du lieu am thanh duoc xu ly trong mot lan.
// Chuong trinh se doi DMA thu thap du 256 mau roi moi bat dau phan tich.
#define BLOCK_SIZE 256
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim1_ch1;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
//-------------------------------------------------
// Cac bien trang thai toan cuc
//-------------------------------------------------

// Luu trang thai hoat dong hien tai cua chuong trinh (o Menu hay dang chay hieu ung).
ProgramState currentState = STATE_MENU;

// Luu vi tri hieu ung dang duoc chon trong menu (0 hoac 1).
int selectedEffect = 0;


//-------------------------------------------------
// Buffer va co (flag) cho viec lay mau am thanh
//-------------------------------------------------

// Bo dem de luu du lieu tho tu ADC. Kich thuoc gap doi de ap dung ky thuat double-buffering:
// trong khi CPU xu ly nua dau, DMA se ghi du lieu vao nua sau va nguoc lai.
uint16_t adc_buffer[BLOCK_SIZE * 2];

// Co bao hieu nua buffer dau tien da co du lieu moi va san sang de xu ly.
// 'volatile' de dam bao CPU luon doc gia tri moi nhat cua bien tu RAM.
volatile bool first_half_ready = false;

// Co bao hieu nua buffer thu hai da co du lieu moi va san sang de xu ly.
volatile bool second_half_ready = false;


//-------------------------------------------------
// Cac bien cho viec dieu khien dai LED WS2812B
//-------------------------------------------------

// Mang luu tru gia tri mau goc (RGB) cho tung den LED.
uint8_t LED_Data[MAX_LED][4];

// Mang luu tru mau sau khi da dieu chinh do sang (neu dung).
uint8_t LED_Mod[MAX_LED][4];

// Buffer chua du lieu xung PWM da duoc ma hoa, san sang de DMA gui di.
uint16_t pwmData[(24*MAX_LED)+50];

// Co bao hieu qua trinh gui du lieu toi LED qua DMA-Timer da hoan tat.
volatile int datasentflag = 0;

// Bien dem buoc de tao chuyen dong cho hieu ung cau vong.
uint16_t rainbow_effStep = 0;

// Luu gia tri am luong (RMS) duoc tinh toan tu tin hieu am thanh.
uint16_t volume_rms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_FSMC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
//--- Cac ham lien quan den Giao dien Nguoi dung (UI) ---

// Ve giao dien menu chinh len man hinh LCD.
void drawMenu(int selection);

// Ve man hinh thong bao khi dang chay mot hieu ung.
void drawEffectScreen(const char* effectName);

// Kiem tra trang thai xoay cua Rotary Encoder.
Encoder_Rotation checkRotaryEncoder(void);

// Doc trang thai nut nhan (SW) cua Encoder, co chong doi va chong giu phim.
uint8_t readSwitch(void);


//--- Cac ham thuc thi hieu ung chinh ---

// Goi logic de chay hieu ung nhay theo am thanh.
void run_sound_sync_effect(void);

// Goi logic de chay hieu ung cau vong.
void run_rainbow_effect(void);


//--- Cac ham dieu khien LED cap thap ---

// Dat gia tri mau (R, G, B) cho mot den LED duy nhat trong bo dem.
void Set_LED(int LEDnum, int Red, int Green, int Blue);

// Dieu chinh do sang tong the cua dai LED.
void Set_Brightness(int brightness);

// Gui du lieu mau sac tu bo dem ra dai LED thong qua TIM+DMA.
void WS2812_Send(void);


//--- Ham xu ly am thanh ---

// Tinh toan gia tri am luong RMS (Root Mean Square) tu mot khoi du lieu am thanh.
uint16_t calculate_rms(uint16_t* buffer, uint16_t num_samples, uint16_t dc_offset);
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
  MX_DMA_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_FSMC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // Khoi tao LCD
  lcdInit();
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);

  // Bat dau cac tien trinh nen
  HAL_TIM_Base_Start(&htim2); // Bat dau Timer de kich ADC
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, BLOCK_SIZE * 2); // Bat dau ADC lay mau qua DMA

  // Ve giao dien menu ban dau
  drawMenu(selectedEffect);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	    // Doc trang thai tu cac thiet bi dau vao mot lan o dau vong lap
	    Encoder_Rotation rotation = checkRotaryEncoder();
	    uint8_t switch_pressed = readSwitch();

	    // Xu ly logic dua tren trang thai hien tai cua chuong trinh
	    switch(currentState)
	    {
	        // TRUONG HOP 1: Dang o man hinh Menu
	        case STATE_MENU:
	            // Neu nguoi dung xoay encoder...
	            if (rotation != ENCODER_NO_ROTATION)
	            {
	                // ...thay doi hieu ung duoc chon (chuyen qua lai giua 0 va 1).
	                selectedEffect = (selectedEffect + 1) % 2;
	                // Ve lai menu de cap nhat giao dien, lam noi bat lua chon moi.
	                drawMenu(selectedEffect);
	            }
	            // Neu nguoi dung nhan nut SW...
	            if (switch_pressed)
	            {
	                // Kiem tra xem hieu ung nao dang duoc chon
	                if (selectedEffect == 0) {
	                    // Chuyen sang trang thai chay hieu ung "SOUND SYNC"
	                    currentState = STATE_EFFECT_SOUND_SYNC;
	                    // Hien thi man hinh thong bao tuong ung
	                    drawEffectScreen("SOUND SYNC");
	                } else {
	                    // Chuyen sang trang thai chay hieu ung "RAINBOW"
	                    currentState = STATE_EFFECT_RAINBOW;
	                    // Hien thi man hinh thong bao
	                    drawEffectScreen("RAINBOW");
	                    // Reset lai bien dem de hieu ung cau vong bat dau tu dau
	                    rainbow_effStep = 0;
	                }
	            }
	            break;

	        // TRUONG HOP 2: Dang chay hieu ung "Sound Sync"
	        case STATE_EFFECT_SOUND_SYNC:
	            // Lien tuc goi ham de chay logic hieu ung
	            run_sound_sync_effect();
	            // Neu nguoi dung nhan nut SW trong khi hieu ung dang chay...
	            if (switch_pressed) {
	                 // ...chuyen trang thai ve lai Menu
	                 currentState = STATE_MENU;
	                 // Tat het den LED truoc khi ve menu de tranh de lai hieu ung cu
	                 for(int i=0; i<MAX_LED; i++) Set_LED(i,0,0,0);
	                 WS2812_Send();
	                 // Ve lai giao dien menu
	                 drawMenu(selectedEffect);
	            }
	            break;

	        // TRUONG HOP 3: Dang chay hieu ung "Rainbow"
	        case STATE_EFFECT_RAINBOW:
	            // Lien tuc goi ham de chay logic hieu ung
	            run_rainbow_effect();
	            // Neu nguoi dung nhan nut SW trong khi hieu ung dang chay...
	            if (switch_pressed) {
	                // ...chuyen trang thai ve lai Menu
	                currentState = STATE_MENU;
	                // Tat het den LED
	                for(int i=0; i<MAX_LED; i++) Set_LED(i,0,0,0);
	                WS2812_Send();
	                // Ve lai giao dien menu
	                drawMenu(selectedEffect);
	            }
	            break;
	    }
	    // Khi o man hinh menu, them mot do tre nho de giam tai cho CPU
	    if (currentState == STATE_MENU) HAL_Delay(10);
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
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 90-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
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
  htim2.Init.Period = 124;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ENC_CLK_Pin ENC_DT_Pin ENC_SW_Pin */
  GPIO_InitStruct.Pin = ENC_CLK_Pin|ENC_DT_Pin|ENC_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_BL_Pin */
  GPIO_InitStruct.Pin = LCD_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_BL_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  // Quan trọng: Các chân FSMC phải được cấu hình ở đây bởi CubeMX
  /* USER CODE END MX_GPIO_Init_2 */
}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */
  // Quan trọng: Hàm này phải được tạo bởi CubeMX để cấu hình đúng timing
  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */
  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM1 memory initialization sequence
  */
  hsram1.Instance = FSMC_NORSRAM_DEVICE;
  hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram1.Init */
  hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
  hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram1.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
  hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 6;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 6;
  Timing.BusTurnAroundDuration = 0;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */
  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Dat co bao hieu nua buffer dau tien da san sang.
    first_half_ready = true;

    // Lenh khong lam gi, thuong dung de dat breakpoint khi debug.
    __NOP();
}




void drawMenu(int selection)
{
    // Xoa sach man hinh ve mau den de chuan bi ve menu moi.
    lcdFillRGB(COLOR_BLACK);

    //--- Ve phan Tieu De ---
    lcdSetTextFont(&Font24); // Chon font lon de ve tieu de.
    lcdSetTextColor(COLOR_WHITE, COLOR_BLACK); // Dat mau chu la mau trang, mau nen la mau den.
    lcdSetCursor(10, 20); // Di chuyen con tro den vi tri de viet tieu de.
    lcdPrintf("CHON HIEU UNG"); // In chuoi tieu de ra man hinh.

    //--- Ve Nut lua chon so 1 ---

    // Kiem tra xem nut 1 co dang duoc chon hay khong de quyet dinh mau sac.
    // Neu selection == 0, nut se co mau sac noi bat (vang, xanh cyan).
    // Neu khong, nut se co mau sac thong thuong (trang, xanh navy).
    uint16_t b1_bg = (selection == 0) ? COLOR_DARKCYAN : COLOR_NAVY;       // Mau nen cua nut
    uint16_t b1_brd = (selection == 0) ? COLOR_YELLOW : COLOR_WHITE;      // Mau duong vien cua nut
    uint16_t b1_txt = (selection == 0) ? COLOR_YELLOW : COLOR_WHITE;      // Mau chu ben trong nut

    // Thuc hien ve nut 1
    lcdFillRect(30, 80, 180, 60, b1_bg);     // Ve hinh chu nhat dac lam nen cho nut bam.
    lcdDrawRect(30, 80, 180, 60, b1_brd);    // Ve duong vien cho nut bam.
    lcdSetTextColor(b1_txt, b1_bg);         // Thiet lap mau chu va mau nen (trung voi nen nut).
    lcdSetTextFont(&Font16);                // Chon font chu co kich thuoc nho hon cho nut bam.
    lcdSetCursor(43, 102);                  // Dat con tro vao giua nut de can chinh chu.
    lcdPrintf("NHAY THEO NHAC");            // In ten cua hieu ung.

    //--- Ve Nut lua chon so 2 (Logic tuong tu nut 1) ---

    // Kiem tra xem nut 2 co dang duoc chon hay khong (selection == 1).
    uint16_t b2_bg = (selection == 1) ? COLOR_DARKCYAN : COLOR_NAVY;
    uint16_t b2_brd = (selection == 1) ? COLOR_YELLOW : COLOR_WHITE;
    uint16_t b2_txt = (selection == 1) ? COLOR_YELLOW : COLOR_WHITE;

    // Thuc hien ve nut 2
    lcdFillRect(30, 160, 180, 60, b2_bg);
    lcdDrawRect(30, 160, 180, 60, b2_brd);
    lcdSetTextColor(b2_txt, b2_bg);
    lcdSetTextFont(&Font16);
    lcdSetCursor(76, 182);
    lcdPrintf("CAU VONG");
}





void drawEffectScreen(const char* effectName)
{
    lcdFillRGB(COLOR_BLACK);
    lcdSetTextFont(&Font16);
    lcdSetTextColor(COLOR_GREEN, COLOR_BLACK);
    lcdSetCursor(10, 50);
    lcdPrintf("Dang chay hieu ung:");
    lcdSetTextFont(&Font20);
    lcdSetTextColor(COLOR_WHITE, COLOR_BLACK);
    lcdSetCursor(10, 100);
    lcdPrintf(effectName);
    lcdSetTextFont(&Font12);
    lcdSetTextColor(COLOR_RED, COLOR_BLACK);
    lcdSetCursor(10, 260);
    lcdPrintf("Nhan SW de quay lai Menu");
}





/**
  * @brief  Kiem tra trang thai xoay cua Rotary Encoder.
  * @note   Ham nay su dung phuong phap phat hien canh xuong cua tin hieu CLK,
  * sau do kiem tra trang thai cua tin hieu DT de xac dinh chieu xoay.
  * @retval Trang thai xoay (CW, CCW, hoac khong xoay).
  */
Encoder_Rotation checkRotaryEncoder(void)
{
    // Bien static de luu trang thai cua chan CLK o lan goi ham truoc.
    // 'static' giup bien nay giu nguyen gia tri giua cac lan goi ham.
    // Khoi tao la 1 vi GPIO duoc cau hinh pull-up (muc cao khi khong hoat dong).
    static uint8_t last_clk_state = 1;

    // Doc trang thai hien tai cua chan CLK.
    uint8_t current_clk_state = HAL_GPIO_ReadPin(ENC_CLK_GPIO_Port, ENC_CLK_Pin);

    // Chi xu ly khi co su thay doi trang thai tu cao (1) xuong thap (0).
    // Day chinh la thoi diem encoder di qua mot "khac".
    if (current_clk_state == 0 && last_clk_state == 1)
    {
        // Them mot do tre rat nho de chong nhieu tin hieu.
        HAL_Delay(2);

        // Tai thoi diem CLK vua chuyen xuong thap, ta doc gia tri cua chan DT.
        // Chinh gia tri cua DT se quyet dinh chieu xoay.
        if (HAL_GPIO_ReadPin(ENC_DT_GPIO_Port, ENC_DT_Pin) == 0) {
            // Neu DT cung o muc thap, nghia la encoder dang xoay theo chieu kim dong ho.
            last_clk_state = current_clk_state; // Cap nhat trang thai truoc khi thoat.
            return ENCODER_CW_ROTATION;
        } else {
            // Nguoc lai, neu DT o muc cao, nghia la encoder xoay nguoc chieu kim dong ho.
            last_clk_state = current_clk_state;
            return ENCODER_CCW_ROTATION;
        }
    }

    // Cap nhat lai trang thai cua CLK de chuan bi cho lan kiem tra tiep theo.
    last_clk_state = current_clk_state;

    // Neu khong phat hien duoc canh xuong, tra ve trang thai khong xoay.
    return ENCODER_NO_ROTATION;
}





uint8_t readSwitch(void)
{
    // Kiem tra xem nut co duoc nhan xuong hay khong (chan SW duoc noi xuong GND -> muc thap).
    if (HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin) == GPIO_PIN_RESET)
    {
        // Cho mot khoang thoi gian ngan de chong nhieu/doi (debounce).
        // Neu sau 200ms ma nut van duoc nhan, ta coi day la mot lan nhan hop le.
        HAL_Delay(200);

        if (HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin) == GPIO_PIN_RESET)
        {
            // Vong lap nay se bi "treo" o day cho den khi nguoi dung tha nut ra.
            // Dieu nay dam bao chuong trinh chi ghi nhan 1 lan duy nhat cho moi lan nhan.
            while(HAL_GPIO_ReadPin(ENC_SW_GPIO_Port, ENC_SW_Pin) == GPIO_PIN_RESET);

            // Khi nut da duoc tha ra, tra ve 1 de bao hieu mot lan nhan hop le da hoan tat.
            return 1;
        }
    }

    // Neu khong co nut nao duoc nhan, tra ve 0.
    return 0;
}





/**
  * @brief  Chay hieu ung cau vong lien tuc, khong phu thuoc vao am thanh.
  * @note   Su dung thuat toan pha tron mau tuyen tinh giua 3 mau co ban (G->B->R)
  * de tao ra mot dai mau muot ma.
  * @param  None
  * @retval None
  */
void run_rainbow_effect(void)
{
    float factor1, factor2; // He so pha tron mau
    uint16_t ind;          // Vi tri ao cua den LED trong chu ky hieu ung
    uint8_t r, g, b;       // Cac bien luu gia tri mau RGB tam thoi

    // Cac hang so xac dinh cach hieu ung cau vong duoc tao ra.
    // Chung duoc dich tu code Arduino mau.
    const float step_increment = 2.266667f; // Buoc nhay cho moi den LED ke tiep
    const int total_steps = 68;             // Tong so buoc trong mot chu ky hoan chinh cua hieu ung
    const float segment_length = 22.666667f; // Do dai cua moi doan chuyen mau (total_steps / 3)

    // Duyet qua tung den LED de tinh toan mau sac cho no.
    for(uint16_t j = 0; j < MAX_LED; j++) {
        // Tinh toan vi tri "ao" cua den LED hien tai trong toan bo chu ky hieu ung.
        // 'rainbow_effStep' lam cho toan bo dai mau di chuyen qua moi frame.
        ind = rainbow_effStep + (uint16_t)(j * step_increment);

        // Xac dinh xem den LED dang o doan chuyen mau nao (0, 1, hay 2).
        int segment = (int)((ind % total_steps) / segment_length);

        switch(segment) {
            case 0: // Chuyen tu Xanh la (G) sang Xanh duong (B)
                // factor1 giam tu 1 ve 0, factor2 tang tu 0 len 1.
                factor1 = 1.0f - ((float)(ind % total_steps) / segment_length);
                factor2 = ((float)(ind % total_steps)) / segment_length;
                r = 0;
                g = (uint8_t)(255 * factor1); // Kenh Green giam dan
                b = (uint8_t)(255 * factor2); // Kenh Blue tang dan
                break;

            case 1: // Chuyen tu Xanh duong (B) sang Do (R)
                factor1 = 1.0f - ((float)(ind % total_steps - segment_length) / segment_length);
                factor2 = ((float)(ind % total_steps - segment_length)) / segment_length;
                r = (uint8_t)(255 * factor2); // Kenh Red tang dan
                g = 0;
                b = (uint8_t)(255 * factor1); // Kenh Blue giam dan
                break;

            case 2: // Chuyen tu Do (R) ve Xanh la (G)
                factor1 = 1.0f - ((float)(ind % total_steps - 2 * segment_length) / segment_length);
                factor2 = ((float)(ind % total_steps - 2 * segment_length)) / segment_length;
                r = (uint8_t)(255 * factor1); // Kenh Red giam dan
                g = (uint8_t)(255 * factor2); // Kenh Green tang dan
                b = 0;
                break;
            default: // Truong hop du phong
                r=0; g=0; b=0;
                break;
        }
        // Luu mau vua tinh toan vao bo dem cho den LED thu 'j'.
        Set_LED(j, r, g, b);
    }

    // Ap dung do sang da chon.
    Set_Brightness(10);

    // Gui toan bo du lieu trong bo dem ra dai LED thuc te.
    WS2812_Send();

    // Tang buoc hieu ung len 1 de frame tiep theo, dai mau se dich chuyen.
    rainbow_effStep++;

    // Neu hieu ung da chay xong mot chu ky, bat dau lai tu dau.
    if(rainbow_effStep >= total_steps) {
        rainbow_effStep = 0;
    }

    // Dieu khien toc do cua hieu ung. So cang nho, hieu ung cang nhanh.
    //HAL_Delay(7);
}





// Ham callback cua DMA-Timer -> da gui xong
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag=1;
}





// Ham luu gia tri mau cho LED
void Set_LED (int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}





/**
  * @brief  Dieu chinh do sang tong the cua toan bo dai LED.
  * @note   Ham nay khong thay doi mau goc trong LED_Data.
  * Thay vao do, no tinh toan gia tri mau moi voi do sang da duoc ap dung
  * va luu ket qua vao mang LED_Mod.
  * @param  brightness: Gia tri do sang mong muon, tu 0 (toi nhat) den 45 (sang nhat).
  * @retval None
  */
void Set_Brightness (int brightness)
{
    // Toan bo chuc nang nay chi duoc bien dich neu USE_BRIGHTNESS duoc bat (bang 1).
    #if USE_BRIGHTNESS

	// Gioi han gia tri do sang trong khoang an toan tu 0 den 45.
	if (brightness > 45) brightness = 45;
	if (brightness < 0) brightness = 0;

	// Duyet qua tung den LED de ap dung do sang.
	for (int i=0; i<MAX_LED; i++)
	{
		// Sao chep du lieu khong phai mau (vi du: chi so cua den).
		LED_Mod[i][0] = LED_Data[i][0];

		// Voi moi den, xu ly 3 thanh phan mau (G, R, B).
		for (int j=1; j<4; j++)
		{
			// Truong hop dac biet: neu do sang la 0, tat luon den.
			if(brightness == 0) {
				LED_Mod[i][j] = 0;
			} else {
				// Su dung cong thuc luong giac de dieu chinh do sang mot cach phi tuyen tinh.
				float angle = 90 - brightness;
				angle = angle * PI / 180; // Chuyen doi tu do sang radian

				// Khi brightness = 45 (max), angle = 45, tan(45) = 1 -> mau giu nguyen.
				// Khi brightness giam, angle tang -> tan(angle) tang -> mau toi di.
				if (tan(angle) != 0) {
					LED_Mod[i][j] = (LED_Data[i][j]) / (tan(angle));
				} else {
					// Phong truong hop tan(angle) = 0 (khong xay ra voi input da gioi han).
					LED_Mod[i][j] = LED_Data[i][j];
				}
			}
		}
	}
    #endif
}






/**
  * @brief  Chuyen doi du lieu mau trong mang LED_Data thanh chuoi xung PWM va gui ra LED.
  * @note   Su dung ket hop Timer o che do PWM va bo dieu khien DMA de tu dong hoa viec gui tin hieu.
  * @param  None
  * @retval None
  */
void WS2812_Send (void)
{
	uint32_t indx=0;      // Bien dem cho mang pwmData
	uint32_t color;       // Bien tam de luu gia tri mau 24-bit cua mot LED

	// Duyet qua tung den LED trong dai de chuan bi du lieu
	for (int i= 0; i < MAX_LED; i++)
	{
		// Lay du lieu mau (theo thu tu G-R-B) tu mang LED_Data va ghep lai thanh mot so 24-bit.
		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));

		// Voi moi den,
		// duyet qua tung bit trong 24 bit mau (tu bit cao nhat '23' den bit thap nhat '0').
		for (int j=23; j>=0; j--)
		{
			// Kiem tra gia tri cua bit thu j.
			if (color & (1UL << j))
			{
				// Neu bit la 1, dat gia tri duty cycle cao (vi du: 60/90).
				// Xung HIGH se dai, duoc WS2812B hieu la bit '1'.
				pwmData[indx] = 60;
			}
			else
			{
				// Neu bit la 0, dat gia tri duty cycle thap (vi du: 30/90).
				// Xung HIGH se ngan, duoc WS2812B hieu la bit '0'.
				pwmData[indx] = 30;
			}
			indx++;
		}
	}

	// Them 50 gia tri '0' vao cuoi buffer de tao ra tin hieu Reset (Latch).
	// Xung LOW keo dai nay se bao cho cac den LED "chot" mau va hien thi.
	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	// Bat dau qua trinh phat chuoi xung PWM bang Timer va DMA.
	// DMA se tu dong nap cac gia tri tu 'pwmData' vao thanh ghi cua Timer.
	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);

	// Cho cho den khi DMA gui xong toan bo du lieu.
	// Co 'datasentflag' se duoc bat len 1 trong ham callback ngat cua Timer.
	while (!datasentflag){};

	// Reset co de chuan bi cho lan gui tiep theo.
	datasentflag = 0;
}








/**
  * @brief  Chuyen doi mau sac tu khong gian mau HSV (Hue, Saturation, Value) sang RGB.
  * @note   Su dung phep toan so nguyen de toi uu hoa toc do tren vi dieu khien.
  * @param  h: Hue (sac mau), gia tri tu 0 - 359, dai dien cho vong tron mau.
  * @param  s: Saturation (do bao hoa), gia tri tu 0 (mau xam) den 255 (mau dam nhat).
  * @param  v: Value (do sang), gia tri tu 0 (mau toi) den 255 (mau sang nhat).
  * @retval Cau truc RGB_t chua 3 gia tri mau Red, Green, Blue tuong ung.
  */
RGB_t HSV_to_RGB_integer(uint16_t h, uint8_t s, uint8_t v) {
    RGB_t rgb;
    uint8_t region, remainder, p, q, t;

    // Truong hop dac biet: neu do bao hoa (s) bang 0, mau la mau xam.
    // Tat ca cac kenh R, G, B se co cung gia tri la do sang (v).
    if (s == 0) {
        rgb.r = v;
        rgb.g = v;
        rgb.b = v;
        return rgb;
    }

    // Tinh toan khu vuc (region) cua mau tren vong tron mau (chia lam 6 khu vuc, moi khu 60 do).
    region = h / 60;

    // Tinh toan phan du, cho biet vi tri cua mau trong khu vuc do.
    // Scale gia tri nay ra thang 0-255 de tinh toan.
    remainder = (h % 60) * 255 / 60;

    // Tinh toan cac thanh phan mau trung gian p, q, t.
    // p: Gia tri thap nhat cua 3 kenh R,G,B (khi mau bi giam do bao hoa).
    // q, t: Cac gia tri de tao ra su chuyen tiep muot ma giua cac mau.
    // Phep dich bit '>> 8' tuong duong voi phep chia cho 256, la mot cach toi uu hoa.
    p = (v * (255 - s)) >> 8;
    q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    // Gan gia tri R, G, B tuy thuoc vao khu vuc mau (region).
    switch (region) {
        case 0: // Do -> Vang
            rgb.r = v; rgb.g = t; rgb.b = p;
            break;
        case 1: // Vang -> Xanh la
            rgb.r = q; rgb.g = v; rgb.b = p;
            break;
        case 2: // Xanh la -> Xanh cyan
            rgb.r = p; rgb.g = v; rgb.b = t;
            break;
        case 3: // Xanh cyan -> Xanh duong
            rgb.r = p; rgb.g = q; rgb.b = v;
            break;
        case 4: // Xanh duong -> Tim
            rgb.r = t; rgb.g = p; rgb.b = v;
            break;
        default: // Tim -> Do
            rgb.r = v; rgb.g = p; rgb.b = q;
            break;
    }

    return rgb;
}





/**
  * @brief  Tinh gia tri RMS (Root Mean Square) de do am luong chinh xac.
  * @note   RMS la mot phep do tot hon so voi viec chi lay bien do dinh-dinh
  * de the hien do lon ma tai nguoi cam nhan duoc.
  * @param  buffer: Con tro den bo dem chua cac mau am thanh.
  * @param  num_samples: So luong mau trong bo dem can xu ly.
  * @param  dc_offset: Gia tri DC offset cua tin hieu (thuong la mot nua gia tri ADC max).
  * @retval Gia tri am luong RMS (dang so nguyen).
  */
uint16_t calculate_rms(uint16_t* buffer, uint16_t num_samples, uint16_t dc_offset) {
    // Su dung kieu 'uint32_t' de tranh bi tran so khi cong don cac gia tri lon.
    uint32_t sum_of_squares = 0;

    // Duyet qua tung mau trong khoi du lieu duoc cung cap.
    for (uint16_t i = 0; i < num_samples; i++) {
        // Buoc 1: Loai bo thanh phan DC de chi lay tin hieu xoay chieu (AC).
        // Tin hieu AC co the co gia tri am, nen dung kieu 'int32_t'.
        int32_t ac_signal = buffer[i] - dc_offset;

        // Buoc 2 (Square): Binh phuong gia tri AC (de khu gia tri am)
        // va cong don vao tong (Sum).
        sum_of_squares += ac_signal * ac_signal;
    }

    // Buoc 3 (Mean): Tinh gia tri trung binh cong cua tong binh phuong.
    // Buoc 4 (Root): Lay can bac hai cua gia tri trung binh do.
    // Ham sqrtf() la ham tinh can bac hai cho so thuc (float).
    return sqrtf((float)sum_of_squares / num_samples);
}






void run_sound_sync_effect(void)
{
    uint16_t* processing_buffer = NULL;

    if (first_half_ready) {
        processing_buffer = &adc_buffer[0];
        first_half_ready = false;
    } else if (second_half_ready) {
        processing_buffer = &adc_buffer[BLOCK_SIZE];
        second_half_ready = false;
    }

    if (processing_buffer != NULL)
    {
    	// TINH TOAN DC OFFSET THUC TE
    	uint32_t sum = 0;
    	for(int i=0; i<BLOCK_SIZE; i++) {
    	    sum += processing_buffer[i];
    	}
    	uint16_t dc_offset_actual = sum / BLOCK_SIZE;
        // PHAN TICH AM THANH
    	volume_rms = calculate_rms(processing_buffer, BLOCK_SIZE, dc_offset_actual);

        // --- 2. ANH XA AM THANH SANG HINH ANH ---
        // Anh xa am luong RMS ra so luong den LED sang
        int leds_to_light = (volume_rms - 10) * MAX_LED / (500 - 10); // Vi du nguong moi
        if (leds_to_light < 0) leds_to_light = 0;
        if (leds_to_light > MAX_LED) leds_to_light = MAX_LED;

        // Anh xa am luong ra mau sac (Hue tu 240-Xanh toi 0-Do)
        uint16_t hue = 240 - ((uint32_t)leds_to_light * 240) / MAX_LED;

        // VE HIEU UNG
        for (int i = 0; i < MAX_LED; i++) {
            if (i < leds_to_light) {
                // Tat ca cac den sang se co cung mot mau nong/lanh tuy thuoc vao so luong den
                RGB_t color = HSV_to_RGB_integer(hue, 255, 200); // S=255, V=200
                Set_LED(i, color.r, color.g, color.b);
            } else {
                Set_LED(i, 0, 0, 0); // Tat cac den con lai
            }
        }
        // Gui du lieu toi LED
        WS2812_Send();
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
