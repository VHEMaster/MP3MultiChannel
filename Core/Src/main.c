#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "mp3common.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>

typedef StaticTask_t osStaticThreadDef_t;

IWDG_HandleTypeDef hiwdg;

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai2_a;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

osThreadId_t taskCommHandle;
uint32_t taskCommBuffer[256];
osStaticThreadDef_t taskCommControlBlock;
const osThreadAttr_t taskComm_attributes = {
  .name = "taskComm",
  .cb_mem = &taskCommControlBlock,
  .cb_size = sizeof(taskCommControlBlock),
  .stack_mem = &taskCommBuffer[0],
  .stack_size = sizeof(taskCommBuffer),
  .priority = (osPriority_t) osPriorityLow,
};

osThreadAttr_t taskPlayer_attributes = {
  .name = "taskPlayer",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osMutexAttr_t mutexPlayer_attributes = {
    .name= "mutexPlayer",
    .attr_bits = osMutexRecursive,
    .cb_mem = NULL,
    .cb_size = 0
};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SAI2_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM5_Init(void);
void StartCommTask(void *argument);
void StartPlayerTask(void *argument);

#define PLAYERS_COUNT       (8)
#define MP3_SAMPLE_SIZE     (1152 * 2)
#define SAMPLE_BUFFER_SIZE  ((MP3_SAMPLE_SIZE) * 2)
#define FILE_BUFFER_SIZE    (4096)
#define MAX_FILE_PATH       (128)

uint16_t gSampleBuffer[PLAYERS_COUNT][MP3_SAMPLE_SIZE];
uint32_t gSamplesBuffer[PLAYERS_COUNT][SAMPLE_BUFFER_SIZE];
uint8_t gFileBuffer[PLAYERS_COUNT][FILE_BUFFER_SIZE];

#define TDM_BUFFER_SIZE 1152
uint32_t gTdmFinalBuffer[TDM_BUFFER_SIZE] __attribute__((aligned(32)));

typedef struct {
    const char *name;
    char file[MAX_FILE_PATH];
    volatile uint8_t enabled;
    volatile uint8_t playing;
    volatile uint8_t changed;
    uint8_t volume;
    osMutexId_t mutex;
    osThreadId_t task;
    uint32_t bufferSize;
    uint32_t *buffer;
    uint8_t *fileBuffer;
    uint32_t fileBufferSize;
    uint16_t *sampleBuffer;
    uint32_t sampleBufferSize;
    uint32_t buffer_rd;
    uint32_t buffer_wr;
}sPlayerData;

typedef struct {
    uint32_t count;
    sPlayerData player[PLAYERS_COUNT];
}sPlayersData;

sPlayersData gPlayersData = { PLAYERS_COUNT };


static inline uint32_t getavail(uint32_t wr, uint32_t rd, uint32_t size) {
  if(wr >= rd) return (wr - rd);
  else return (size - rd + wr);
}

static inline uint32_t getfree(uint32_t wr, uint32_t rd, uint32_t size) {
  return size - getavail(wr, rd, size);
}

static void HandleSaiDma(uint32_t *buffer, uint32_t size)
{
  uint32_t samples_per_channel = size / PLAYERS_COUNT;

  for(int i = 0; i < PLAYERS_COUNT; i++)
  {
    if(gPlayersData.player[i].playing && getavail(gPlayersData.player[i].buffer_wr, gPlayersData.player[i].buffer_rd, gPlayersData.player[i].bufferSize) >= samples_per_channel)
    {
      for(int j = 0; j < samples_per_channel; j++)
      {
        gTdmFinalBuffer[j * PLAYERS_COUNT + i] = gPlayersData.player[i].buffer[gPlayersData.player[i].buffer_rd];
        if(gPlayersData.player[i].buffer_rd + 1 >= gPlayersData.player[i].bufferSize)
          gPlayersData.player[i].buffer_rd = 0;
        else gPlayersData.player[i].buffer_rd++;
      }
    } else {
      memset(buffer, 0, size * sizeof(*buffer));
    }
  }

  SCB_CleanDCache_by_Addr(buffer, size);
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  HandleSaiDma(&gTdmFinalBuffer[TDM_BUFFER_SIZE / 2], TDM_BUFFER_SIZE / 2);
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  HandleSaiDma(&gTdmFinalBuffer[0], TDM_BUFFER_SIZE / 2);
}


void StartCommTask(void *argument)
{
  sPlayersData *playersdata = (sPlayersData *)argument;
  char *str;

  FATFS fs = {0};
  FRESULT res;

  while(1) {
    res = f_mount(&fs, SDPath, 1);
    if(res == FR_OK)
      break;
    osDelay(500);
  }

  memset(gTdmFinalBuffer, 0, sizeof(gTdmFinalBuffer));

  for(int i = 0; i < PLAYERS_COUNT; i++) {
    memset(&playersdata->player[i], 0, sizeof(playersdata->player[i]));

    str = (char *)pvPortMalloc(12); sprintf(str, "player%d", i+1); playersdata->player[i].name = str;
    str = (char *)pvPortMalloc(16); sprintf(str, "mutexPlayer%d", i+1); mutexPlayer_attributes.name = str;
    str = (char *)pvPortMalloc(16); sprintf(str, "taskPlayer%d", i+1); taskPlayer_attributes.name = str;

    sprintf(playersdata->player[i].file, "music/%d.mp3", i+1);

    playersdata->player[i].enabled = 0;
    playersdata->player[i].playing = 0;
    playersdata->player[i].volume = 100;

    playersdata->player[i].bufferSize = SAMPLE_BUFFER_SIZE;
    playersdata->player[i].buffer = gSamplesBuffer[i];
    playersdata->player[i].sampleBufferSize = MP3_SAMPLE_SIZE;
    playersdata->player[i].sampleBuffer = gSampleBuffer[i];
    playersdata->player[i].fileBufferSize = FILE_BUFFER_SIZE;
    playersdata->player[i].fileBuffer = gFileBuffer[i];

    playersdata->player[i].mutex = osMutexNew(&mutexPlayer_attributes);
    playersdata->player[i].task = osThreadNew(StartPlayerTask, &playersdata->player[i], &taskPlayer_attributes);
  }

  HandleSaiDma(gTdmFinalBuffer, TDM_BUFFER_SIZE);
  HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)gTdmFinalBuffer, TDM_BUFFER_SIZE);

  for(;;)
  {
    osDelay(1);
  }
}

void StartPlayerTask(void *argument)
{
  sPlayerData *playerdata = (sPlayerData *)argument;
  HMP3Decoder mp3Handler = MP3InitDecoder();
  MP3FrameInfo mp3Info;


  int32_t syncword;
  uint32_t *buffer;
  int16_t *stereo;
  uint32_t buffersize;
  uint16_t *samplebuffer;
  uint8_t *filebuffer;
  uint32_t filebuffersize;
  uint8_t playing;
  uint8_t enabled;
  uint8_t changed;
  float volume;

  FIL file = {0};
  FRESULT res;
  UINT read;
  uint32_t toread;
  uint32_t filesize;

  uint8_t *tempfilebuffer;
  int16_t *mp3buffer;


  playing = 0;

  while(1)
  {
    osMutexAcquire(playerdata->mutex, portMAX_DELAY);
    filebuffer = playerdata->fileBuffer;
    filebuffersize = playerdata->fileBufferSize;
    samplebuffer = playerdata->sampleBuffer;
    buffer = playerdata->buffer;
    stereo = (int16_t *)buffer;
    buffersize = playerdata->bufferSize;
    volume = playerdata->volume * 0.01f;
    enabled = playerdata->enabled;
    changed = playerdata->changed;
    osMutexRelease(playerdata->mutex);

    if(changed) {
      osMutexAcquire(playerdata->mutex, portMAX_DELAY);
      playerdata->buffer_rd = 0;
      playerdata->buffer_wr = 0;
      playerdata->changed = 0;
      if(playing) {
        playing = 0;
        osMutexRelease(playerdata->mutex);
        memset(buffer, 0, buffersize * sizeof(*buffer));
        res = f_close(&file);
        if(res != FR_OK) {

        }
      } else {
        osMutexRelease(playerdata->mutex);
      }

      if(enabled) {
        res = f_open(&file, playerdata->file, FA_READ);
        if(res != FR_OK) {
          playerdata->enabled = 0;
          enabled = 0;
          playing = 0;
        } else {
          playing = 1;
          filesize = f_size(&file);
        }

      }
    }

    playerdata->playing = playing;

    if(playing) {

      if(f_eof(&file)) {
        f_lseek(&file, 0);
        filesize = f_size(&file);
        playerdata->playing = 0;
      }

      toread = filebuffersize;
      if(toread > filesize)
        toread = filesize;

      res = f_read(&file, filebuffer, toread, &read);
      if(res != FR_OK) {
        continue;
      }

      syncword = MP3FindSyncWord(filebuffer, read);
      if(syncword <= ERR_MP3_INDATA_UNDERFLOW)
      {
        filesize -= read;
        continue;
      }
      if(syncword > 0)
      {
        f_lseek(&file, f_tell(&file) - read + syncword);
        filesize -= syncword;
        continue;
      }

      syncword = MP3GetNextFrameInfo(mp3Handler, &mp3Info, filebuffer);
      if(syncword != ERR_MP3_NONE)
      {
        if(filesize > 2)
        {
          f_lseek(&file, f_tell(&file) - read + 2);
          filesize -= 2;
        }
        continue;
      }

      tempfilebuffer = filebuffer;
      mp3buffer = (int16_t *)samplebuffer;
      syncword = MP3Decode(mp3Handler, &tempfilebuffer, (int*)&filesize, mp3buffer, 0);

      if(syncword != ERR_MP3_NONE)
      {
        filesize -= read;
        continue;
      }

      MP3GetLastFrameInfo(mp3Handler, &mp3Info);

      if(mp3Info.samprate == 0 || mp3Info.nChans <= 0) {
        filesize -= read;
        continue;
      }

      if(volume < 1.0f) {
        for(int i = 0; i < mp3Info.outputSamps; i++) {
          mp3buffer[i] = (float)mp3buffer[i] * volume;
        }
      }

      while(getfree(playerdata->buffer_wr, playerdata->buffer_rd, playerdata->bufferSize) <= mp3Info.outputSamps / 2)
        osDelay(1);

      if(mp3Info.nChans == 2) {
        for(int i = 0; i < mp3Info.outputSamps;) {
          stereo[playerdata->buffer_wr * 2] = (float)mp3buffer[i++] * volume;
          stereo[playerdata->buffer_wr * 2 + 1] = (float)mp3buffer[i++] * volume;

          if(playerdata->buffer_wr + 1 >= playerdata->bufferSize)
            playerdata->buffer_wr = 0;
          else playerdata->buffer_wr++;
        }
      } else if(mp3Info.nChans == 1) {
        for(int i = 0; i < mp3Info.outputSamps; i++) {
          mp3buffer[i] = (float)mp3buffer[i] * volume;
          stereo[playerdata->buffer_wr * 2] = mp3buffer[i];
          stereo[playerdata->buffer_wr * 2 + 1] = mp3buffer[i];
          playerdata->buffer_wr++;
        }
      }

      playerdata->playing = 1;


      filesize -= read;
    } else {
      osDelay(1);
    }
  }
}

int main(void)
{
  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SAI2_Init();
  MX_SDMMC1_SD_Init();
  MX_USART1_UART_Init();
  //MX_IWDG_Init();
  MX_TIM5_Init();
  MX_FATFS_Init();

  osKernelInitialize();

  taskCommHandle = osThreadNew(StartCommTask, &gPlayersData, &taskComm_attributes);

  osKernelStart();

  while (1)
  {

  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SAI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SAI2_Init(void)
{

  /* USER CODE BEGIN SAI2_Init 0 */

  /* USER CODE END SAI2_Init 0 */

  /* USER CODE BEGIN SAI2_Init 1 */

  /* USER CODE END SAI2_Init 1 */
  hsai_BlockA1.Instance = SAI2_Block_A;
  hsai_BlockA1.Init.Protocol = SAI_FREE_PROTOCOL;
  hsai_BlockA1.Init.AudioMode = SAI_MODEMASTER_TX;
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_32;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_FALLINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_EMPTY;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_STEREOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.FrameInit.FrameLength = 256;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_LOW;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_FIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_32B;
  hsai_BlockA1.SlotInit.SlotNumber = 8;
  hsai_BlockA1.SlotInit.SlotActive = 0x00000000;
  if (HAL_SAI_Init(&hsai_BlockA1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SAI2_Init 2 */

  /* USER CODE END SAI2_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockBypass = SDMMC_CLOCK_BYPASS_DISABLE;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 108-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 12, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 12, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_D3_Pin|LED_D4_Pin|LED_D5_Pin|LED_D6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SDMMC1_DETECT_Pin */
  GPIO_InitStruct.Pin = SDMMC1_DETECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SDMMC1_DETECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_D3_Pin LED_D4_Pin LED_D5_Pin */
  GPIO_InitStruct.Pin = LED_D3_Pin|LED_D4_Pin|LED_D5_Pin|LED_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
