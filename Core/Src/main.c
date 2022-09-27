#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "mp3common.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <limits.h>
#include "bsp_driver_sd.h"

typedef StaticTask_t osStaticThreadDef_t;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

SAI_HandleTypeDef hsai_BlockA1;
DMA_HandleTypeDef hdma_sai2_a;

SD_HandleTypeDef hsd1;
DMA_HandleTypeDef hdma_sdmmc1_rx;
DMA_HandleTypeDef hdma_sdmmc1_tx;

TIM_HandleTypeDef htim5;

osThreadId_t taskCommHandle;
uint32_t taskCommBuffer[512];
osStaticThreadDef_t taskCommControlBlock;
const osThreadAttr_t taskComm_attributes = {
  .name = "taskComm",
  .cb_mem = &taskCommControlBlock,
  .cb_size = sizeof(taskCommControlBlock),
  .stack_mem = &taskCommBuffer[0],
  .stack_size = sizeof(taskCommBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadAttr_t taskPlayer_attributes = {
  .name = "taskPlayer",
  .stack_size = 320 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

osThreadAttr_t taskIdle_attributes = {
  .name = "taskIdle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityIdle,
};

const osMutexAttr_t mutexMp3_attributes = {
    .name= "mutexMp3",
    .attr_bits = osMutexRecursive,
    .cb_mem = NULL,
    .cb_size = 0
};

const osMutexAttr_t mutexFS_attributes = {
    .name= "mutexFS",
    .attr_bits = osMutexRecursive,
    .cb_mem = NULL,
    .cb_size = 0
};

const osMutexAttr_t mutexCommTx_attributes = {
    .name= "mutexCommTx",
    .attr_bits = osMutexRecursive,
    .cb_mem = NULL,
    .cb_size = 0
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
void StartIdleTask(void *argument);
void StartCommTask(void *argument);
void StartPlayerTask(void *argument);

#define UART_CMD_BUFFER 64
#define UART_TX_BUFFER 256
#define UART_RX_BUFFER 256

#define PLAYERS_COUNT       (4)
#define MP3_SAMPLE_SIZE     (1152 * 2)
#define SAMPLE_BUFFER_SIZE  ((MP3_SAMPLE_SIZE) * 4 / 3)
#define FILE_BUFFER_SIZE    (2048)
#define MAX_FILE_PATH       (128)

int16_t gSampleBuffer[PLAYERS_COUNT][MP3_SAMPLE_SIZE];
int16_t gSamplesBuffer[PLAYERS_COUNT][SAMPLE_BUFFER_SIZE];
uint8_t gFileBuffer[PLAYERS_COUNT][FILE_BUFFER_SIZE];
osMutexId_t gMp3Mutex;
osMutexId_t gFSMutex;
osThreadId_t gTaskIdle;

volatile float gCpuLoad = 0;
volatile float gCpuLoadMax = 0;
volatile uint32_t gIdleTick = 1000;
volatile uint32_t gMp3LedLast = 0;

#define TDM_BUFFER_SIZE 1152
uint16_t gTdmFinalBuffer[TDM_BUFFER_SIZE] __attribute__((aligned(32)));

typedef struct {
    const char *name;
    HMP3Decoder decoder;
    char file[MAX_FILE_PATH];
    volatile uint8_t enabled;
    volatile uint8_t playing;
    volatile uint8_t changed;
    uint8_t volume;
    osMutexId_t mutex;
    osThreadId_t task;
    uint32_t bufferSize;
    int16_t *buffer;
    uint8_t *fileBuffer;
    uint32_t fileBufferSize;
    int16_t *sampleBuffer;
    uint32_t sampleBufferSize;
    uint32_t buffer_rd;
    uint32_t buffer_wr;
    uint32_t underflow_rd;
    uint32_t mp3_errors;
    uint32_t fs_errors;
    float file_percentage;
}sPlayerData;

typedef struct {
    uint32_t count;
    sPlayerData player[PLAYERS_COUNT];
}sPlayersData;

typedef struct {
    uint8_t rx_buffer[UART_RX_BUFFER];
    char tx_buffer[UART_TX_BUFFER];
    char rx_cmd[UART_CMD_BUFFER];
    UART_HandleTypeDef *huart;
    uint32_t rx_rd;
    uint32_t rx_wr;
    uint32_t rx_cmd_ptr;
    osSemaphoreId_t tx_sem;
    osMutexId_t mutex;
}sCommData __attribute__((aligned(32)));

sCommData gCommData = {0};
sPlayersData gPlayersData = { PLAYERS_COUNT };


static inline uint32_t getavail(uint32_t wr, uint32_t rd, uint32_t size) {
  if(wr >= rd) return (wr - rd);
  else return (size - rd + wr);
}

static inline uint32_t getfree(uint32_t wr, uint32_t rd, uint32_t size) {
  return size - getavail(wr, rd, size);
}

static void HandleSaiDma(uint16_t *buffer, uint32_t size)
{
  uint32_t samples_per_channel = size / PLAYERS_COUNT / 2;
  int index;

  for(int i = 0; i < PLAYERS_COUNT; i++)
  {
    if(getavail(gPlayersData.player[i].buffer_wr, gPlayersData.player[i].buffer_rd, gPlayersData.player[i].bufferSize) >= samples_per_channel)
    {
      for(int j = 0; j < samples_per_channel; j++)
      {
        index = (((j & ~1) * PLAYERS_COUNT) + (i << 1) + (j & 1)) << 1;
        buffer[index] = gPlayersData.player[i].buffer[gPlayersData.player[i].buffer_rd];
        buffer[index + 1] = 0;
        if(gPlayersData.player[i].buffer_rd + 1 >= gPlayersData.player[i].bufferSize)
          gPlayersData.player[i].buffer_rd = 0;
        else gPlayersData.player[i].buffer_rd++;
      }
    } else {
      if(gPlayersData.player[i].playing) {
        gPlayersData.player[i].underflow_rd++;
      }
      for(int j = 0; j < samples_per_channel; j++)
      {
        index = (((j & ~1) * PLAYERS_COUNT) + (i << 1) + (j & 1)) << 1;
        buffer[index] = 0;
        buffer[index + 1] = 0;
      }
    }
  }

  SCB_CleanDCache_by_Addr((uint32_t *)buffer, size * sizeof(*buffer));
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
  HandleSaiDma(&gTdmFinalBuffer[TDM_BUFFER_SIZE / 2], TDM_BUFFER_SIZE / 2);
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  HandleSaiDma(&gTdmFinalBuffer[0], TDM_BUFFER_SIZE / 2);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == gCommData.huart) {
    osSemaphoreRelease(gCommData.tx_sem);
  }
}

osStatus_t Comm_Transmit(sCommData *commdata, char *cmd, ...)
{
  osStatus_t sem_status;
  int strl;
  va_list va_args;

  sem_status = osMutexAcquire(commdata->mutex, osWaitForever);
  if(sem_status == osOK) {
    sem_status = osSemaphoreAcquire(commdata->tx_sem, osWaitForever);
    if(sem_status == osOK) {
      va_start(va_args, cmd);
      vsnprintf(commdata->tx_buffer, UART_TX_BUFFER - 4, cmd, va_args);
      va_end(va_args);
      strl = strlen(commdata->tx_buffer);
      commdata->tx_buffer[strl++] = '\r';
      commdata->tx_buffer[strl++] = '\n';

      SCB_CleanDCache_by_Addr((uint32_t *)commdata->tx_buffer, strl);
      if(HAL_UART_Transmit_DMA(commdata->huart, (uint8_t *)commdata->tx_buffer, strl) != HAL_OK) {
        osSemaphoreRelease(gCommData.tx_sem);
        sem_status = osError;
      }
    }
    osMutexRelease(commdata->mutex);
  }

  return sem_status;
}

int Comm_RxProcess(sCommData *commdata, char **command, uint32_t *size)
{
  char chr;

  *command = NULL;

  commdata->rx_wr = commdata->huart->RxXferSize - commdata->huart->hdmarx->Instance->NDTR;

  while(commdata->rx_wr != commdata->rx_rd) {
    SCB_InvalidateDCache_by_Addr((uint32_t *)commdata->rx_buffer, commdata->huart->RxXferSize);
    chr = commdata->rx_buffer[commdata->rx_rd++];
    if(commdata->rx_rd >= UART_RX_BUFFER || commdata->rx_rd >= commdata->huart->RxXferSize) {
      commdata->rx_rd = 0;
    }

    if(commdata->rx_cmd_ptr + 1 >= UART_CMD_BUFFER || chr == '\r' || chr == '\n') {
      commdata->rx_cmd[commdata->rx_cmd_ptr] = '\0';
      if(commdata->rx_cmd_ptr > 0) {
        if(command) {
          memset(&commdata->rx_cmd[commdata->rx_cmd_ptr], 0, UART_CMD_BUFFER - commdata->rx_cmd_ptr);
          *command = commdata->rx_cmd;
        }
        if(size) {
          *size = commdata->rx_cmd_ptr;
        }
        commdata->rx_cmd_ptr = 0;
        return 1;
      }
    } else {
      commdata->rx_cmd[commdata->rx_cmd_ptr++] = chr;
    }
  }
  return 0;
}

void StartIdleTask(void *argument)
{
  while(1) {
    gIdleTick++;
    osDelay(1);
  }
}

void StartCommTask(void *argument)
{
  uint64_t time = 0;
  sSdStats sdstats;
  sPlayersData *playersdata = (sPlayersData *)argument;
  char *str = NULL;
  char *args = NULL;
  uint32_t len = 0;
  uint32_t arg;
  uint32_t tick;
  uint32_t current;
  uint32_t arg_start, arg_end;
  const float msr_koff = 0.0166667f;
  float sd_avg = 0;
  float ram_avg = 0;
  float cpu_avg = 0;

  FATFS fs = {0};
  FRESULT res;

  HAL_GPIO_WritePin(LED_D3_GPIO_Port, LED_D3_Pin, GPIO_PIN_SET);

  while(1) {
    res = f_mount(&fs, SDPath, 1);
    if(res == FR_OK)
      break;
    osDelay(200);
    HAL_GPIO_TogglePin(LED_D4_GPIO_Port, LED_D4_Pin);
  }

  HAL_GPIO_WritePin(LED_D4_GPIO_Port, LED_D4_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_D5_GPIO_Port, LED_D5_Pin, GPIO_PIN_SET);

  gMp3Mutex = osMutexNew(&mutexMp3_attributes);
  gFSMutex = osMutexNew(&mutexFS_attributes);

  memset(&gCommData, 0, sizeof(gCommData));
  gCommData.huart = &huart1;
  gCommData.tx_sem = osSemaphoreNew(1, 1, NULL);
  gCommData.mutex = osMutexNew(&mutexCommTx_attributes);
  HAL_UART_Receive_DMA(gCommData.huart, gCommData.rx_buffer, UART_RX_BUFFER);

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

    playersdata->player[i].decoder = MP3InitDecoder();
    playersdata->player[i].mutex = osMutexNew(&mutexPlayer_attributes);
    playersdata->player[i].task = osThreadNew(StartPlayerTask, &playersdata->player[i], &taskPlayer_attributes);
  }

  HandleSaiDma(gTdmFinalBuffer, TDM_BUFFER_SIZE);
  HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t*)gTdmFinalBuffer, TDM_BUFFER_SIZE);

  gTaskIdle = osThreadNew(StartIdleTask, NULL, &taskIdle_attributes);

  current = xTaskGetTickCount();
  tick = 0;

  for(;;)
  {
    if(Comm_RxProcess(&gCommData, &str, &len)) {
      for(int i = 0; i < gPlayersData.count; i++) {
        if(strstr(str, gPlayersData.player[i].name) == str && str[strlen(gPlayersData.player[i].name)] == '.') {
          str += strlen(gPlayersData.player[i].name) + 1;
          arg_start = -1;
          arg_end = -1;
          args = strchr(str, '(');
          if(args == NULL)
            break;
          arg_start = args - str;
          while(args != NULL) {
            args = strchr(args + 1, ')');
            if(args == NULL)
              break;
            arg_end = args - str;
          }
          if(arg_end == -1)
            break;
          if(arg_start < arg_end) {
            args = &str[arg_start + 1];
            str[arg_start] = '\0';
            str[arg_end] = '\0';
            if(strcmp(str, "play") == 0) {
              osMutexAcquire(gPlayersData.player[i].mutex, osWaitForever);
              strcpy(gPlayersData.player[i].file, args);
              gPlayersData.player[i].enabled = 1;
              gPlayersData.player[i].changed = 1;
              osMutexRelease(gPlayersData.player[i].mutex);
              Comm_Transmit(&gCommData, "OK");
            } else if(strcmp(str, "stop") == 0) {
              osMutexAcquire(gPlayersData.player[i].mutex, osWaitForever);
              gPlayersData.player[i].enabled = 0;
              gPlayersData.player[i].changed = 1;
              osMutexRelease(gPlayersData.player[i].mutex);
              Comm_Transmit(&gCommData, "OK");
            } else if(strcmp(str, "vol") == 0) {
              if(sscanf(args, "%ld", &arg) == 1) {
                osMutexAcquire(gPlayersData.player[i].mutex, osWaitForever);
                gPlayersData.player[i].volume = arg;
                osMutexRelease(gPlayersData.player[i].mutex);
                Comm_Transmit(&gCommData, "OK");
              }
            }
          }
          break;
        }
      }
    }

    taskENTER_CRITICAL();
    if(xTaskGetTickCount() - gMp3LedLast > 1000) {
      gMp3LedLast = xTaskGetTickCount();
      HAL_GPIO_WritePin(LED_D6_GPIO_Port, LED_D6_Pin, GPIO_PIN_RESET);
    }
    taskEXIT_CRITICAL();

    current += 30;
    osDelayUntil(current);
    if(++tick >= 1000/30) {
      tick = 0;
      time++;

      sdstats = BSP_SD_GetStats();

      if(gIdleTick > 1000)
        gIdleTick = 1000;

      gCpuLoad = (float)(1000 - gIdleTick) * 0.1f;
      gIdleTick = 0;

      if(gCpuLoadMax < gCpuLoad)
        gCpuLoadMax = gCpuLoad;

      if(cpu_avg == 0.0f)
        cpu_avg = gCpuLoad;
      else cpu_avg = cpu_avg * (1.0f - msr_koff) + gCpuLoad * msr_koff;

      if(sd_avg == 0.0f)
        sd_avg = sdstats.readLast;
      else sd_avg = sd_avg * (1.0f - msr_koff) + sdstats.readLast * msr_koff;

      if(ram_avg == 0.0f)
        ram_avg =  xPortGetFreeHeapSize();
      else ram_avg = ram_avg * (1.0f - msr_koff) + xPortGetFreeHeapSize() * msr_koff;

      Comm_Transmit(&gCommData, "SD: %.1fKB/s Max: %.1fKB/s Avg: %.1fKB/s\r\n"
          "CPU: %.1f%% Max: %.1f%% Avg: %.1f%%\r\n"
          "RAM: %.2fKB Min: %.1fKB Avg: %.2fKB",

          (float)sdstats.readLast / 1024.0f,
          (float)sdstats.readMax / 1024.0f,
          sd_avg / 1024.0f,

          gCpuLoad,
          gCpuLoadMax,
          cpu_avg,

          (float)xPortGetFreeHeapSize() / 1024.0f,
          (float)xPortGetMinimumEverFreeHeapSize() / 1024.0f,
          ram_avg / 1024.0f);

    }
  }
}

void StartPlayerTask(void *argument)
{
  sPlayerData *playerdata = (sPlayerData *)argument;
  MP3FrameInfo mp3Info;


  int32_t syncword;
  int16_t *buffer;
  int16_t *samplebuffer = NULL;
  uint8_t *filebuffer = NULL;
  uint32_t filebuffersize = 0;
  uint8_t playing = 0;
  uint8_t enabled = 0;
  uint8_t changed = 0;
  uint8_t restart = 0;
  float volume = 1.0f;

  FIL file = {0};
  FRESULT res;
  UINT read;
  uint32_t toread = 0;
  uint32_t mp3read = 0;
  uint32_t mp3read_prev = 0;
  uint32_t content = 0;

  uint8_t *tempfilebuffer;
  int16_t *mp3buffer;

  playing = 0;

  while(1)
  {
    filebuffer = playerdata->fileBuffer;
    filebuffersize = playerdata->fileBufferSize;
    samplebuffer = playerdata->sampleBuffer;
    buffer = playerdata->buffer;

    if(osMutexAcquire(playerdata->mutex, osWaitForever) == osOK) {
      volume = playerdata->volume * 0.01f;
      enabled = playerdata->enabled;
      changed = playerdata->changed;
      osMutexRelease(playerdata->mutex);
    }

    if(changed || restart) {
      if(osMutexAcquire(playerdata->mutex, 0) == osOK) {
        restart = 0;
        playerdata->buffer_rd = 0;
        playerdata->buffer_wr = 0;
        playerdata->changed = 0;
        if(playing) {
          playing = 0;
          osMutexRelease(playerdata->mutex);
          //memset(buffer, 0, buffersize * sizeof(*buffer));
          content = 0;
          osMutexAcquire(gFSMutex, osWaitForever);
          res = f_close(&file);
          osMutexRelease(gFSMutex);
          if(res != FR_OK) {

          }
        } else {
          osMutexRelease(playerdata->mutex);
        }

        if(enabled) {
          content = 0;
          osMutexAcquire(gFSMutex, osWaitForever);
          res = f_open(&file, playerdata->file, FA_READ);
          osMutexRelease(gFSMutex);
          if(res != FR_OK) {
            playerdata->enabled = 0;
            enabled = 0;
            playing = 0;
          } else {
            playing = 1;
          }

        }
      }
    }

    playerdata->playing = playing;

    if(playing) {

      if(f_eof(&file)) {
        osMutexAcquire(gFSMutex, osWaitForever);
        res = f_rewind(&file);
        osMutexRelease(gFSMutex);
        if(res != FR_OK) {

        }
        playerdata->playing = 0;
      }

      if(content < 2048) {
        toread = filebuffersize - content;
        if(toread > f_size(&file) - f_tell(&file))
          toread = f_size(&file) - f_tell(&file);

        for(int i = 0; i < content; i++) {
          filebuffer[i] = filebuffer[toread + i];
        }

        while(toread > 0) {
          osMutexAcquire(gFSMutex, osWaitForever);
          res = f_read(&file, &filebuffer[content], toread, &read);
          osMutexRelease(gFSMutex);

          content += read;
          toread -= read;

          if(res != FR_OK) {
            break;
          }
        }

        if(res != FR_OK) {
          restart = 1;
          playerdata->fs_errors++;
          continue;
        }
      }

      tempfilebuffer = &filebuffer[filebuffersize - content];

      syncword = MP3FindSyncWord(tempfilebuffer, content);
      if(syncword <= ERR_MP3_INDATA_UNDERFLOW)
      {
        playerdata->mp3_errors++;
        content = 0;
        continue;
      }
      if(syncword > 0)
      {
        content -= syncword;
        continue;
      }

      osMutexAcquire(gMp3Mutex, osWaitForever);

      syncword = MP3GetNextFrameInfo(playerdata->decoder, &mp3Info, tempfilebuffer);
      if(syncword != ERR_MP3_NONE)
      {
        osMutexRelease(gMp3Mutex);
        playerdata->mp3_errors++;
        if(content > 2) {
          content -= 2;
        } else {
          content = 0;
        }
        continue;
      }

      mp3buffer = (int16_t *)samplebuffer;
      mp3read_prev = content;
      mp3read = content;

      if(playerdata == &gPlayersData.player[0] && mp3read_prev == 3678 && f_tell(&file) == 6228) {
        playerdata->file_percentage++;

      }

      //DecodeHuffmanPairs returns -2 while parallel decoding of multiple MP3s :(
      syncword = MP3Decode(playerdata->decoder, &tempfilebuffer, (int*)&mp3read, mp3buffer, 0);
      osMutexRelease(gMp3Mutex);
      content = mp3read;

      if(syncword != ERR_MP3_NONE) {
        if(playerdata == &gPlayersData.player[0]) {
          playerdata->file_percentage++;
        }
        playerdata->mp3_errors++;
        continue;
      }


      if(mp3Info.samprate == 0 || mp3Info.nChans <= 0) {
        continue;
      }

      playerdata->file_percentage = ((float)f_tell(&file) / (float)f_size(&file)) * 100.0f;

      while(getfree(playerdata->buffer_wr, playerdata->buffer_rd, playerdata->bufferSize) <= mp3Info.outputSamps) {
        osDelay(1);
      }

      if(mp3Info.nChans == 2) {
        for(int i = 0; i < mp3Info.outputSamps; i++) {
          buffer[playerdata->buffer_wr] = (float)(mp3buffer[i]) * volume;

          if(playerdata->buffer_wr + 1 >= playerdata->bufferSize)
            playerdata->buffer_wr = 0;
          else playerdata->buffer_wr++;
        }
      } else if(mp3Info.nChans == 1) {
        for(int i = 0; i < mp3Info.outputSamps * 2; i++) {
          buffer[playerdata->buffer_wr] = (float)(mp3buffer[i >> 1]) * volume;

          if(playerdata->buffer_wr + 1 >= playerdata->bufferSize)
            playerdata->buffer_wr = 0;
          else playerdata->buffer_wr++;
        }
      }

      playerdata->playing = 1;

      taskENTER_CRITICAL();
      if(xTaskGetTickCount() - gMp3LedLast > 100) {
        gMp3LedLast = xTaskGetTickCount();
        HAL_GPIO_TogglePin(LED_D6_GPIO_Port, LED_D6_Pin);
      }
      taskEXIT_CRITICAL();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

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
  RCC_OscInitStruct.PLL.PLLM = 25;
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



  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_PLLI2S | RCC_PERIPHCLK_SAI2;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 158; //44100 = 158, 48000 = 172
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 7;
  PeriphClkInitStruct.PLLI2S.PLLI2SQ = 7;
  PeriphClkInitStruct.PLLI2S.PLLI2SP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLI2SDivQ = 1;
  PeriphClkInitStruct.Sai2ClockSelection = RCC_SAI2CLKSOURCE_PLLI2S;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_RCC_MCOConfig(RCC_MCO2, RCC_MCO2SOURCE_PLLI2SCLK, RCC_MCODIV_1);
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
  hsai_BlockA1.Init.DataSize = SAI_DATASIZE_16;
  hsai_BlockA1.Init.FirstBit = SAI_FIRSTBIT_MSB;
  hsai_BlockA1.Init.ClockStrobing = SAI_CLOCKSTROBING_RISINGEDGE;
  hsai_BlockA1.Init.Synchro = SAI_ASYNCHRONOUS;
  hsai_BlockA1.Init.OutputDrive = SAI_OUTPUTDRIVE_DISABLE;
  hsai_BlockA1.Init.NoDivider = SAI_MASTERDIVIDER_ENABLE;
  hsai_BlockA1.Init.FIFOThreshold = SAI_FIFOTHRESHOLD_FULL;
  hsai_BlockA1.Init.AudioFrequency = SAI_AUDIO_FREQUENCY_44K;
  hsai_BlockA1.Init.SynchroExt = SAI_SYNCEXT_DISABLE;
  hsai_BlockA1.Init.MonoStereoMode = SAI_MONOMODE;
  hsai_BlockA1.Init.CompandingMode = SAI_NOCOMPANDING;
  hsai_BlockA1.Init.TriState = SAI_OUTPUT_NOTRELEASED;
  hsai_BlockA1.FrameInit.FrameLength = 256;
  hsai_BlockA1.FrameInit.ActiveFrameLength = 1;
  hsai_BlockA1.FrameInit.FSDefinition = SAI_FS_STARTFRAME;
  hsai_BlockA1.FrameInit.FSPolarity = SAI_FS_ACTIVE_HIGH;
  hsai_BlockA1.FrameInit.FSOffset = SAI_FS_BEFOREFIRSTBIT;
  hsai_BlockA1.SlotInit.FirstBitOffset = 0;
  hsai_BlockA1.SlotInit.SlotSize = SAI_SLOTSIZE_DATASIZE;
  hsai_BlockA1.SlotInit.SlotNumber = 16;
  hsai_BlockA1.SlotInit.SlotActive = 0x0000FFFF;
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

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);



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
