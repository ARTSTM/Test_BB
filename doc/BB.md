----! Presentation ----!

# Adding code

## 1 - Code section: /* USER CODE BEGIN Includes */

```c
/* Step #1 Include of header files: sensor driver, NEAI lib, printf */
#include "iks01a3_motion_sensors.h"
#include "iks01a3_motion_sensors_ex.h"
#include "NanoEdgeAI.h"
#include "stdio.h"
```

## 2 - Code section: /* USER CODE BEGIN PTD */

```c
/* Step #2 Definition of enumerated type of FSM states*/
typedef enum
{
  LOG,
  LEARN,
  INFERENCE,
} States_t;
```

## 3 - Code section: /* USER CODE BEGIN PD */
```c
/* Step #3 Application configuration */
#define ACCELERO_ODR         (float)(26.0)  /* Accelerometer Output Data Rate [Hz] */
#define ACCELERO_RANGE       (int32_t)(4)   /* Accelerometer range [-4000mg; +4000mg] */
#define LOG_AXIS_NUMBER      (uint32_t)(3)  /* Number of axis of signal to be logged */
#define LOG_SIGNAL_SIZE      (uint32_t)(64 * LOG_AXIS_NUMBER)  /* Signal size to be logged */
#define SIGNAL_SIZE          (uint32_t)(DATA_INPUT_USER * AXIS_NUMBER)  /* Runtime signal size */
#define LEARNING_ITERATIONS  (uint32_t)(8)
#define LD2_LOG_PERIOD       (uint32_t)(75)  /* LD2 toggling half period [ms] following the app state */
#define LD2_LEARN_PERIOD     (uint32_t)(1000)
#define LD2_INFERENCE_PERIOD (uint32_t)(250)
#define LD2_ANOMALY_PERIOD   (uint32_t)(20)
```

## 4 - Code section: /* USER CODE BEGIN PV */
```c
/* Step #4 Variables definition */
volatile States_t appState = LEARN;  /* FSM state */
volatile uint32_t dataRdyFlag = RESET;  /* Accelerometer Data Ready flag -> EXTI */
volatile uint32_t ld2PeriodCnt = 0;  /* LD2 toggling counter incremented by SysTick */
volatile uint32_t ld2Period = 0;  /* LD2 toggle timing to be set by user */
static float neaiBuf[SIGNAL_SIZE];  /* Signal buffer */
```

## 5 - Code section: /* USER CODE BEGIN PFP */
```c
/* Step #5 Private function prototypes */
static void MEMS_Init(void);
static void MODE_Init(void);
static void FillBuffer(uint32_t size,uint8_t mode);
static void Log(void);
static void Learn(void);
static void Inference(void);
```

## 6 - Code section: /* USER CODE BEGIN 0 */
```c
/* Step #6 Redirection of printf output stream to ST-Link VCP */
int _write(int fd, char * ptr, int len)
{
  HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,HAL_MAX_DELAY);
  return len;
}
```

## 7 - Code section: /* USER CODE BEGIN 2 */
```c
/* Step #7 Application initialization */
MEMS_Init();
MODE_Init();
NanoEdgeAI_initialize();
```

## 8 - Code section: /* USER CODE BEGIN 3 */
```c
/* Step #8 Main loop: Finite State Machine */
switch (appState)
{
  case LOG:
    Log();
    break;
  case LEARN:
    Learn();
    appState = INFERENCE;
    break;
  case INFERENCE:
    Inference();
    break;
  default:
    appState = LEARN;
    break;
}
```

## 9 - Code section: /* USER CODE BEGIN 4 */
```c
/* Step #9 Application code, EXTI and SysTick callbacks*/
static void MODE_Init(void)
{
  uint32_t i;

  for (i=0;i<100;i++)  /* USER Button (PC13) to select app mode: PC13=1->LEARN/INFERENCE, PC13=0 (for 1s)->LOG */
  {
	if (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==GPIO_PIN_SET)
    {
      appState = LEARN;
      break;
    }
    HAL_Delay(10);
  }
  if (i>=100)
  {
	appState = LOG;
  }

}

static void MEMS_Init(void)
{
  IKS01A3_MOTION_SENSOR_AxesRaw_t axes;

  IKS01A3_MOTION_SENSOR_Init(IKS01A3_LSM6DSO_0,MOTION_ACCELERO);
  IKS01A3_MOTION_SENSOR_DRDY_Enable_Interrupt(IKS01A3_LSM6DSO_0,MOTION_ACCELERO,IKS01A3_MOTION_SENSOR_INT1_PIN);
  IKS01A3_MOTION_SENSOR_SetOutputDataRate(IKS01A3_LSM6DSO_0,MOTION_ACCELERO,ACCELERO_ODR);
  IKS01A3_MOTION_SENSOR_SetFullScale(IKS01A3_LSM6DSO_0,MOTION_ACCELERO,ACCELERO_RANGE);
  IKS01A3_MOTION_SENSOR_GetAxesRaw(IKS01A3_LSM6DSO_0,MOTION_ACCELERO,&axes);
  IKS01A3_MOTION_SENSOR_Enable(IKS01A3_LSM6DSO_0,MOTION_ACCELERO);
}

static void FillBuffer(uint32_t size, uint8_t mode)
{
  IKS01A3_MOTION_SENSOR_Axes_t acc_axes;
  uint32_t idx = 0;

  while (idx < size)
  {
    if (dataRdyFlag!=RESET)
    {
      dataRdyFlag = RESET;
      IKS01A3_MOTION_SENSOR_GetAxes(IKS01A3_LSM6DSO_0,MOTION_ACCELERO,&acc_axes);
      if (idx < size)
      {
        neaiBuf[idx] =   (float)acc_axes.x;
        neaiBuf[idx+1] = (float)acc_axes.y;
        neaiBuf[idx+2] = (float)acc_axes.z;
        if (mode!=0)
        {
          if (idx < size-AXIS_NUMBER)
          {
            printf("%d,%d,%d,",(int)acc_axes.x,(int)acc_axes.y,(int)acc_axes.z);
          }
          else  /* no separator, new line */
          {
            printf("%d,%d,%d\r\n",(int)acc_axes.x,(int)acc_axes.y,(int)acc_axes.z);
          }
        }
        idx += AXIS_NUMBER;
      }
    }
  }
}

static void Log(void)
{
  ld2Period = LD2_LOG_PERIOD;
  FillBuffer(LOG_SIGNAL_SIZE,1);
}

static void Learn(void)
{
  uint8_t learnStatus;
  uint32_t learnIteration = 0;

  ld2Period = LD2_LEARN_PERIOD;
  while (learnIteration < LEARNING_ITERATIONS)
  {
    printf("Learning... %d/%d",(int)learnIteration+1,(int)LEARNING_ITERATIONS);
    FillBuffer(SIGNAL_SIZE,0);
    learnStatus = NanoEdgeAI_learn(neaiBuf);
    if (learnStatus==1) { printf(" -> OK\r\n"); }
    else { printf(" -> ERR\r\n"); }
    learnIteration++;
  }
}

static void Inference(void)
{
  uint8_t similarity;

  FillBuffer(SIGNAL_SIZE,0);
  similarity = NanoEdgeAI_detect(neaiBuf);
  printf("Similarity = %d%%",(int)similarity);
  if (similarity<90)
  {
	ld2Period = LD2_ANOMALY_PERIOD;
    printf(" -> ANOMALY\r\n");
    /* Add delay here to indicate Anomaly when high ODR -> short time of signal acquisition */
  }
  else
  {
	ld2Period = LD2_INFERENCE_PERIOD;
    printf(" -> OK\r\n");
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_5)  /* Accelerometer Data Ready */
  {
    dataRdyFlag=SET;
  }
  else if (GPIO_Pin == GPIO_PIN_13)  /* USER Button Press */
  {
    if (appState==INFERENCE)
    {
      appState = LEARN;
    }
  }
}

void HAL_IncTick(void)
{
  uwTick += (uint32_t)uwTickFreq;
  if(ld2PeriodCnt>0)
  {
	ld2PeriodCnt--;
  }
  else
  {
    ld2PeriodCnt = ld2Period;
    HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
  }
}
```