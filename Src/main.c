/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 16/07/2014 20:44:35
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN 0 */
#include "usbd_cdc_buf.h"
osMessageQId RcvBoxId;
static uint16_t idxTxBuffer = 0;

/* USER CODE END 0 */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void StartThread(void const * argument);

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* System interrupt init*/
  /* Sets the priority grouping field */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
 	osMessageQDef(RcvBox, RX_BUFFER_COUNT, uint32_t);
	RcvBoxId = osMessageCreate(osMessageQ(RcvBox), NULL);

  /* USER CODE END 2 */

  /* Code generated for FreeRTOS */
  /* Create Start thread */
  osThreadDef(USER_Thread, StartThread, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  osThreadCreate (osThread(USER_Thread), NULL);

  /* Start scheduler */
  osKernelStart(NULL, NULL);

  /* We should never get here as control is now taken by the scheduler */

  /* USER CODE BEGIN 3 */
  /* Infinite loop */
  while (1)
  {

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBPLLCLK_DIV1_5;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __SYSCFG_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

#define MAX_LINE_LENGTH 128
#define MAX_CMD_BUF_COUNT	3
typedef struct {
	volatile uint32_t Length;
	uint8_t Buffer[MAX_LINE_LENGTH];
} CommandBufferDef;
CommandBufferDef CmdBuf[MAX_CMD_BUF_COUNT];
static uint16_t currentCmdIdx;

struct {
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
} GpioLedMap[8] = {
	{GPIOE, GPIO_PIN_9},		/* LD3 */
	{GPIOE, GPIO_PIN_8},		/* LD4 */
	{GPIOE, GPIO_PIN_15},		/* LD6 */
	{GPIOE, GPIO_PIN_14},		/* LD8 */
	{GPIOE, GPIO_PIN_13},		/* LD10 */
	{GPIOE, GPIO_PIN_12},		/* LD9 */
	{GPIOE, GPIO_PIN_11},		/* LD7 */
	{GPIOE, GPIO_PIN_10},		/* LD5 */
};

/**
 * @brief Set LD3 to LE10 state according to ascii code.
 * @param ch: ascii code
 * @retval None
 */
static void AsciiToLed(uint8_t ch)
{
	for (int i = 0; i < 8; i++) {
		GPIO_PinState state = ((ch >> i) & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GpioLedMap[i].GPIOx, GpioLedMap[i].GPIO_Pin, state);
	}
}


static void PrintStr(char *str, uint32_t len)
{
	UsbUserBufferDef *txBufPtr = &UsbUserTxBuffer[idxTxBuffer];
	memcpy(txBufPtr->Buffer, str, len);
	txBufPtr->Length = len;
	while (CDC_Transmit_FS(txBufPtr->Buffer, len) != USBD_OK) {}
	idxTxBuffer = (idxTxBuffer + 1) % TX_BUFFER_COUNT;
}

#define BACKSPACE_ECHO	"\b \b"

static void ParseInputChars(UsbUserBufferDef *rxPtr)
{
	uint8_t *p = &rxPtr->Buffer[0];
	uint8_t *tail = &rxPtr->Buffer[rxPtr->Length];
	CommandBufferDef *cmdBufPtr = &CmdBuf[currentCmdIdx];
	while (p < tail) {
		switch (*p) {
			case '\r':
				// execute command
				PrintStr("\r\n", 2);
				PrintStr((char *)cmdBufPtr->Buffer, cmdBufPtr->Length);
				PrintStr("\r\n", 2);
			  currentCmdIdx = (currentCmdIdx + 1 ) % MAX_CMD_BUF_COUNT;
				cmdBufPtr = &CmdBuf[currentCmdIdx];
				cmdBufPtr->Length = 0;
				break;
			case '\b':
				if (cmdBufPtr->Length > 0) {
					cmdBufPtr->Length--;
					PrintStr(BACKSPACE_ECHO, strlen(BACKSPACE_ECHO));
				}
				break;
			default:
				PrintStr((void *)p, 1);
				AsciiToLed(*p);
				cmdBufPtr->Buffer[cmdBufPtr->Length] = *p;
				cmdBufPtr->Length++;
		}
		p++;
	}
}

/* USER CODE END 4 */

static void StartThread(void const * argument) {

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */
 
	osEvent evt;
	UsbUserBufferDef *rxBufPtr;
  /* Infinite loop */
  for(;;)
  {
    evt = osMessageGet(RcvBoxId, osWaitForever);
		// VCP EchoBack
		if (evt.status == osEventMessage) {
			rxBufPtr = evt.value.p;
			ParseInputChars(rxBufPtr);
		}
		//check received length, read UserRxBufferFS
  }

  /* USER CODE END 5 */ 

}
 

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
/* User can add his own implementation to report the file name and line number,
ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
