/**
  ******************************************************************************
  * File Name          : command.c
  * Description        : Command Interpreter
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
  *
  * COPYRIGHT(c) 2014 Y.Magara
  */

/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "usb_device.h"
#include "gpio.h"
#include "usbd_cdc_buf.h"
#include "command.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN 4 */

#define MAX_CMD_BUF_COUNT	3
static CommandBufferDef CmdBuf[MAX_CMD_BUF_COUNT];
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


void cmdVersion(CommandBufferDef *cmd);
void cmdRelocate(CommandBufferDef *cmd);
void cmdPutOn(CommandBufferDef *cmd);
void cmdTakeOff(CommandBufferDef *cmd);
void cmdHelp(CommandBufferDef *cmd);

struct CmdDic {
	const char *name;
	void (*func)(CommandBufferDef *cmd);
} CmdDic[] = {
	{"VERSION", cmdVersion},
	// RELOCATE
	// PUTON <A/B/C/D>
	// TAKEOFF [A/B/C/D]
	{NULL, NULL}
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

/**
 * Print a string via VCP TX port.
 */
static void PutStr(char *str)
{
	uint32_t len = strlen(str);
	UsbUserBufferDef *txBufPtr = &UsbUserTxBuffer[idxTxBuffer];
	strlcpy((char *)txBufPtr->Buffer, str, MAX_COMMAND_LENGTH);
	txBufPtr->Length = len;
	while (CDC_Transmit_FS(txBufPtr->Buffer, len) == USBD_BUSY) {}
	idxTxBuffer = (idxTxBuffer + 1) % TX_BUFFER_COUNT;
}

/**
 * Print a character via VCP TX port.
 */
static void PutChr(char c)
{
	UsbUserBufferDef *txBufPtr = &UsbUserTxBuffer[idxTxBuffer];
	txBufPtr->Buffer[0] = c;
	txBufPtr->Length = 1;
	while (CDC_Transmit_FS(txBufPtr->Buffer, 1) == USBD_BUSY) {}
	idxTxBuffer = (idxTxBuffer + 1) % TX_BUFFER_COUNT;
}

/**
  * Print version number.
  */
void cmdVersion(CommandBufferDef *cmd)
{
	PutStr(VERSION_STR);
	PutStr("\r\n");
}

/**
  * Re-scan mechanical limit position.
  */
void cmdRelocate(CommandBufferDef *cmd)
{
	PutStr("Re-scan mechanical limit position.\r\n");
}


/**
  * Put a card on the RF antenna.
  */
void cmdPutOn(CommandBufferDef *cmd)
{
	PutStr("Put a card on the RF antenna.\r\n");
}

/**
  * Take a card off from the RF antenna.
  */
void cmdTakeOff(CommandBufferDef *cmd)
{
	PutStr("Take a card off from the RF antenna.\r\n");
}

/**
  * Show command help.
  */
void cmdHelp(CommandBufferDef *cmd)
{
	PutStr("Show command help.\r\n");
}


static void LookupCommand(CommandBufferDef *cmd)
{
	struct CmdDic *cmdPtr = CmdDic;
	while (cmdPtr->name != NULL)
	{
		uint16_t cmdLength = strlen(cmdPtr->name);
		if (cmdPtr->func != NULL & cmd->Length >= cmdLength && strncmp(cmdPtr->name, cmd->Buffer, cmdLength) == 0)
		{
			cmdPtr->func(cmd);
		}
		cmdPtr++;
	}
}

/**
 * Parse input string from VCP RX port.
 */
void ParseInputChars(UsbUserBufferDef *rxPtr)
{
	uint8_t *p = &rxPtr->Buffer[0];
	uint8_t *tail = &rxPtr->Buffer[rxPtr->Length];
	CommandBufferDef *cmdBufPtr = &CmdBuf[currentCmdIdx];
	while (p < tail && cmdBufPtr->Length < MAX_COMMAND_LENGTH) {
		switch (*p) {
			case '\r':
				// execute command
				PutStr("\r\n");
				cmdBufPtr->Buffer[cmdBufPtr->Length] = '\0';
				PutStr((char *)cmdBufPtr->Buffer);
				PutStr("\r\n");
				LookupCommand(cmdBufPtr);
			  currentCmdIdx = (currentCmdIdx + 1 ) % MAX_CMD_BUF_COUNT;
				cmdBufPtr = &CmdBuf[currentCmdIdx];
				cmdBufPtr->Length = 0;
				break;
			case '\b':
				if (cmdBufPtr->Length > 0) {
					cmdBufPtr->Length--;
					PutStr("\b \b");
				}
				break;
			default:
				PutChr(*p);
				AsciiToLed(*p);
				cmdBufPtr->Buffer[cmdBufPtr->Length] = *p;
				cmdBufPtr->Length++;
		}
		p++;
	}
}

/*****END OF FILE****/
