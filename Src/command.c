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
#include "i2c.h"
#include "usbd_cdc_buf.h"
#include "command.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN 4 */

#define MSG_CRLF "\r\n"
#define MSG_EMPTY_ARGUMENT "Empty argument.\r\n"
#define MAG_INVALID_PARAMETER "Invalid parameter.\r\n"

#define MIN_CARD_POS 1
#define MAX_CARD_POS 4

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

typedef struct  {
	const char *name;
	void (*func)(CommandBufferDef *cmd);
} CommandOp;

CommandOp CmdDic[] = {
	{"PUTON", cmdPutOn},
	{"TAKEOFF", cmdTakeOff},
	{"RELOCATE", cmdRelocate},
	{"HELP", cmdHelp},
	{"VERSION", cmdVersion},
	{NULL, NULL}
};

// card position index
typedef enum {
	Index_Ant = 0,
	Index_Home,
	Index_A,
	Index_B,
	Index_C,
	Index_F = Index_C,
	Index_D,
	Index_V = Index_D,
	Index_MAX = Index_D
} TrayIndex;

// turn table angle: [0..95]
#define STEP_PER_REV	96
#define STEP_PER_30DEG	(STEP_PER_REV/12)

// PM step correspnding to TrayAngle
uint16_t TrayStepPos[] = {
	0,
	STEP_PER_30DEG, 
	2*STEP_PER_30DEG,
	3*STEP_PER_30DEG, 
	4*STEP_PER_30DEG,
	5*STEP_PER_30DEG
};
uint16_t currStepPos;

#define I2C_ADDR_PH_A_w	((0x63<<1)+0)
#define I2C_ADDR_PH_B_w	((0x64<<1)+0)
#define I2C_SUB_CTRL	0
#define VSET_MIN	0x06
#define VSET_MAX	0x3F
#define VSET	VSET_MIN
#define OUT_POS	((VSET<<2)+1)
#define OUT_NEG	((VSET<<2)+2)

#define PHASE_COUNT	4
// [PAHSE][A/B][data]
uint8_t PM_PHASE[PHASE_COUNT][2][2] = {
	// phase1
	{
		{I2C_SUB_CTRL, OUT_POS},
		{I2C_SUB_CTRL, OUT_NEG}
	}
	// phase2
	, {
		{I2C_SUB_CTRL, OUT_POS},
		{I2C_SUB_CTRL, OUT_POS}
	}
	// phase3
	, {
		{I2C_SUB_CTRL, OUT_NEG},
		{I2C_SUB_CTRL, OUT_POS}
	}
	// phase4
	, {
		{I2C_SUB_CTRL, OUT_NEG},
		{I2C_SUB_CTRL, OUT_NEG}
	}
};
static uint16_t currentPhase;
#define INTER_PHASE_DELAY_MS	1000

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
	* Convert character to card position No.
	* 0: home, 1:A, 2:B, 3:C, 4:D
	*/
static TrayIndex Chr2CardNo(char c)
{
	TrayIndex card = Index_Ant;
	switch (c)
	{
		case 'A':
			card = Index_A;
			break;
		case 'B':
			card = Index_B;
			break;
		case 'C':
		case 'F':
			card = Index_C;
			break;
		case 'D':
		case 'V':
			card = Index_D;
			break;
		default:
			;
	}
	return card;
}

static void TurnTable(TrayIndex card)
{
	int16_t dst = TrayStepPos[card];
	int16_t diff = dst - currStepPos;
	int16_t step = 0;
	if (diff > 0) {
		step = 1;
	} else if (step < 0) {
		step = -1;
	}
	
	while (dst != currStepPos) {
		uint16_t nextPhase = (currentPhase + step) % PHASE_COUNT;
		if (HAL_I2C_Master_Transmit(&hi2c1, I2C_ADDR_PH_A_w, PM_PHASE[nextPhase][0], 2, 25) != osOK) {
			break;
		}
//		if (HAL_I2C_Master_Transmit(&hi2c2, I2C_ADDR_PH_B_w, PM_PHASE[nextPhase][1], 2, 25) != osOK) {
//			break;
//		}
		currentPhase = nextPhase;
		currStepPos += step;
		osDelay(INTER_PHASE_DELAY_MS);
	}
}

/**
	* 0: home, 1:A, 2:B, 3:C, 4:D
	*/
static void MoveCard(TrayIndex start, TrayIndex end)
{
	if (start > Index_D) {
		PutStr("Invalid start position : ");
		PutChr(start + '0');
		PutStr(MSG_CRLF);
		return;
	}
	if (end > 4) {
		PutStr("Invalid end position : ");
		PutChr(end + '0');
		PutStr(MSG_CRLF);
		return;
	}
	if (start == end) {
		PutStr(MAG_INVALID_PARAMETER);
		return;
	}
	PutStr("Move card from ");
	PutChr(start + '0');
	PutStr(" to ");
	PutChr(end + '0');
	PutStr(".\r\n");
	
	// lift up arm
	// turn arm to FROM position
	TurnTable(start);
	// lift down arm
	// vacuum on
	// lift up arm
	// turn arm to TO position
	TurnTable(end);
	// lift down arm
	// vacuum off
	// lift up arm
	// turn arm to home position
	TurnTable(Index_Home);
}

/**
  * Print version number.
  */
void cmdVersion(CommandBufferDef *cmd)
{
	PutStr(VERSION_STR);
	PutStr(MSG_CRLF);
}

/**
  * Re-scan mechanical limit position.
  */
void cmdRelocate(CommandBufferDef *cmd)
{
	PutStr("Re-scan mechanical limit position.\r\n");
	// lift up arm
	// turn arm to right while mechanical limit
	//    sense photo reflector
	// turn arm to home position
}


/**
  * Put a card on the RF antenna.
	*
	* PUTON <A/B/C/D>
  */
void cmdPutOn(CommandBufferDef *cmd)
{
	if (cmd->Arg == NULL) {
		PutStr(MSG_EMPTY_ARGUMENT);
	}
	// 0: home, 1:A, 2:B, 3:C, 4:D
	TrayIndex card = Chr2CardNo(cmd->Arg[0]);
	if (card >= MIN_CARD_POS && card <= MAX_CARD_POS) {
		MoveCard(card, Index_Ant);
	} else {
		PutStr(MAG_INVALID_PARAMETER);
	}
}

/**
  * Take a card off from the RF antenna.
	*
	* TAKEOFF <A/B/C/D>
  */
void cmdTakeOff(CommandBufferDef *cmd)
{
	if (cmd->Arg == NULL) {
		PutStr(MSG_EMPTY_ARGUMENT);
	}
	// 0: home, 1:A, 2:B, 3:C, 4:D
	TrayIndex card = Chr2CardNo(cmd->Arg[0]);
	if (card >= MIN_CARD_POS && card <= MAX_CARD_POS) {
		MoveCard(Index_Ant, card);
	} else {
		PutStr(MAG_INVALID_PARAMETER);
	}
}

/**
  * Show command help.
  */
void cmdHelp(CommandBufferDef *cmd)
{
	PutStr("Show command help.\r\n");
}

/**
  * Split command and argument.
  */
static void SplitArg(CommandBufferDef *cmd)
{
	cmd->Arg = NULL;
	char *ptr = cmd->Buffer;
	char *tail = ptr + MAX_COMMAND_LENGTH;
	while (ptr < tail) {
		if (*ptr == '\0' || *ptr == ' ' || *ptr=='\t') {
			break;
		}
		ptr++;
	}
	cmd->CmdLength = (ptr - cmd->Buffer);
	if (*ptr != '\0') {
		while (ptr < tail) {
			ptr++;
			if (*ptr != ' ' && *ptr != '\t') {
				cmd->Arg = ptr;
				break;
			}
		}
	}
}
void StartMotorThread(void const * argument)
{
	osEvent evt;
	CommandBufferDef *cmdBuf;
  /* Infinite loop */
  for(;;)
  {
    evt = osMessageGet(CmdBoxId, osWaitForever);
		if (evt.status == osEventMessage) {
			cmdBuf = evt.value.p;
			cmdBuf->func(cmdBuf);
		}
		//check received length, read UserRxBufferFS
  }
}
static void LookupCommand(CommandBufferDef *cmd)
{
	CommandOp *cmdPtr, *matched;
	uint16_t matchCount;
	SplitArg(cmd);
	for (int len = 1; len <= cmd->CmdLength; len++) {
		cmdPtr = CmdDic;
		matchCount = 0;
		while (cmdPtr->name != NULL)
		{
			if (strncmp(cmdPtr->name, cmd->Buffer, len) == 0) {
				matchCount++;
				matched = cmdPtr;
			}
			cmdPtr++;
		}
		// check if only one command matched or not
		if (matchCount > 1) {
			continue;
		}
		else 
		{
			if (matchCount == 1 && cmd->CmdLength <= strlen(matched->name) && strncmp(matched->name, cmd->Buffer, cmd->CmdLength) == 0) {
				cmd->func = matched->func;
				matched->func(cmd);
				osMessagePut(RcvBoxId, (uint32_t)cmd, 0);
			} else {
				PutStr("SYNTAX ERROR\r\n");
			}
			break;
		}
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
				PutStr(MSG_CRLF);
			  if (cmdBufPtr->Length > 0) {
					cmdBufPtr->Buffer[cmdBufPtr->Length] = '\0';
					LookupCommand(cmdBufPtr);
					currentCmdIdx = (currentCmdIdx + 1 ) % MAX_CMD_BUF_COUNT;
					cmdBufPtr = &CmdBuf[currentCmdIdx];
					cmdBufPtr->Length = 0;
				}
				break;
			case '\b':
				if (cmdBufPtr->Length > 0) {
					cmdBufPtr->Length--;
					PutStr("\b \b");
				}
				break;
			case ' ':
			case '\t':
				// skip space character at line top
				if (cmdBufPtr->Length == 0) {
					break;
				}
			default:
				AsciiToLed(*p);
				PutChr(*p);
				// capitalize
			  if (*p >= 'a' && *p <= 'z') {
					*p = *p - ('a' - 'A');
				}
				cmdBufPtr->Buffer[cmdBufPtr->Length] = *p;
				cmdBufPtr->Length++;
		}
		p++;
	}
}

/*****END OF FILE****/
