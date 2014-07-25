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
#include "tim.h"
#include "usbd_cdc_buf.h"
#include "command.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN 4 */

#define MSG_CRLF "\r\n"
#define MSG_EMPTY_ARGUMENT "Empty argument.\r\n"
#define MAG_INVALID_PARAMETER "Invalid parameter.\r\n"

static CommandBufferDef CmdBuf[MAX_CMD_BUF_COUNT];
static uint16_t currentCmdIdx;

static const uint8_t HexChr[] = "0123456789ABCDEF";
struct {
	GPIO_TypeDef* GPIOx;
	uint16_t GPIO_Pin;
} static const GpioLedMap[8] = {
	{GPIOE, GPIO_PIN_9},		/* LD3 */
	{GPIOE, GPIO_PIN_8},		/* LD4 */
	{GPIOE, GPIO_PIN_15},		/* LD6 */
	{GPIOE, GPIO_PIN_14},		/* LD8 */
	{GPIOE, GPIO_PIN_13},		/* LD10 */
	{GPIOE, GPIO_PIN_12},		/* LD9 */
	{GPIOE, GPIO_PIN_11},		/* LD7 */
	{GPIOE, GPIO_PIN_10},		/* LD5 */
};

static void cmdVersion(CommandBufferDef *cmd);
static void cmdRelocate(CommandBufferDef *cmd);
static void cmdStepRotate(CommandBufferDef *cmd);
static void cmdPutOn(CommandBufferDef *cmd);
static void cmdTakeOff(CommandBufferDef *cmd);
static void cmdHelp(CommandBufferDef *cmd);

typedef struct  {
	const char *const name;
	void (*const func)(CommandBufferDef *cmd);
} CommandOp;

static const CommandOp CmdDic[] = {
	{"PUTON", cmdPutOn},
	{"TAKEOFF", cmdTakeOff},
	{"RELOCATE", cmdRelocate},
	{"STEP", cmdStepRotate},
	{"HELP", cmdHelp},
	{"VERSION", cmdVersion},
	{NULL, NULL}
};

// turn table angle: [0..95]
#define STEP_PER_REV	96
#define STEP_PER_30DEG	(STEP_PER_REV/12)

// PM step correspnding to TrayAngle
static const int16_t TrayStepPos[] = {
	-STEP_PER_30DEG,
	0,
	STEP_PER_30DEG, 
	2*STEP_PER_30DEG,
	3*STEP_PER_30DEG, 
	4*STEP_PER_30DEG
};
static int16_t currStepPos;
static uint32_t currServoPos;

#define I2C_ADDR_PH_A_w	(0xC6)
#define I2C_ADDR_PH_B_w	(0xC2)
#define I2C_SUB_CTRL	0
#define VSET_MIN	0x06
#define VSET_MAX	0x3F
#define OUT_POS_MAX	((VSET_MAX<<2)+1)
#define OUT_NEG_MAX	((VSET_MAX<<2)+2)
#define OUT_POS_MIN	((VSET_MIN<<2)+1)
#define OUT_NEG_MIN	((VSET_MIN<<2)+2)

#define PHASE_COUNT	4
// [PAHSE][A/B][data]
static const uint8_t PM_PHASE[PHASE_COUNT][2] = {
	// phase1
	{1, 0}
	// phase2
	, {1,	1}
	// phase3
	, {0,	1}
	// phase4
	, {0,	0}
};
static int16_t currentPhase;
#define INTER_PHASE_COUNTUP_WIDTH 8
#define INTER_PHASE_COUNTDOWN_WIDTH 8
#define INTER_PHASE_DELAY_MAX_MS	30
#define INTER_PHASE_DELAY_MIN_MS	5
#define INTER_PHASE_DELAY_RANGE (INTER_PHASE_DELAY_MAX_MS-INTER_PHASE_DELAY_MIN_MS)

#define GPIO_PIN_VACUUM_PUMP GPIO_PIN_10
#define VACUUM_ON_DELAY_MS 1500
#define VACUUM_OFF_DELAY_MS 500

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
	uint16_t retry = 1000;
	while (CDC_Transmit_FS(txBufPtr->Buffer, len) == USBD_BUSY && retry-- > 0) {
		osDelay(1);
	}
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
	uint16_t retry = 1000;
	while (CDC_Transmit_FS(txBufPtr->Buffer, 1) == USBD_BUSY && retry-- > 0) {
		osDelay(1);
	}
	idxTxBuffer = (idxTxBuffer + 1) % TX_BUFFER_COUNT;
}

static void PutUint16(uint16_t value)
{
	for (int s = 12; s >= 0; s -= 4) {
		PutChr(HexChr[0x0F & (value >> s)]);
	}
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

/**
* devaddr: target device address.
  */
static void FaderStep(uint16_t devAddr, int16_t startPolarity, uint32_t stepdelayInterval)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t step = 0x04;
	uint8_t data[2];
	uint8_t start, goal;
	data[0] = I2C_SUB_CTRL;

	// fade out
	start = (VSET_MAX<<2);
	goal = (VSET_MIN<<2);
	if (startPolarity == 0) {
		start += 1;
		goal += 1;
	} else {
		start += 2;
		goal += 2;
	}
	for (uint8_t value = start; value >= goal; value -= step)
	{
		data[1] = value;
		status = HAL_I2C_Master_Transmit(&hi2c1, devAddr, data, 2, 10);
		if (status != HAL_OK) {
			break;
		}
		if (stepdelayInterval > 0 && (value % stepdelayInterval) == 0) osDelay(1);
	}

	// fade in
	start = (VSET_MIN<<2);
	goal = (VSET_MAX<<2);
	if (startPolarity == 0) {
		start += 2;
		goal += 2;
	} else {
		start += 1;
		goal += 1;
	}
	for (uint16_t value = start; value < goal; value += step)
	{
		data[1] = (0xFF & value);
		status = HAL_I2C_Master_Transmit(&hi2c1, devAddr, data, 2, 10);
		if (status != HAL_OK) {
			break;
		}
		if (stepdelayInterval > 0 && (value % stepdelayInterval) == 0) osDelay(1);
	}
}

static void TurnTable(int16_t dst)
{
	int16_t step = 0;
	int16_t countdown = 0;
	int16_t countup = 0;
	if (dst > currStepPos) {
		step = 1;
		countdown = dst - currStepPos;
	} else if (dst < currStepPos) {
		step = -1;
		countdown = currStepPos - dst;
	} else {
		return;
	}

	uint32_t phasedelay = INTER_PHASE_DELAY_MAX_MS;
	uint32_t stepdelayInterval = 1;
	uint8_t currA = PM_PHASE[currentPhase][0];
	uint8_t currB = PM_PHASE[currentPhase][1];
	uint8_t nextA, nextB;
	while (countdown != 0) {
		stepdelayInterval = MIN(countdown, countup);
		if (countdown < INTER_PHASE_COUNTDOWN_WIDTH || countup < INTER_PHASE_COUNTUP_WIDTH) {
			phasedelay = MIN(countdown,countup) * INTER_PHASE_DELAY_RANGE / INTER_PHASE_COUNTDOWN_WIDTH + INTER_PHASE_DELAY_MIN_MS;
		} else {
			phasedelay = INTER_PHASE_DELAY_MIN_MS;
		}
		int16_t nextPhase = (currentPhase + step) % PHASE_COUNT;
		if (nextPhase < 0) {
			nextPhase += PHASE_COUNT;
		}
		nextA = PM_PHASE[nextPhase][0];
		nextB = PM_PHASE[nextPhase][1];
		if (currA != nextA) {
			FaderStep(I2C_ADDR_PH_A_w, currA, stepdelayInterval);
			currA = nextA;
		}
		if (currB != nextB) {
			FaderStep(I2C_ADDR_PH_B_w, currB, stepdelayInterval);
			currB = nextB;
		}
		currentPhase = nextPhase;
		currStepPos += step;
		countdown--;
		countup++;
		osDelay(phasedelay);
	}
}

void MoveServo(uint32_t pulse)
{
  static TIM_OC_InitTypeDef sConfigOC;
	uint32_t step = (pulse >= currServoPos ? 1 : -1);
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	for (;;) {
		sConfigOC.Pulse = currServoPos;
		HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		 if (currServoPos == pulse) break;
		currServoPos += step;
		osDelay(3);
	};

	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
}

static void Vacuum(GPIO_PinState isVacuum, uint32_t millisec)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_VACUUM_PUMP, isVacuum);
	osDelay(millisec);
}

/**
	* 0: home, 1:A, 2:B, 3:C, 4:D
	*/
static void MoveCard(TrayIndex start, TrayIndex end)
{
	if (start > Index_MAX_CARD) {
		PutStr("Invalid start position.\r\n");
		return;
	}
	if (end > Index_MAX_CARD) {
		PutStr("Invalid end position.\r\n");
		return;
	}
	
	// lift up arm
	MoveServo(PWM_ARM_UP);
	// turn arm to FROM position
	TurnTable(TrayStepPos[start]);
	// lift down arm
	MoveServo(PWM_ARM_DOWN);
	// vacuum on
	Vacuum(GPIO_PIN_SET, VACUUM_ON_DELAY_MS);
	// lift up arm
	MoveServo(PWM_ARM_UP);
	// turn arm to TO position
	TurnTable(TrayStepPos[end]);
	// lift down arm
	MoveServo(PWM_ARM_DOWN);
	// vacuum off
	Vacuum(GPIO_PIN_RESET, VACUUM_OFF_DELAY_MS);
	// lift up arm
	MoveServo(PWM_ARM_UP);
	// turn arm to home position
	TurnTable(TrayStepPos[Index_Home]);
}

/**
  * Print version number.
  */
static void cmdVersion(CommandBufferDef *cmd)
{
	PutStr(VERSION_STR);
	PutStr(MSG_CRLF);
}

/**
  * Re-scan mechanical limit position.
  */
static void cmdRelocate(CommandBufferDef *cmd)
{
	PutStr("Re-scan mechanical limit position.\r\n");
	// lift up arm
	MoveServo(PWM_ARM_UP);
	// turn arm to right while mechanical limit
	//    sense photo reflector
	TurnTable(TrayStepPos[Index_Ant]);
	// turn arm to home position
	TurnTable(TrayStepPos[Index_Home]);
}

static void cmdStepRotate(CommandBufferDef *cmd)
{
	if (cmd->Arg == NULL) {
		PutStr(MSG_EMPTY_ARGUMENT);
	}
	if (cmd->Arg[0] == 'L') {
		TurnTable(currStepPos + 1);
		currStepPos--;
	} else if (cmd->Arg[0] == 'R') {
		TurnTable(currStepPos - 1);
		currStepPos++;
	} else {
		PutStr(MAG_INVALID_PARAMETER);
	}
}

/**
  * Put a card on the RF antenna.
	*
	* PUTON <A/B/C/D>
  */
static void cmdPutOn(CommandBufferDef *cmd)
{
	if (cmd->Arg == NULL) {
		PutStr(MSG_EMPTY_ARGUMENT);
	}
	// 0: home, 1:A, 2:B, 3:C, 4:D
	TrayIndex card = Chr2CardNo(cmd->Arg[0]);
	if (card >= Index_MIN_CARD && card <= Index_MAX_CARD) {
		PutChr(card + '0');
		PutStr(" -> 0\r\n");
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
static void cmdTakeOff(CommandBufferDef *cmd)
{
	if (cmd->Arg == NULL) {
		PutStr(MSG_EMPTY_ARGUMENT);
	}
	// 0: home, 1:A, 2:B, 3:C, 4:D
	TrayIndex card = Chr2CardNo(cmd->Arg[0]);
	if (card >= Index_MIN_CARD && card <= Index_MAX_CARD) {
		PutStr("0 -> ");
		PutChr(card + '0');
		PutStr("\r\n");
		MoveCard(Index_Ant, card);
	} else {
		PutStr(MAG_INVALID_PARAMETER);
	}
}

/**
  * Show command help.
  */
static void cmdHelp(CommandBufferDef *cmd)
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
	currServoPos = 1350;
	MoveServo(PWM_ARM_UP);
//	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
//	osDelay(500);
//	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  /* Infinite loop */
  for(;;)
  {
    evt = osMessageGet(CmdBoxId, osWaitForever);
		if (evt.status == osEventMessage) {
			cmdBuf = evt.value.p;
			cmdBuf->func(cmdBuf);
			PutStr("OK\r\n");
		}
		//check received length, read UserRxBufferFS
  }
}
static void LookupCommand(CommandBufferDef *cmd)
{
	const CommandOp *cmdPtr, *matched;
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
				osMessagePut(CmdBoxId, (uint32_t)cmd, 0);
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
