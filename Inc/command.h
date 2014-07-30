/**
  ******************************************************************************
  * @file           : COMMAND
  * @brief          : Header for Command Interpreter.
  ******************************************************************************
  * COPYRIGHT(c) 2014 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice,
  * this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  * this list of conditions and the following disclaimer in the documentation
  * and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of its contributors
  * may be used to endorse or promote products derived from this software
  * without specific prior written permission.
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
  * COPYRIGHT(c) 2014 Y.Magara
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMAND_H
#define __COMMAND_H
#include "cmsis_os.h"
#include "usbd_cdc_buf.h"

#define VERSION_STR "LegoArm version 0.1"

#define MAX_COMMAND_LENGTH 127
#define MAX_CMD_BUF_COUNT	3
typedef struct CommandBufferDef {
	uint32_t Length;
	char Buffer[MAX_COMMAND_LENGTH + 1];
	uint32_t CmdLength;
	char *Arg;
	void (*func)(struct CommandBufferDef *cmd);
} CommandBufferDef;

// card position index
typedef enum {
	Index_Ant = 0,
	Index_Home = 1,
	Index_A = 2,
	Index_B = 3,
	Index_C = 4,
	Index_D = 5,
	Index_F = Index_C,
	Index_V = Index_D,
	Index_MIN_CARD = Index_A,
	Index_MAX_CARD = Index_D
} TrayIndex;

#define PWM_ARM_UP (1500-1)
#define PWM_ARM_DOWN (1300-1)
#define SERVO_WAIT_DEFAULT_MS 400

extern osMessageQId  CmdBoxId;

extern void ParseInputChars(UsbUserBufferDef *rxPtr);
extern void StartMotorThread(void const * argument);

#endif /* __COMMAND_H */
