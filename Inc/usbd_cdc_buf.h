/**
  ******************************************************************************
  * @file           : USB_CDC_BUF
  * @brief          : Header for bufferes to send/receive usb data.
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
#ifndef __USBD_CDC_BUF_H
#define __USBD_CDC_BUF_H
#include "cmsis_os.h"
#include "usbd_cdc_if.h"

#define RX_BUFFER_COUNT	2
#define TX_BUFFER_COUNT	2
typedef struct {
	volatile uint32_t Length;
	uint8_t Buffer[CDC_DATA_FS_MAX_PACKET_SIZE];
} UsbUserBufferDef;
/* Send Data over USB CDC are stored in this buffer       */
extern UsbUserBufferDef UsbUserTxBuffer[];
extern uint16_t idxTxBuffer;
/* Received Data over USB are stored in this buffer       */
extern UsbUserBufferDef UsbUserRxBuffer[];

extern osMessageQId  RcvBoxId;

#endif /* __USBD_CDC_BUF_H */
