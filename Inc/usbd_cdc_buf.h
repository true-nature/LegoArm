
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CDC_BUF_H
#define __USBD_CDC_BUF_H

/* USER CODE BEGIN 1 */

#define RX_BUFFER_COUNT	2
#define TX_BUFFER_COUNT	2
typedef struct {
	volatile uint32_t Length;
	uint8_t Buffer[CDC_DATA_HS_MAX_PACKET_SIZE];
} UsbUserBufferDef;
/* Send Data over USB CDC are stored in this buffer       */
extern UsbUserBufferDef UsbUserTxBuffer[];
/* Received Data over USB are stored in this buffer       */
extern UsbUserBufferDef UsbUserRxBuffer[];

extern volatile osMessageQId  RcvBoxId;
/* USER CODE END 1 */

#endif /* __USBD_CDC_BUF_H */
