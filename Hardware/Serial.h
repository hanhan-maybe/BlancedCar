#ifndef __SERIAL_H
#define __SERIAL_H

#include "stm32f10x.h"

/* =====================================================
 * USART1 接收方式选择
 * 0：RXNE 中断逐字节接收
 * 1：DMA 环形缓冲区接收
 * ===================================================== */
#define SERIAL_RX_USE_DMA      1

#define SERIAL_DMA_RX_SIZE     128

extern uint8_t Serial_RxData;
extern uint8_t Serial_RxFlag;

/* CPU 测试用统计变量 */
extern volatile uint32_t Serial_RxByteCount;
extern volatile uint32_t Serial_RxIrqCount;
extern volatile uint32_t Serial_DmaPollCount;
extern volatile uint32_t Serial_RxDropCount;
extern volatile uint16_t Serial_DmaWritePos;
extern volatile uint16_t Serial_DmaLastPos;

void Serial_Init(void);

void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_Printf(char *format, ...);

uint8_t Serial_GetRxFlag(void);
uint8_t Serial_GetRxData(void);

/* DMA 模式下需要周期调用 */
void Serial_PollDMA(void);

#endif
