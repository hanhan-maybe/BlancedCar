#ifndef __BLUESERIAL_H
#define __BLUESERIAL_H

#include "stm32f10x.h"

#define BLUESERIAL_PACKET_SIZE 100
#define BLUESERIAL_DMA_RX_SIZE      128

/* 1：使用 DMA 接收；0：使用 RXNE 中断接收 */
#define BLUESERIAL_RX_USE_DMA       1

/* DMA 接收调试变量 */
extern volatile uint32_t BlueSerial_RxByteCount;
extern volatile uint32_t BlueSerial_RxPacketCount;
extern volatile uint32_t BlueSerial_RxDropCount;
extern volatile uint16_t BlueSerial_DmaWritePos;
extern volatile uint16_t BlueSerial_DmaLastPos;


extern char BlueSerial_RxPacket[BLUESERIAL_PACKET_SIZE];
extern uint8_t BlueSerial_RxFlag;

void BlueSerial_Init(void);
void BlueSerial_SendByte(uint8_t Byte);
void BlueSerial_SendArray(uint8_t *Array, uint16_t Length);
void BlueSerial_SendString(char *String);
void BlueSerial_SendNumber(uint32_t Number, uint8_t Length);
void BlueSerial_Printf(char *format, ...);

/* 新增：安全读取完整接收帧 */
uint8_t BlueSerial_GetRxPacket(char *Buffer, uint16_t BufferSize);

/* 新增：DMA轮询解析函数 */
void BlueSerial_PollDMA(void);

#endif
