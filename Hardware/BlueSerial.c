#include "stm32f10x.h"                  // Device header
#include "BlueSerial.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* DMA 接收调试变量 */
char BlueSerial_RxPacket[BLUESERIAL_PACKET_SIZE];
uint8_t BlueSerial_RxFlag;

#if BLUESERIAL_RX_USE_DMA
static uint8_t BlueSerial_DmaRxBuffer[BLUESERIAL_DMA_RX_SIZE];
#endif

volatile uint32_t BlueSerial_RxByteCount = 0;
volatile uint32_t BlueSerial_RxPacketCount = 0;
volatile uint32_t BlueSerial_RxDropCount = 0;
volatile uint16_t BlueSerial_DmaWritePos = 0;
volatile uint16_t BlueSerial_DmaLastPos = 0;


char BlueSerial_RxPacket[BLUESERIAL_PACKET_SIZE];
uint8_t BlueSerial_RxFlag;

void BlueSerial_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

#if BLUESERIAL_RX_USE_DMA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif

    GPIO_InitTypeDef GPIO_InitStructure;

    /* USART2_TX PA2 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART2_RX PA3 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);

#if BLUESERIAL_RX_USE_DMA

    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Channel6);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BlueSerial_DmaRxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = BLUESERIAL_DMA_RX_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel6, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel6, ENABLE);
    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);

#else

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

#endif

    USART_Cmd(USART2, ENABLE);
}


uint8_t BlueSerial_GetRxPacket(char *Buffer, uint16_t BufferSize)
{
    uint8_t Ret = 0;

    if (Buffer == 0 || BufferSize == 0)
    {
        return 0;
    }

    taskENTER_CRITICAL();

    if (BlueSerial_RxFlag == 1)
    {
        strncpy(Buffer, BlueSerial_RxPacket, BufferSize - 1);
        Buffer[BufferSize - 1] = '\0';

        BlueSerial_RxFlag = 0;
        Ret = 1;
    }

    taskEXIT_CRITICAL();

    return Ret;
}



void BlueSerial_SendByte(uint8_t Byte)
{
	USART_SendData(USART2, Byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

void BlueSerial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Array[i]);
	}
}

void BlueSerial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		BlueSerial_SendByte(String[i]);
	}
}

uint32_t BlueSerial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void BlueSerial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		BlueSerial_SendByte(Number / BlueSerial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

void BlueSerial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	BlueSerial_SendString(String);
}

/* =========================================================
 * 以下是新增的协议解析函数
 * ========================================================= */
static void BlueSerial_ParseByte(uint8_t RxData)
{
    static uint8_t RxState = 0;
    static uint8_t pRxPacket = 0;

    BlueSerial_RxByteCount++;

    if (RxState == 0)
    {
        if (RxData == '[' && BlueSerial_RxFlag == 0)
        {
            RxState = 1;
            pRxPacket = 0;
        }
    }
    else if (RxState == 1)
    {
        if (RxData == ']')
        {
            RxState = 0;
            BlueSerial_RxPacket[pRxPacket] = '\0';
            BlueSerial_RxFlag = 1;
            BlueSerial_RxPacketCount++;
        }
        else
        {
            if (pRxPacket < BLUESERIAL_PACKET_SIZE - 1)
            {
                BlueSerial_RxPacket[pRxPacket] = RxData;
                pRxPacket++;
            }
            else
            {
                RxState = 0;
                pRxPacket = 0;
                BlueSerial_RxDropCount++;
            }
        }
    }
}

/* DMA 轮询解析函数，适用于 BLUESERIAL_RX_USE_DMA = 1 的情况 */
void BlueSerial_PollDMA(void)
{
#if BLUESERIAL_RX_USE_DMA
    uint16_t WritePos;

    WritePos = BLUESERIAL_DMA_RX_SIZE - DMA_GetCurrDataCounter(DMA1_Channel6);
    BlueSerial_DmaWritePos = WritePos;

    while (BlueSerial_DmaLastPos != WritePos)
    {
        BlueSerial_ParseByte(BlueSerial_DmaRxBuffer[BlueSerial_DmaLastPos]);

        BlueSerial_DmaLastPos++;

        if (BlueSerial_DmaLastPos >= BLUESERIAL_DMA_RX_SIZE)
        {
            BlueSerial_DmaLastPos = 0;
        }
    }
#endif
}

/* USART2 RXNE 中断服务程序，适用于 BLUESERIAL_RX_USE_DMA = 0 的情况 */
#if !BLUESERIAL_RX_USE_DMA
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        uint8_t RxData = USART_ReceiveData(USART2);

        BlueSerial_ParseByte(RxData);

        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

#endif

