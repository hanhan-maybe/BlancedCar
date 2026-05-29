#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include <stdio.h>
#include <stdarg.h>

/* =====================================================
 * 原有接收变量
 * ===================================================== */
uint8_t Serial_RxData;
uint8_t Serial_RxFlag;

/* =====================================================
 * DMA 接收缓冲区
 * USART1_RX 对应 DMA1_Channel5
 * ===================================================== */
#if SERIAL_RX_USE_DMA
static uint8_t Serial_DmaRxBuffer[SERIAL_DMA_RX_SIZE];
#endif

/* =====================================================
 * CPU 测试用统计变量
 * ===================================================== */
volatile uint32_t Serial_RxByteCount = 0;    // 已接收字节数
volatile uint32_t Serial_RxIrqCount = 0;     // RXNE 中断次数
volatile uint32_t Serial_DmaPollCount = 0;   // DMA 轮询次数
volatile uint32_t Serial_RxDropCount = 0;    // 预留：丢弃计数
volatile uint16_t Serial_DmaWritePos = 0;    // DMA 当前写位置
volatile uint16_t Serial_DmaLastPos = 0;     // 上一次处理到的位置


/* =====================================================
 * 内部函数：统一处理接收到的 1 个字节
 *
 * RXNE 模式：
 *   USART1_IRQHandler() 每收到 1 字节调用一次
 *
 * DMA 模式：
 *   Serial_PollDMA() 从 DMA 缓冲区取出新增字节后调用
 * ===================================================== */
static void Serial_ProcessRxByte(uint8_t RxData)
{
    Serial_RxData = RxData;
    Serial_RxFlag = 1;

    Serial_RxByteCount++;
}


void Serial_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

#if SERIAL_RX_USE_DMA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
#endif

    GPIO_InitTypeDef GPIO_InitStructure;

    /* USART1_TX: PA9 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* USART1_RX: PA10 */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);

#if SERIAL_RX_USE_DMA

    /* =====================================================
     * USART1_RX DMA 配置
     * USART1_RX -> DMA1_Channel5
     * 方向：USART1->DR 到 Serial_DmaRxBuffer[]
     * ===================================================== */
    DMA_InitTypeDef DMA_InitStructure;

    DMA_DeInit(DMA1_Channel5);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Serial_DmaRxBuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = SERIAL_DMA_RX_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel5, &DMA_InitStructure);

    DMA_Cmd(DMA1_Channel5, ENABLE);

    /* 允许 USART1_RX 产生 DMA 请求 */
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

#else

    /* =====================================================
     * RXNE 中断模式
     * 每收到 1 个字节进入一次 USART1_IRQHandler
     * ===================================================== */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure);

#endif

    USART_Cmd(USART1, ENABLE);
}


void Serial_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}


void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    uint16_t i;

    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Array[i]);
    }
}


void Serial_SendString(char *String)
{
    uint8_t i;

    for (i = 0; String[i] != '\0'; i++)
    {
        Serial_SendByte(String[i]);
    }
}


uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;

    while (Y--)
    {
        Result *= X;
    }

    return Result;
}


void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;

    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}


int fputc(int ch, FILE *f)
{
    Serial_SendByte(ch);
    return ch;
}


void Serial_Printf(char *format, ...)
{
    char String[100];
    va_list arg;

    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);

    Serial_SendString(String);
}


uint8_t Serial_GetRxFlag(void)
{
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;
        return 1;
    }

    return 0;
}


uint8_t Serial_GetRxData(void)
{
    return Serial_RxData;
}


/* =====================================================
 * DMA 模式下周期调用
 *
 * 建议在 FreeRTOS 任务中每 5 ms 调用一次：
 *
 * while (1)
 * {
 *     Serial_PollDMA();
 *     vTaskDelay(pdMS_TO_TICKS(5));
 * }
 * ===================================================== */
void Serial_PollDMA(void)
{
#if SERIAL_RX_USE_DMA

    uint16_t WritePos;

    Serial_DmaPollCount++;

    /*
     * DMA_GetCurrDataCounter() 表示 DMA 缓冲区还剩多少个数据未写满。
     * 当前写位置 = 缓冲区大小 - 剩余计数
     */
    WritePos = SERIAL_DMA_RX_SIZE - DMA_GetCurrDataCounter(DMA1_Channel5);

    if (WritePos >= SERIAL_DMA_RX_SIZE)
    {
        WritePos = 0;
    }

    Serial_DmaWritePos = WritePos;

    /*
     * 从上次处理位置 Serial_DmaLastPos
     * 追到当前 DMA 写位置 WritePos
     */
    while (Serial_DmaLastPos != WritePos)
    {
        Serial_ProcessRxByte(Serial_DmaRxBuffer[Serial_DmaLastPos]);

        Serial_DmaLastPos++;

        if (Serial_DmaLastPos >= SERIAL_DMA_RX_SIZE)
        {
            Serial_DmaLastPos = 0;
        }
    }

#endif
}


/* =====================================================
 * RXNE 中断模式下使用
 *
 * DMA 模式下不定义 USART1_IRQHandler，
 * 避免重复接收和重复统计。
 * ===================================================== */
#if !SERIAL_RX_USE_DMA

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
    {
        uint8_t RxData;

        RxData = USART_ReceiveData(USART1);

        Serial_RxIrqCount++;
        Serial_ProcessRxByte(RxData);

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

#endif
