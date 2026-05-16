#include "stm32f10x.h"                  // Device header
#include "Delay.h"

uint8_t Key_Num;

void Key_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

uint8_t Key_GetNum(void)
{
	uint8_t Temp;
	if (Key_Num)
	{
		Temp = Key_Num;
		Key_Num = 0;
		return Temp;
	}
	return 0;
}

uint8_t Key_GetState(void)
{
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)
	{
		return 1;
	}
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0)
	{
		return 2;
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0)
	{
		return 3;
	}
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == 0)
	{
		return 4;
	}
	return 0;
}

//松手触发的按键消抖程序
//改良非阻塞式按键检测，每1 ms采样一次，状态连续稳定20 ms后再确认按键状态变化
void Key_Tick(void)
{
    static uint8_t LastRawState = 0;     // 上一次原始采样状态
    static uint8_t StableState = 0;      // 已确认的稳定状态
    static uint8_t DebounceCount = 0;    // 稳定计数

    uint8_t RawState = Key_GetState();

    /* 如果当前原始状态与上次不同，说明可能正在抖动 */
    if (RawState != LastRawState)
    {
        LastRawState = RawState;
        DebounceCount = 0;
    }
    else
    {
        /* 原始状态持续稳定 */
        if (DebounceCount < 20)
        {
            DebounceCount++;
        }

        /* 稳定达到20ms，且与已确认状态不同，确认发生状态变化 */
        if (DebounceCount >= 20 && RawState != StableState)
        {
            uint8_t PrevStableState = StableState;
            StableState = RawState;

            /* 松手触发：之前有按键，现在无按键 */
            if (StableState == 0 && PrevStableState != 0)
            {
                Key_Num = PrevStableState;
            }
        }
    }
}
