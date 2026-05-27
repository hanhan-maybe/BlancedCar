#ifndef __PARAM_STORE_H
#define __PARAM_STORE_H

#include "stm32f10x.h"

/* 参数状态显示变量 */
extern volatile uint8_t ParamSaveStatus;
extern volatile uint8_t ParamLoadStatus;
extern volatile uint8_t ParamVerifyStatus;

/* 从 Flash 加载参数
 * 返回 1：加载成功
 * 返回 0：Flash 中无有效参数，继续使用 AppData.c 默认参数
 */
uint8_t ParamStore_Load(void);

/* 验证 Flash 中保存的参数是否与当前 RAM 参数一致 */
uint8_t ParamStore_VerifyCurrent(void);

/* 将当前 RAM 中的 PID 参数保存到 Flash
 * 返回 1：保存成功
 * 返回 0：Flash 擦写失败
 */
uint8_t ParamStore_Save(void);

/* 恢复 RAM 中的默认参数
 * 注意：这里只恢复 RAM，不擦除 Flash。
 * 如果需要把默认值写入 Flash，需要再发送 [save,param]。
 */
void ParamStore_ResetToDefault(void);

#endif
