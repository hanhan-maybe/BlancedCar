#include "ParamStore.h"
#include "AppData.h"

#include <string.h>

/* =========================================================
 * Flash 参数区地址
 *
 * STM32F103C8T6 常见 Flash 为 64 KB；
 * 最后一页大小为 1 KB；
 * 最后一页起始地址为 0x0800FC00。
 *
 * 注意：
 * 1. 需要确认程序代码没有占用最后一页；
 * 2. 可以通过 Keil 编译后的 Program Size 或 map 文件确认。
 * ========================================================= */
#define PARAM_FLASH_ADDR    0x0800FC00u

/* 参数有效性标记 */
#define PARAM_MAGIC         0x42435031u
#define PARAM_VERSION       1u

/* =========================================================
 * 参数保存/加载/验证状态
 *
 * 0：未执行或未知
 * 1：成功
 * 2：失败
 * ========================================================= */
volatile uint8_t ParamSaveStatus = 0;    // 0 未保存/未知，1 保存成功，2 保存失败
volatile uint8_t ParamLoadStatus = 0;    // 0 未加载/未知，1 加载成功，2 加载失败
volatile uint8_t ParamVerifyStatus = 0;  // 0 未验证，1 验证成功，2 验证失败

/* =========================================================
 * 浮点数近似比较
 *
 * Flash 中保存的是 float，读回后理论上应完全一致；
 * 这里仍使用很小的误差范围，避免调试时受显示精度影响。
 * ========================================================= */
static uint8_t ParamStore_FloatEqual(float a, float b)
{
    float diff = a - b;

    if (diff < 0.0f)
    {
        diff = -diff;
    }

    return (diff < 0.0001f);
}

/* =========================================================
 * Flash 参数记录结构体
 *
 * checksum 放在最后；
 * 计算校验和时排除 checksum 字段本身。
 * ========================================================= */
typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t length;

    float AngleKp;
    float AngleKi;
    float AngleKd;
    float AngleOutOffset;

    float SpeedKp;
    float SpeedKi;
    float SpeedKd;

    float TurnKp;
    float TurnKi;
    float TurnKd;

    float BalanceAngleTrimValue;

    uint32_t checksum;
} ParamRecord_t;

/* =========================================================
 * 简单字节累加校验
 *
 * 作用：
 * 判断 Flash 中参数是否完整有效。
 * 当前项目参数量较小，使用累加校验即可。
 * ========================================================= */
static uint32_t ParamStore_CalcChecksum(const ParamRecord_t *record)
{
    const uint8_t *data = (const uint8_t *)record;
    uint32_t sum = 0;
    uint16_t i;

    for (i = 0; i < (sizeof(ParamRecord_t) - sizeof(uint32_t)); i++)
    {
        sum += data[i];
    }

    return sum;
}

/* =========================================================
 * 将 Flash 记录应用到当前 PID 参数
 * ========================================================= */
static void ParamStore_ApplyRecord(const ParamRecord_t *record)
{
    AnglePID.Kp = record->AngleKp;
    AnglePID.Ki = record->AngleKi;
    AnglePID.Kd = record->AngleKd;
    AnglePID.OutOffset = record->AngleOutOffset;

    SpeedPID.Kp = record->SpeedKp;
    SpeedPID.Ki = record->SpeedKi;
    SpeedPID.Kd = record->SpeedKd;

    TurnPID.Kp = record->TurnKp;
    TurnPID.Ki = record->TurnKi;
    TurnPID.Kd = record->TurnKd;

    BalanceAngleTrim = record->BalanceAngleTrimValue;
}

uint8_t ParamStore_Load(void)
{
    const ParamRecord_t *record = (const ParamRecord_t *)PARAM_FLASH_ADDR;
    uint32_t checksum;

     /*
     * 默认认为加载失败。
     * 只有所有校验都通过并成功应用参数后，才改为 1。
     */
    ParamLoadStatus = 2;

    /* 检查 magic，判断 Flash 中是否写入过参数 */
    if (record->magic != PARAM_MAGIC)
    {
        return 0;
    }

    /* 检查版本，避免后续结构体升级后误读旧数据 */
    if (record->version != PARAM_VERSION)
    {
        return 0;
    }

    /* 检查结构体长度 */
    if (record->length != sizeof(ParamRecord_t))
    {
        return 0;
    }

    /* 检查 checksum */
    checksum = ParamStore_CalcChecksum(record);

    if (checksum != record->checksum)
    {
        return 0;
    }

    /* 校验通过后再应用参数 */
    ParamStore_ApplyRecord(record);
    /*
     * 6. 记录加载成功状态。
     */
    ParamLoadStatus = 1;

    return 1;
}

uint8_t ParamStore_Save(void)
{
    ParamRecord_t record;
    FLASH_Status status;
    uint16_t i;
    uint16_t *pData;

    memset(&record, 0, sizeof(record));

    /* 填写参数记录头 */
    record.magic = PARAM_MAGIC;
    record.version = PARAM_VERSION;
    record.length = sizeof(ParamRecord_t);

    /* 保存当前三环 PID 参数 */
    record.AngleKp = AnglePID.Kp;
    record.AngleKi = AnglePID.Ki;
    record.AngleKd = AnglePID.Kd;
    record.AngleOutOffset = AnglePID.OutOffset;

    record.SpeedKp = SpeedPID.Kp;
    record.SpeedKi = SpeedPID.Ki;
    record.SpeedKd = SpeedPID.Kd;

    record.TurnKp = TurnPID.Kp;
    record.TurnKi = TurnPID.Ki;
    record.TurnKd = TurnPID.Kd;

    /* 保存静态平衡角补偿 */
    record.BalanceAngleTrimValue = BalanceAngleTrim;

    /* 计算校验和 */
    record.checksum = ParamStore_CalcChecksum(&record);

    FLASH_Unlock();

    /* 擦除参数页 */
    status = FLASH_ErasePage(PARAM_FLASH_ADDR);

    if (status != FLASH_COMPLETE)
{
    FLASH_Lock();
    ParamSaveStatus = 2;
    return 0;
}

    if (status != FLASH_COMPLETE)
    {
        FLASH_Lock();
        return 0;
    }

    /* 按半字写入 Flash */
    pData = (uint16_t *)&record;

    for (i = 0; i < sizeof(ParamRecord_t) / 2; i++)
    {
        status = FLASH_ProgramHalfWord(
            PARAM_FLASH_ADDR + i * 2,
            pData[i]
        );

        if (status != FLASH_COMPLETE)
        {
            FLASH_Lock();
            return 0;
        }
    }

    FLASH_Lock();

    /* =========================================================
    * 写入完成后，立即从 Flash 读回并验证。
    * 只有验证通过，才认为本次保存真正成功。
    * ========================================================= */
    if (ParamStore_VerifyCurrent())
    {
        ParamSaveStatus = 1;
        return 1;
    }
    else
    {
        ParamSaveStatus = 2;
        return 0;
    }

}

void ParamStore_ResetToDefault(void)
{
    /* =====================================================
     * 恢复当前 AppData.c 中的默认参数。
     * 这里只修改 RAM，不擦除 Flash。
     * ===================================================== */

    AnglePID.Kp = 1.55f;
    AnglePID.Ki = 0.1f;
    AnglePID.Kd = 6.0f;
    AnglePID.OutOffset = 3.0f;

    SpeedPID.Kp = 2.2f;
    SpeedPID.Ki = 0.1f;
    SpeedPID.Kd = 0.0f;

    TurnPID.Kp = 5.5f;
    TurnPID.Ki = 0.07f;
    TurnPID.Kd = 0.0f;

    BalanceAngleTrim = 2.0f;
}
/*验证函数*/
uint8_t ParamStore_VerifyCurrent(void)
{
    const ParamRecord_t *record = (const ParamRecord_t *)PARAM_FLASH_ADDR;
    uint32_t checksum;

    /* 1. 检查基本有效性 */
    if (record->magic != PARAM_MAGIC)
    {
        ParamVerifyStatus = 2;
        return 0;
    }

    if (record->version != PARAM_VERSION)
    {
        ParamVerifyStatus = 2;
        return 0;
    }

    if (record->length != sizeof(ParamRecord_t))
    {
        ParamVerifyStatus = 2;
        return 0;
    }

    checksum = ParamStore_CalcChecksum(record);

    if (checksum != record->checksum)
    {
        ParamVerifyStatus = 2;
        return 0;
    }

    /* 2. 比较 Flash 数据与当前 RAM 参数是否一致 */
    if (!ParamStore_FloatEqual(record->AngleKp, AnglePID.Kp)) goto verify_fail;
    if (!ParamStore_FloatEqual(record->AngleKi, AnglePID.Ki)) goto verify_fail;
    if (!ParamStore_FloatEqual(record->AngleKd, AnglePID.Kd)) goto verify_fail;
    if (!ParamStore_FloatEqual(record->AngleOutOffset, AnglePID.OutOffset)) goto verify_fail;

    if (!ParamStore_FloatEqual(record->SpeedKp, SpeedPID.Kp)) goto verify_fail;
    if (!ParamStore_FloatEqual(record->SpeedKi, SpeedPID.Ki)) goto verify_fail;
    if (!ParamStore_FloatEqual(record->SpeedKd, SpeedPID.Kd)) goto verify_fail;

    if (!ParamStore_FloatEqual(record->TurnKp, TurnPID.Kp)) goto verify_fail;
    if (!ParamStore_FloatEqual(record->TurnKi, TurnPID.Ki)) goto verify_fail;
    if (!ParamStore_FloatEqual(record->TurnKd, TurnPID.Kd)) goto verify_fail;

    if (!ParamStore_FloatEqual(record->BalanceAngleTrimValue, BalanceAngleTrim)) goto verify_fail;

    ParamVerifyStatus = 1;
    return 1;

verify_fail:
    ParamVerifyStatus = 2;
    return 0;
}
