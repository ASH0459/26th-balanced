#define BSP_DWT_H

#include "main.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    uint32_t s;
    uint16_t ms;
    uint16_t us;
} DWT_Time_t;

void DWT_Init(uint32_t CPU_Freq_mHz);
fp32 DWT_GetDeltaT(uint32_t *cnt_last);
fp64 DWT_GetDeltaT64(uint32_t *cnt_last);
fp32 DWT_GetTimeline_s(void);
fp32 DWT_GetTimeline_ms(void);
uint64_t DWT_GetTimeline_us(void);
void DWT_Delay(fp32 Delay);
void DWT_SysTimeUpdate(void);

extern DWT_Time_t SysTime;

#ifdef __cplusplus
}
#endif
