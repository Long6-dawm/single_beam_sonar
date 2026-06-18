#ifndef __DSP_PROCESS_H
#define __DSP_PROCESS_H

#include "arm_math.h"

#define FFT_LEN 4096
#define FS 500000.0f
#define V_SOUND 1500.0f

// 处理状态枚举
typedef enum {
    DSP_IDLE = 0,
    DSP_PROCESSING,
    DSP_SUCCESS,
    DSP_ERROR
} DSP_Status_t;

// 外部调用接口
void DSP_Init(void);
DSP_Status_t DSP_RunMatchFilter(uint16_t* raw_data, float32_t* distance);

#endif