#ifndef __DSP_PROCESS_H
#define __DSP_PROCESS_H

#include "arm_math.h"

#define FFT_LEN 4096
#define FS 500000.0f
#define V_SOUND 1500.0f
// 正常（单站/收发合一）模式默认置零长度：发射脉宽 1ms = 500 点，多留 0.2ms 余量 = 600 点
#define BLANK_IDX 600

// 处理状态枚举
typedef enum {
    DSP_IDLE = 0,
    DSP_PROCESSING,
    DSP_SUCCESS,
    DSP_ERROR
} DSP_Status_t;

// 外部调用接口
void DSP_Init(void);
DSP_Status_t DSP_RunMatchFilter(uint16_t* raw_data, float32_t* distance, uint32_t* peak_index, uint16_t blank_samples);

#endif