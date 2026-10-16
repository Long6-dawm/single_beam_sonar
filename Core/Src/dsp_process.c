#include "dsp_process.h"
#include "ref_fft_data.h" // 包含 MATLAB 生成的 REF_FFT_CONJ 数组
#include "stdio.h"        // 为了使用printf调试输出
#include <string.h>       // 为了使用 strlen
#include "main.h"         // 让编译器认识 HAL 库的串口函数

// 内存优化：我们将某些大数组定义在特定区域，或在函数内复用
// 注意：adc_f32 和 fft_out 可以复用同一块内存以节省空间
static float32_t dsp_buffer_a[FFT_LEN]; 
static float32_t dsp_buffer_b[FFT_LEN] __attribute__((section(".ccmram")));

static arm_rfft_fast_instance_f32 fft_inst;

/**
 * @brief 初始化 DSP 实例
 */
// void DSP_Init(void) {
//     arm_rfft_fast_init_f32(&fft_inst, FFT_LEN);
// }
// 注意：如果你想在初始化时检查状态，可以这样写：
extern UART_HandleTypeDef huart1; 

void DSP_Init(void) {
    // 1. 初始化并获取状态
    arm_status status = arm_rfft_fast_init_f32(&fft_inst, FFT_LEN);
    
    // 2. 准备发送缓冲区
    char msg_buf[64];
    
    // 3. 判断并组装字符串
    if (status != ARM_MATH_SUCCESS) {
        sprintf(msg_buf, "DSP INIT FAILED! ERROR: %d\r\n", status);
    } else {
        sprintf(msg_buf, "DSP INIT SUCCESS!\r\n");
    }
    
    // 4. 直接通过 HAL 库底层发送
    HAL_UART_Transmit(&huart1, (uint8_t*)msg_buf, strlen(msg_buf), 100);
}
// 调试信息结束

/**
 * @brief 执行匹配滤波（脉冲压缩）
 * @param raw_data 输入的 ADC 原始数据指针
 * @param distance 输出计算得到的距离结果
 * @return 处理状态
 */
DSP_Status_t DSP_RunMatchFilter(uint16_t* raw_data, float32_t* distance) {
    float32_t max_val;
    uint32_t peak_idx;

    // 1. 预处理：uint16 转 float 并去直流 (2048 为 12位 ADC 中值)
    //    发射串扰置零：前 BLANK_IDX 点（覆盖发射窗口）直接置 0，
    //    避免发射波形串到接收端形成的相关峰（index≈0）干扰寻峰
    for (uint16_t i = 0; i < FFT_LEN; i++) {
        if (i < BLANK_IDX) {
            dsp_buffer_a[i] = 0.0f;
        } else {
            dsp_buffer_a[i] = (float32_t)raw_data[i] - 2048.0f;
        }
    }

    // 2. 正向 FFT (实数)
    // dsp_buffer_a (时域回波) -> dsp_buffer_b (频域回波)
    arm_rfft_fast_f32(&fft_inst, dsp_buffer_a, dsp_buffer_b, 0);

    // 3. 频域复数点乘 (匹配滤波核心)
    // 计算：Echo_FFT * Ref_FFT_Conjugate
    // arm_cmplx_mult_cmplx_f32 要求长度为复数对的数量，即 FFT_LEN/2
    arm_cmplx_mult_cmplx_f32(dsp_buffer_b, (float32_t*)REF_FFT_CONJ, dsp_buffer_b, FFT_LEN/2);

    // 4. 逆 FFT (IFFT) 变回时域
    // dsp_buffer_b (处理后频域) -> dsp_buffer_a (脉冲压缩时域)
    arm_rfft_fast_f32(&fft_inst, dsp_buffer_b, dsp_buffer_a, 1);

    // 5. 取模/绝对值（提取包络）
    arm_abs_f32(dsp_buffer_a, dsp_buffer_a, FFT_LEN);

    // 6. 寻峰：在整个 16384 长度内找能量最大的点
    arm_max_f32(dsp_buffer_a, FFT_LEN, &max_val, &peak_idx);

    // 7. 距离计算
    // 距离 = (索引 / 采样率) * 声速 / 2
    *distance = ((float32_t)peak_idx / FS) * V_SOUND / 2.0f;

    return DSP_SUCCESS;
}