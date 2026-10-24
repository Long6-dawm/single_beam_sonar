/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lfm_data.h"
#include "arm_math.h"
#include <stdio.h>      // 修复 sprintf 警告
#include <string.h>     // 修复 strlen 警告
#include "dsp_process.h" // 告诉 main 函数 DSP_Init 和 DSP_RunMatchFilter 是什么
#include "test_echo.h"  // 包含测试回波数据的头文件
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 接收调试（分置换能器）置零长度：接收端不发射、无电串扰，默认不置零。
// 若功放 EMI 在起始处造成尖峰，可改为 20~50（须远小于直达波 index，如 0.3m≈100）。
#define RX_DEBUG_BLANK 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t rx_data[2]; // 用于存放接收到的 0x0D 和 0x00/01
uint16_t adc_raw[4096];        // 关键：定义 ADC 原始数据缓冲区
volatile uint8_t adc_complete_flag = 0; // 关键：定义采集完成标志位

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    uint8_t count;      // 剩余翻转次数 (闪烁5次 = 10次翻转)
    uint8_t is_active;  // 是否正在闪烁
} LED_Blinker_t;

LED_Blinker_t blinker0 = {GPIOF, GPIO_PIN_9, 0, 0};
LED_Blinker_t blinker1 = {GPIOF, GPIO_PIN_10, 0, 0};

volatile uint8_t sim_mode = 0; // 增加 volatile 确保调试器修改后 CPU 能立即察觉

// ===== 调试模式相关状态 =====
volatile uint8_t tx_debug = 0;          // 发射调试激活
volatile uint8_t rx_debug = 0;          // 接收调试激活
volatile uint8_t sync_flag = 0;         // AWG Sync 触发标志（PC1 EXTI 置位）
volatile uint16_t tx_period_ms = 100;   // 发射重复周期(ms)，默认100
volatile uint32_t last_tx_tick = 0;     // 上次发射时刻
volatile uint8_t dump_request = 0;      // 请求导出一帧 raw
volatile uint8_t rx_expect_period = 0;  // 1=等待接收周期参数(2字节)
volatile uint8_t adc_acquiring = 0;     // 1=正在采集（防止页面3画到半帧）
volatile uint8_t wave_pending = 0;      // 请求在页面3显示最近一帧波形
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Start_Blink(LED_Blinker_t* b) {
    b->count = 10;     // 设置翻转10次（5次亮灭）
    b->is_active = 1;
}

// 将 adc_raw[4096] 以二进制帧导出：AA 55 | len(2字节小端,=4096) | 4096*uint16(小端)
void Dump_ADC_Raw(void) {
    uint8_t hdr[4] = {0xAA, 0x55, 0x00, 0x10}; // 0x1000 = 4096 样本数
    HAL_UART_Transmit(&huart1, hdr, 4, 100);

    uint8_t* p = (uint8_t*)adc_raw;
    uint32_t len = sizeof(adc_raw); // 8192 字节
    uint32_t sent = 0;
    while (sent < len) {
        uint32_t chunk = (len - sent) > 256 ? 256 : (len - sent);
        HAL_UART_Transmit(&huart1, p + sent, chunk, 1000);
        sent += chunk;
    }
}

/* ---------- 淘晶驰串口屏命令辅助（页面3波形/文本） ---------- */
// 发送一条淘晶驰命令，自动追加 FF FF FF 结束符
static void TJC_Send(const char* cmd) {
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd, strlen(cmd), 100);
    uint8_t end[3] = {0xFF, 0xFF, 0xFF};
    HAL_UART_Transmit(&huart1, end, 3, 100);
}

static void Wave_Clear(const char* obj, uint8_t ch) {
    char buf[20];
    snprintf(buf, sizeof(buf), "cle %s,%d", obj, (int)ch);
    TJC_Send(buf);
}

static void Wave_Add(const char* obj, uint8_t ch, uint8_t val) {
    char buf[24];
    snprintf(buf, sizeof(buf), "add %s,%d,%d", obj, (int)ch, (int)val);
    TJC_Send(buf);
}

static void Text_Set(const char* obj, const char* txt) {
    char buf[40];
    snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", obj, txt);
    TJC_Send(buf);
}

// 在页面3显示"最近一帧"：s0=原始包络, s1=匹配输出, v_vpp/v_peak/v_dist=文本
// 调用前应确保 adc_raw 与 DSP 匹配输出属于同一完整帧（见 adc_acquiring 保护）
void Display_Debug_Curves(void) {
    const uint16_t N = 512;              // 波形点数
    const uint16_t STEP = FFT_LEN / N;   // 4096/512 = 8
    const float32_t* match = DSP_GetMatchOutput();

    // 1) 匹配输出峰值（索引 + 幅度）—— 与 s1 曲线严格对应
    float32_t peak_amp = 0.0f;
    uint32_t peak_idx = 0;
    for (uint32_t i = 0; i < FFT_LEN; i++) {
        if (match[i] > peak_amp) { peak_amp = match[i]; peak_idx = i; }
    }

    // 2) 原始时域 Vpp（伏特）与距离
    uint16_t raw_min = 4095, raw_max = 0;
    for (uint32_t i = 0; i < FFT_LEN; i++) {
        if (adc_raw[i] < raw_min) raw_min = adc_raw[i];
        if (adc_raw[i] > raw_max) raw_max = adc_raw[i];
    }
    float32_t vpp  = (float32_t)(raw_max - raw_min) * 3.3f / 4096.0f;
    float32_t dist = ((float32_t)peak_idx / FS) * V_SOUND / 2.0f;

    // 3) 清波形
    Wave_Clear("s0", 0);
    Wave_Clear("s1", 0);

    // 4) 每 8 点合并为 1 点，交替发送两条曲线
    for (uint16_t n = 0; n < N; n++) {
        uint16_t base = n * STEP;

        // s0：原始包络 = 组内 max(|adc-2048|)，缩放到 0~255
        uint16_t env = 0;
        for (uint16_t k = 0; k < STEP; k++) {
            int32_t v = (int32_t)adc_raw[base + k] - 2048;
            if (v < 0) v = -v;
            if ((uint16_t)v > env) env = (uint16_t)v;
        }
        uint8_t val0 = (uint8_t)((uint32_t)env * 255U / 2048U);

        // s1：匹配输出 = 组内最大，按本帧峰值归一化到 0~255
        float32_t m = 0.0f;
        for (uint16_t k = 0; k < STEP; k++) {
            float32_t v = match[base + k];
            if (v > m) m = v;
        }
        uint8_t val1 = (peak_amp > 0.0f) ? (uint8_t)(m / peak_amp * 255.0f) : 0U;

        Wave_Add("s0", 0, val0);
        Wave_Add("s1", 0, val1);
    }

    // 5) 文本
    char tbuf[24];
    snprintf(tbuf, sizeof(tbuf), "%.3fV", (double)vpp);
    Text_Set("v_vpp", tbuf);
    snprintf(tbuf, sizeof(tbuf), "%lu", (unsigned long)peak_idx);
    Text_Set("v_peak", tbuf);
    snprintf(tbuf, sizeof(tbuf), "%.3fm", (double)dist);
    Text_Set("v_dist", tbuf);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC_Init();
  MX_USART1_UART_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  DSP_Init();

  // 1. 启动用于控制 LED 闪烁的定时器 TIM7 (保持开启，等待 count 变化)
  HAL_TIM_Base_Start_IT(&htim7); 

  // 2. 开启一次串口接收中断，准备接收你发的 0D 00 或 0D 01
  HAL_UART_Receive_IT(&huart1, rx_data, 2);

  // 3. 发送开机问候语，顺便验证波特率是否正常
  uint8_t start_msg[] = "System Ready - Waiting for 0D 00 or 0D 01\r\n";
  HAL_UART_Transmit(&huart1, start_msg, sizeof(start_msg), 100);
  
  //强制静态输出测试
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048); // 强制输出 1.65V

  // 让 ADC 进入 DMA 待命状态，等待 TIM2 触发 =====
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, 4096);

  // float32_t test_val = 3.14159265f;
  // float32_t result;

  // // 调用 DSP 库函数计算 sin(π)
  // result = arm_sin_f32(test_val);

  // // 通过串口打印结果
  // char msg[64];
  // // 注意：如果你的 printf 不支持浮点，可以手动转成整数部分和小数部分打印
  // sprintf(msg, "DSP Test: sin(PI) = %f\r\n", (double)result);
  // HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // ===== 发射调试：按周期反复发送 LFM =====
    if (tx_debug)
    {
        if ((HAL_GetTick() - last_tx_tick) >= tx_period_ms)
        {
            HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)LFM_SIGNAL, LFM_DATA_LEN, DAC_ALIGN_12B_R);
            last_tx_tick = HAL_GetTick();
        }
    }

    // ===== 接收调试：AWG Sync 触发一次采集 =====
    if (rx_debug && sync_flag)
    {
        sync_flag = 0;
        HAL_ADC_Stop_DMA(&hadc1);
        HAL_TIM_Base_Start(&htim2);                     // 启动 500kHz 触发
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, 4096);
        adc_acquiring = 1;                              // 标记采集中
    }

    // ===== 采集完成处理 =====
    if (adc_complete_flag || (sim_mode == 1 && !rx_debug && !tx_debug))
    {
        adc_complete_flag = 0;

        if (rx_debug)
        {
            // 接收调试：跑 DSP 填充匹配输出，等下一个 sync（不向屏幕发连续文本）
            float32_t result_dist = 0;
            uint32_t pk = 0;
            DSP_RunMatchFilter(adc_raw, &result_dist, &pk, RX_DEBUG_BLANK);
        }
        else
        {
            // 正常 / 仿真模式
            if (sim_mode == 1)
            {
                sim_mode = 0;
                HAL_ADC_Stop_DMA(&hadc1);
                memcpy(adc_raw, matlab_sim_echo, sizeof(uint16_t) * 4096);
            }

            float32_t result_dist = 0;
            if (DSP_RunMatchFilter(adc_raw, &result_dist, NULL, BLANK_IDX) == DSP_SUCCESS)
            {
                char screen_buf[32];
                int dist_mm = (int)(result_dist * 1000);
                snprintf(screen_buf, sizeof(screen_buf), "x0.val=%d", dist_mm);
                HAL_UART_Transmit(&huart1, (uint8_t*)screen_buf, strlen(screen_buf), 100);
                uint8_t end_cmd[3] = {0xFF, 0xFF, 0xFF};
                HAL_UART_Transmit(&huart1, end_cmd, 3, 100);
            }

            // 重新进入 ADC 待命，准备下一轮
            HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, 4096);
        }
    }

    // ===== 导出原始 ADC 数据 =====
    if (dump_request)
    {
        dump_request = 0;
        Dump_ADC_Raw();
    }

    // ===== 页面3显示最近一帧波形（按需，保证同一完整帧） =====
    if (wave_pending && !adc_acquiring)
    {
        wave_pending = 0;
        Display_Debug_Curves();
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM7) {
        // 处理 LED0
        if (blinker0.is_active && blinker0.count > 0) {
            HAL_GPIO_TogglePin(blinker0.port, blinker0.pin);
            blinker0.count--;
            if (blinker0.count == 0) {
                blinker0.is_active = 0;
                HAL_GPIO_WritePin(blinker0.port, blinker0.pin, GPIO_PIN_SET); // 确保最后是灭的
            }
        }
        // 处理 LED1
        if (blinker1.is_active && blinker1.count > 0) {
            HAL_GPIO_TogglePin(blinker1.port, blinker1.pin);
            blinker1.count--;
            if (blinker1.count == 0) {
                blinker1.is_active = 0;
                HAL_GPIO_WritePin(blinker1.port, blinker1.pin, GPIO_PIN_SET); // 确保最后是灭的
            }
        }
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1) // 确保是对应的串口
    {
        if(rx_expect_period)
        {
            // 收到周期参数：2 字节小端(ms)
            tx_period_ms = rx_data[0] | ((uint16_t)rx_data[1] << 8);
            if(tx_period_ms < 1) tx_period_ms = 1;
            rx_expect_period = 0;
        }
        else if(rx_data[0] == 0x0D)
        {
            switch(rx_data[1])
            {
                case 0x00: // LED0 闪烁
                    Start_Blink(&blinker0);
                    break;

                case 0x01: // 单次发射（正常测距）
                    Start_Blink(&blinker1);
                    HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)LFM_SIGNAL, LFM_DATA_LEN, DAC_ALIGN_12B_R);
                    HAL_TIM_Base_Start(&htim2);
                    adc_acquiring = 1; // 标记采集中
                    break;

                case 0x02: // 进入调试模式（占位）
                    break;

                case 0x03: // 发射调试开始
                    rx_debug = 0;
                    HAL_ADC_Stop_DMA(&hadc1);       // 停 ADC，避免干扰
                    HAL_TIM_Base_Start(&htim2);     // TIM2 常开触发 DAC
                    tx_debug = 1;
                    last_tx_tick = HAL_GetTick();
                    break;

                case 0x04: // 发射调试停止
                    tx_debug = 0;
                    HAL_TIM_Base_Stop(&htim2);
                    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
                    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
                    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
                    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, 4096);
                    break;

                case 0x05: // 接收调试开始
                    tx_debug = 0;
                    HAL_TIM_Base_Stop(&htim2);
                    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
                    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
                    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
                    HAL_ADC_Stop_DMA(&hadc1);
                    rx_debug = 1;
                    sync_flag = 0;
                    break;

                case 0x06: // 接收调试停止
                    rx_debug = 0;
                    HAL_TIM_Base_Stop(&htim2);
                    HAL_ADC_Stop_DMA(&hadc1);
                    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, 4096);
                    break;

                case 0x07: // 退出调试，回正常模式
                    tx_debug = 0;
                    rx_debug = 0;
                    HAL_TIM_Base_Stop(&htim2);
                    HAL_DAC_Stop_DMA(&hdac, DAC_CHANNEL_1);
                    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
                    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 2048);
                    HAL_ADC_Stop_DMA(&hadc1);
                    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, 4096);
                    break;

                case 0x08: // 设置发射周期，等待 2 字节参数
                    rx_expect_period = 1;
                    break;

                case 0x09: // 导出一帧 raw
                    dump_request = 1;
                    break;

                case 0x0B: // 页面3显示最近一帧波形（原始包络 + 匹配输出 + 文本）
                    wave_pending = 1;
                    break;

                default:
                    break;
            }
        }

        HAL_UART_Receive_IT(huart, rx_data, 2);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == SYNC_IN_Pin)
    {
        sync_flag = 1; // AWG Sync 上升沿，请求一次采集
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        // 溢出(ORE)等错误会中止接收，这里清标志并重新使能接收，避免"收一次后死机"
        __HAL_UART_CLEAR_OREFLAG(huart);
        HAL_UART_Receive_IT(huart, rx_data, 2);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    adc_acquiring = 0; // 采集完成
    if(tx_debug) {
        return; // 发射调试不关心回波（ADC 已停用）
    }
    if(rx_debug) {
        HAL_TIM_Base_Stop(&htim2); // 收完这一帧就停，等下一个 sync
        adc_complete_flag = 1;
        return;
    }
    HAL_TIM_Base_Stop(&htim2); // 正常模式：停止采样
    adc_complete_flag = 1;     // 置标志位
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
