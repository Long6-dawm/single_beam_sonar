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
    // 逻辑：真实采集完成 OR 手动触发仿真
    if (adc_complete_flag || sim_mode == 1) 
    {
        if (sim_mode == 1) 
        {
            HAL_ADC_Stop_DMA(&hadc1); // 必须先停掉正在跑的 DMA
            
            // 注入 MATLAB 数据 (确保 test_echo.h 已包含)
            memcpy(adc_raw, matlab_sim_echo, sizeof(uint16_t) * 4096); // 执行内存拷贝
            
            // 调试信息：确认数据注入成功 （可以在串口助手看到，如果是屏幕，则应该注释以下调试信息）
            // HAL_UART_Transmit(&huart1, (uint8_t*)"[SIM] Data Injected\r\n", 21, 100);
        }

        float32_t result_dist = 0;
        if (DSP_RunMatchFilter(adc_raw, &result_dist) == DSP_SUCCESS) 
        {
            // --- 1. 打印给电脑串口助手看的调试信息（保留这个好习惯） ---
            // char log_buf[64];
            // int m = (int)result_dist;
            // int mm = (int)((result_dist - m) * 1000);
            // snprintf(log_buf, sizeof(log_buf), "Result: %d.%03d m\r\n", m, mm);
            // HAL_UART_Transmit(&huart1, (uint8_t*)log_buf, strlen(log_buf), 100);

            // ==========================================================
            // --- 2. 发送给串口屏的控制指令 ---
            // ==========================================================
            
            char screen_buf[32];

            // 把真实测出的距离（比如 4999 毫米）发给屏幕的 x0 控件（推荐）
            int dist_mm = (int)(result_dist * 1000); // 将米转换为毫米，串口屏的 val 通常只能接整数
            snprintf(screen_buf, sizeof(screen_buf), "x0.val=%d", dist_mm);

            // 第一步：通过字符串模式发送 "x0.val=..."
            HAL_UART_Transmit(&huart1, (uint8_t*)screen_buf, strlen(screen_buf), 100);

            // 第二步：通过 HEX 模式发送三个 0xFF 结束符
            uint8_t end_cmd[3] = {0xFF, 0xFF, 0xFF};
            HAL_UART_Transmit(&huart1, end_cmd, 3, 100);
        }

        if (sim_mode == 1) 
        {
            sim_mode = 0; // 自动复位，防止无限循环仿真
            HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, 4096); // 恢复真实采集
        }
        adc_complete_flag = 0;

        // 非仿真模式下，采集完成后直接重启 ADC DMA，准备下一轮采集
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw, 4096);
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
        // 1. 立即原样发回给电脑，确认通信是否正常
        HAL_UART_Transmit(huart, rx_data, 2, 100);
        
        // 2. 只有回显正常了，下面的控制逻辑才有意义
        if(rx_data[0] == 0x0D) 
        {
            if(rx_data[1] == 0x00) {
                Start_Blink(&blinker0);
            }
            else if(rx_data[1] == 0x01) {
                Start_Blink(&blinker1);
                HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)LFM_SIGNAL, LFM_DATA_LEN, DAC_ALIGN_12B_R);
                HAL_TIM_Base_Start(&htim2);
            }
        }

        // 3. 重新启动中断接收，否则只会触发一次
        HAL_UART_Receive_IT(huart, rx_data, 2);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    HAL_TIM_Base_Stop(&htim2); // 停止采样
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
