/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <math.h>
#include <string.h>
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#pragma pack(1)
typedef union {
    //缓冲区指针
    uint8_t ptr[9];
    //数据结构体
    struct {
        //帧头
        uint16_t head;
        //功能码
        uint8_t fun;
        //数据
        union {
            uint8_t arr[4];
            float f32;
            uint32_t u32;
        } data;
        //帧尾
        uint16_t end;
    } B;
} Pkg;
#pragma pack()
Pkg buffer;



/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint32_t singleStepPulse = 40;
uint32_t circlePulse = 15000;
int32_t currentAngle_pulse = 0;
int32_t dir_flag = 0;
uint32_t tmpPulse = 0;
uint8_t isSingleMod = 0;
uint8_t isWorking = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
//        if (isSingleMod == 1) {
//            tmpPulse++;
//            if (singleStepPulse <= tmpPulse) {
//                HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
//                tmpPulse = 0;
//                isSingleMod = 0;
//            }
//        }
        currentAngle_pulse += dir_flag;
        if (currentAngle_pulse > circlePulse) {
            currentAngle_pulse = 0;
        } else if (currentAngle_pulse < 0) {
            currentAngle_pulse = (int32_t) circlePulse;
        }
    }
}


float getCurrentAngle_f32() {
    return (float) currentAngle_pulse * 360.0f / (float) circlePulse;
}

void motor_start_u() {
    dir_flag = 1;
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_UART_Transmit(&huart3, buffer.ptr, 9, 0xffff);
}

void motor_start_d() {
    dir_flag = -1;
    HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_UART_Transmit(&huart3, buffer.ptr, 9, 0xffff);
}

void motor_stop() {
    isWorking = 0;
    dir_flag = 0;
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    HAL_UART_Transmit(&huart3, buffer.ptr, 9, 0xffff);
}

void setSpeedPulse(uint32_t pulse_s) {
    uint32_t arr = (84000000 / (htim2.Init.Prescaler + 1)) / pulse_s;
    TIM2->ARR = arr - 1;
    TIM2->CCR3 = (arr / 2) - 1;

}

/**
 * @defgroup head
 * 帧头       0xEF 0xFE
 * 指令码     0x00:设置单步脉冲个数 uint_32
 *           0x01:一圈脉冲个数设置 uint_32
 *           0x02:最高转速设置    uint_32
 *           0x03:零点位置设置    none
 *           0x04:获取当前角度
 *           0x05:开始采集
 *           0x06:正向转动
 *           0x07:反向转动
 *           0x08:停止
 *           0x0A:通知需要请求数据
 *           0x0B:Busy
 * 数据       32位
 * 帧尾
 * @param huart
 * @param Size
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART3) {
        if (Size >= 9) {
            if (buffer.B.head == 0xFEEF && buffer.B.end == 0xEEFF) {
                switch (buffer.B.fun) {
                    //设置每隔多少脉冲发送一次处理数据指令
                    case 0: {
                        singleStepPulse = buffer.B.data.u32;
                        HAL_UART_Transmit(&huart3, buffer.ptr, 9, 0xffff);
                        break;
                    }
                        //设置转一圈需要多少脉冲
                    case 1: {
                        circlePulse = buffer.B.data.u32;
                        HAL_UART_Transmit(&huart3, buffer.ptr, 9, 0xffff);
                        break;
                    }
                        //最高转速设置
                    case 2: {
                        setSpeedPulse(buffer.B.data.u32);
                        HAL_UART_Transmit(&huart3, buffer.ptr, 9, 0xffff);
                        break;
                    }
                        //当前角度清0
                    case 3: {
                        currentAngle_pulse = 0;
                        buffer.B.data.u32 = currentAngle_pulse;
                        HAL_UART_Transmit(&huart3, buffer.ptr, 9, 0xffff);
                        break;
                    }
                        //获取当前角度
                    case 4: {
                        float value = getCurrentAngle_f32();
                        buffer.B.data.f32 = value;
                        HAL_UART_Transmit(&huart3, buffer.ptr, 9, 0xffff);
                        break;
                    }

                    case 5: {
                        isWorking = 1;
                        HAL_UART_Transmit(&huart3, buffer.ptr, 9, 0xffff);
                        break;
                    }
                    case 6: {
                        motor_start_u();
                        break;
                    }
                    case 7: {
                        motor_start_d();
                        break;
                    }
                    case 8: {
                        motor_stop();
                        break;
                    }
                    default:
                        break;
                }
            }
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buffer.ptr, 9);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void) {

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
    MX_TIM2_Init();
    MX_USART3_UART_Init();
    MX_USART6_UART_Init();
    /* USER CODE BEGIN 2 */
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buffer.ptr, 9);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
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
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

#if 0
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
#endif
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
