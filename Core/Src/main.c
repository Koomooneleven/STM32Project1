/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/*PWM相关变量*/
uint8_t duty_cycle_input = 100;       	/* 接收串口输入的占空比*/
static uint32_t pwm_value = 0;         // 储存占空比的值
uint32_t duty_cycle_max = 100;  // 储存最大占空比值
uint8_t received_pwm_value = 0;      	  /*接收的PWM占空比*/
uint8_t new_pwm_value_received = 0;     /*标志位，表示收到新的PWM值*/
static int breathing_direction = 1;    // 占空比变化方向：1或-1
static uint32_t last_update_time = 0;  


/*输入处理相关变量*/
#define MAX_INPUT_LENGTH 20  // 输入缓冲区最大长度
char input_buffer[MAX_INPUT_LENGTH];  // 输入缓冲区
int buffer_index = 0;  // 当前输入缓冲区的位置，表示下一个接收字符应当存放的位置
uint8_t duty_cycle_input;  // 储存当前接收的字节
int received_period_ms = 0;   // 收到的周期值
int new_period_value_received = 0;  // 标识符，标记是否收到周期


/*模式与颜色相关变量*/
static uint8_t current_color = 0;      // 当前的颜色
static uint8_t current_mode = 0;       // 当前模式


/*按键相关变量*/
uint32_t last_button_press = 0;         /*存放上次按键按下的时间，用于消除抖动*/


/*ADC光敏采集相关变量*/
uint32_t AD_Value; 
uint32_t voltage;






//char *display_duty = "current duty:"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Update_PWM(void);
void Update_Period(uint32_t period_ms);
void Get_Lux(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM13_Init();
  MX_TIM10_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	//开启三个定时器
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
	
	//usart启动中断接收
	HAL_UART_Receive_IT(&huart1, &duty_cycle_input, 1); 
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//更新PWM实现呼吸灯效果
		Update_PWM();
		HAL_Delay(10);
		
		//每十秒获取光强数据
		static uint32_t last_voltage_time = 0;
		if (HAL_GetTick() - last_voltage_time >= 10) {
				Get_Lux();
				last_voltage_time = HAL_GetTick();
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/**
 * @brief 更新PWM占空比，用于实现呼吸灯效果
 *        根据当前模式和方向动态调整占空比，并在颜色模式变化后切换灯光颜色
 * @param None
 * @retval None
 */
void Update_PWM(void)
{
		//延时消除
    if (HAL_GetTick() - last_update_time >= 10) 
    {
        last_update_time = HAL_GetTick();
					
        // 如果接收到新的占空比，更新占空比
				// 否则占空比按呼吸灯模式
        if (new_pwm_value_received)  
        {
            pwm_value = received_pwm_value;  
        }
        else
        {
						// pwm根据呼吸方向逐渐增大/减小
            pwm_value += breathing_direction * 1; 
        

        // 控制占空比范围
        if (pwm_value >= 100) {
            pwm_value = 100;
            breathing_direction = -1;  // 占空比达最大，开始反向减小
        } else if (pwm_value <= 0) {
            pwm_value = 0;
            breathing_direction = 1;  // 占空比达最小，开始正向增大
            
            // 更新颜色模式
						/*
						参数current_mode=0时，颜色变化为红、蓝、绿
						参数current_mode=1时，颜色变化为红、绿
						*/
            if (current_mode == 0) {
                current_color = (current_color + 1) % 3;  
            } else if (current_mode == 1) {  
                current_color = (current_color == 0) ? 2 : 0; 
            }
					}
				}

        // 更新占空比
        if (current_color == 0) {  // 红灯
            __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, pwm_value);
            __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
        } else if (current_color == 1) {  // 蓝灯
            __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm_value);
            __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
        } else if (current_color == 2) {  // 绿灯
            __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pwm_value);
        }
    }
}




/**
 * @brief 外部中断回调函数，用于切换呼吸灯模式（两色模式和三色模式）
 *        同时按键消除抖动并打印当前模式
 * @param GPIO_Pin：触发中断的引脚编号
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
    if (GPIO_Pin == GPIO_PIN_0) {  // 如果PA0触发
				new_pwm_value_received = 0;
        uint32_t current_time = HAL_GetTick(); //获取当前系统时间
        if (current_time - last_button_press > 20) 
				{  // 防抖动（200ms）
          current_mode = !current_mode;  // 切换模式
					last_button_press = current_time;  // 更新上次按键时间
        }
				//串口打印当前模式
				if(current_mode == 0)
				{
						printf("RED-BLUE-GREEN");
				}	
				else
				{
						printf("RED-BLUE");
				}
    }
}





/**
 * @brief 串口接收完成回调函数，解析输入的占空比和周期值，并更新相关参数。
 *        支持输入格式<duty_cycle>,<period_ms>\r 或 \n
 *        单独输入占空比时默认周期为1
* @param huart：指向UART句柄的指针
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
		// 判断从uart1接收
    if (huart == &huart1) {
   
				// 如果接收到换行或回车符，表示输入完成
				// 输入缓冲区当前位置设置结束符
        if (duty_cycle_input == '\r' || duty_cycle_input == '\n') {
            input_buffer[buffer_index] = '\0'; 

            // 定义占空比和周期，从缓冲区中解析sscanf
						// 期望格式为<duty_cycle>,<period_ms>
            int pwm_value, period_value;
            if (sscanf((char*)input_buffer, "%d,%d", &pwm_value, &period_value) == 2) {
                // 规范duty cycle有效范围
                if (pwm_value >= 0 && pwm_value <= 100) {
                    received_pwm_value = pwm_value;
                    new_pwm_value_received = 1;  // 标识符置1，表示已接收到固定占空比
                    printf("Received new duty cycle: %d%%\r\n", received_pwm_value);
                } else {
                    printf("Invalid duty cycle value. Please input a value between 0 and 100.\r\n");
                }

                // 规范period_value有效范围
                if (period_value > 0) {
                    received_period_ms = period_value;
                    printf("Received new period: %d ms\r\n", received_period_ms);
                    // 调用更新周期的函数
                    Update_Period(received_period_ms);
                } else {
                    printf("Invalid period value. Please input a value greater than 0.\r\n");
                }
            } else {
                printf("Invalid input format. Please input in the format: <duty_cycle>,<period_ms>\r\n");
            }

            // 重置缓冲区索引并清空输入缓冲区memset
            buffer_index = 0;
            memset(input_buffer, 0, sizeof(input_buffer));
        }
        // 如果还未收到结束符，且缓冲区未满，继续存储数据
        else if (buffer_index < sizeof(input_buffer) - 1) {
            input_buffer[buffer_index++] = duty_cycle_input;
        }

				// 启动串口接收中断，等待下一个字符
        HAL_UART_Receive_IT(&huart1, &duty_cycle_input, 1);  
    }
}





/**
 * @brief 更新PWM周期，根据接收到的周期值重新计算并设置ARR值
 *        如果计算的ARR超出范围，会进行限制
 * @param received_period_ms：接收到的周期值（单位：毫秒）
 * @retval None
 */
void Update_Period(uint32_t received_period_ms) {
    uint32_t psc_value = 99;  // 固定PSC=99
    uint32_t clk_cnt = SystemCoreClock;  // APB2定时器时钟频率(84 MHz)

    // 打印定时器时钟频率
    printf("SystemCoreClock = %lu\n", clk_cnt);

    // 计算ARR
    uint32_t arr_value = (received_period_ms * clk_cnt) / ((psc_value + 1) * 1000) - 1;

    // 打印计算过程
    printf("Received period (ms) = %lu\n", received_period_ms);
    printf("ARR value calculated = %lu\n", arr_value);

    // 限制ARR最大值 65535
    if (arr_value > 65535) {
        arr_value = 65535;
        printf("ARR value limited to 65535\n");
    }

    // 设置ARR寄存器
    if (arr_value <= 65535) {
        __HAL_TIM_SET_AUTORELOAD(&htim10, arr_value);  
        __HAL_TIM_SET_AUTORELOAD(&htim13, arr_value);  
        __HAL_TIM_SET_AUTORELOAD(&htim14, arr_value);  
        printf("ARR value set successfully.\n");
    } else {
        printf("Error: ARR value out of range.\n");
    }

    // 启动PWM输出
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
}






/**
 * @brief 读取ADC的值并将其转化为光照强度（Lux）
 *        每隔0.2s打印一次光照强度
 * @param None
 * @retval None
 */
void Get_Lux(void) {
	//开启ADC接收
    HAL_ADC_Start(&hadc1);
	//等待接收完成
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        AD_Value = HAL_ADC_GetValue(&hadc1);
			//AD电压值到LUX的转换
			float lux = 3872 * pow(AD_Value, -0.7);
			//每隔0.2秒打印一次光强
        static uint32_t last_print_time = 0;
        if (HAL_GetTick() - last_print_time >= 200) {
            printf("Lux = %.1f \r\n", lux);

            last_print_time = HAL_GetTick();
        }
    }
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
