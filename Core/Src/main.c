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


/*PWM��ر���*/
uint8_t duty_cycle_input = 100;       	/* ���մ��������ռ�ձ�*/
static uint32_t pwm_value = 0;         // ����ռ�ձȵ�ֵ
uint32_t duty_cycle_max = 100;  // �������ռ�ձ�ֵ
uint8_t received_pwm_value = 0;      	  /*���յ�PWMռ�ձ�*/
uint8_t new_pwm_value_received = 0;     /*��־λ����ʾ�յ��µ�PWMֵ*/
static int breathing_direction = 1;    // ռ�ձȱ仯����1��-1
static uint32_t last_update_time = 0;  


/*���봦����ر���*/
#define MAX_INPUT_LENGTH 20  // ���뻺������󳤶�
char input_buffer[MAX_INPUT_LENGTH];  // ���뻺����
int buffer_index = 0;  // ��ǰ���뻺������λ�ã���ʾ��һ�������ַ�Ӧ����ŵ�λ��
uint8_t duty_cycle_input;  // ���浱ǰ���յ��ֽ�
int received_period_ms = 0;   // �յ�������ֵ
int new_period_value_received = 0;  // ��ʶ��������Ƿ��յ�����


/*ģʽ����ɫ��ر���*/
static uint8_t current_color = 0;      // ��ǰ����ɫ
static uint8_t current_mode = 0;       // ��ǰģʽ


/*������ر���*/
uint32_t last_button_press = 0;         /*����ϴΰ������µ�ʱ�䣬������������*/


/*ADC�����ɼ���ر���*/
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
	
	//����������ʱ��
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
	
	//usart�����жϽ���
	HAL_UART_Receive_IT(&huart1, &duty_cycle_input, 1); 
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//����PWMʵ�ֺ�����Ч��
		Update_PWM();
		HAL_Delay(10);
		
		//ÿʮ���ȡ��ǿ����
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
 * @brief ����PWMռ�ձȣ�����ʵ�ֺ�����Ч��
 *        ���ݵ�ǰģʽ�ͷ���̬����ռ�ձȣ�������ɫģʽ�仯���л��ƹ���ɫ
 * @param None
 * @retval None
 */
void Update_PWM(void)
{
		//��ʱ����
    if (HAL_GetTick() - last_update_time >= 10) 
    {
        last_update_time = HAL_GetTick();
					
        // ������յ��µ�ռ�ձȣ�����ռ�ձ�
				// ����ռ�ձȰ�������ģʽ
        if (new_pwm_value_received)  
        {
            pwm_value = received_pwm_value;  
        }
        else
        {
						// pwm���ݺ�������������/��С
            pwm_value += breathing_direction * 1; 
        

        // ����ռ�ձȷ�Χ
        if (pwm_value >= 100) {
            pwm_value = 100;
            breathing_direction = -1;  // ռ�ձȴ���󣬿�ʼ�����С
        } else if (pwm_value <= 0) {
            pwm_value = 0;
            breathing_direction = 1;  // ռ�ձȴ���С����ʼ��������
            
            // ������ɫģʽ
						/*
						����current_mode=0ʱ����ɫ�仯Ϊ�졢������
						����current_mode=1ʱ����ɫ�仯Ϊ�졢��
						*/
            if (current_mode == 0) {
                current_color = (current_color + 1) % 3;  
            } else if (current_mode == 1) {  
                current_color = (current_color == 0) ? 2 : 0; 
            }
					}
				}

        // ����ռ�ձ�
        if (current_color == 0) {  // ���
            __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, pwm_value);
            __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
        } else if (current_color == 1) {  // ����
            __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, pwm_value);
            __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 0);
        } else if (current_color == 2) {  // �̵�
            __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, pwm_value);
        }
    }
}




/**
 * @brief �ⲿ�жϻص������������л�������ģʽ����ɫģʽ����ɫģʽ��
 *        ͬʱ����������������ӡ��ǰģʽ
 * @param GPIO_Pin�������жϵ����ű��
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
    if (GPIO_Pin == GPIO_PIN_0) {  // ���PA0����
				new_pwm_value_received = 0;
        uint32_t current_time = HAL_GetTick(); //��ȡ��ǰϵͳʱ��
        if (current_time - last_button_press > 20) 
				{  // ��������200ms��
          current_mode = !current_mode;  // �л�ģʽ
					last_button_press = current_time;  // �����ϴΰ���ʱ��
        }
				//���ڴ�ӡ��ǰģʽ
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
 * @brief ���ڽ�����ɻص����������������ռ�ձȺ�����ֵ����������ز�����
 *        ֧�������ʽ<duty_cycle>,<period_ms>\r �� \n
 *        ��������ռ�ձ�ʱĬ������Ϊ1
* @param huart��ָ��UART�����ָ��
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
		// �жϴ�uart1����
    if (huart == &huart1) {
   
				// ������յ����л�س�������ʾ�������
				// ���뻺������ǰλ�����ý�����
        if (duty_cycle_input == '\r' || duty_cycle_input == '\n') {
            input_buffer[buffer_index] = '\0'; 

            // ����ռ�ձȺ����ڣ��ӻ������н���sscanf
						// ������ʽΪ<duty_cycle>,<period_ms>
            int pwm_value, period_value;
            if (sscanf((char*)input_buffer, "%d,%d", &pwm_value, &period_value) == 2) {
                // �淶duty cycle��Ч��Χ
                if (pwm_value >= 0 && pwm_value <= 100) {
                    received_pwm_value = pwm_value;
                    new_pwm_value_received = 1;  // ��ʶ����1����ʾ�ѽ��յ��̶�ռ�ձ�
                    printf("Received new duty cycle: %d%%\r\n", received_pwm_value);
                } else {
                    printf("Invalid duty cycle value. Please input a value between 0 and 100.\r\n");
                }

                // �淶period_value��Ч��Χ
                if (period_value > 0) {
                    received_period_ms = period_value;
                    printf("Received new period: %d ms\r\n", received_period_ms);
                    // ���ø������ڵĺ���
                    Update_Period(received_period_ms);
                } else {
                    printf("Invalid period value. Please input a value greater than 0.\r\n");
                }
            } else {
                printf("Invalid input format. Please input in the format: <duty_cycle>,<period_ms>\r\n");
            }

            // ���û�����������������뻺����memset
            buffer_index = 0;
            memset(input_buffer, 0, sizeof(input_buffer));
        }
        // �����δ�յ����������һ�����δ���������洢����
        else if (buffer_index < sizeof(input_buffer) - 1) {
            input_buffer[buffer_index++] = duty_cycle_input;
        }

				// �������ڽ����жϣ��ȴ���һ���ַ�
        HAL_UART_Receive_IT(&huart1, &duty_cycle_input, 1);  
    }
}





/**
 * @brief ����PWM���ڣ����ݽ��յ�������ֵ���¼��㲢����ARRֵ
 *        ��������ARR������Χ�����������
 * @param received_period_ms�����յ�������ֵ����λ�����룩
 * @retval None
 */
void Update_Period(uint32_t received_period_ms) {
    uint32_t psc_value = 99;  // �̶�PSC=99
    uint32_t clk_cnt = SystemCoreClock;  // APB2��ʱ��ʱ��Ƶ��(84 MHz)

    // ��ӡ��ʱ��ʱ��Ƶ��
    printf("SystemCoreClock = %lu\n", clk_cnt);

    // ����ARR
    uint32_t arr_value = (received_period_ms * clk_cnt) / ((psc_value + 1) * 1000) - 1;

    // ��ӡ�������
    printf("Received period (ms) = %lu\n", received_period_ms);
    printf("ARR value calculated = %lu\n", arr_value);

    // ����ARR���ֵ 65535
    if (arr_value > 65535) {
        arr_value = 65535;
        printf("ARR value limited to 65535\n");
    }

    // ����ARR�Ĵ���
    if (arr_value <= 65535) {
        __HAL_TIM_SET_AUTORELOAD(&htim10, arr_value);  
        __HAL_TIM_SET_AUTORELOAD(&htim13, arr_value);  
        __HAL_TIM_SET_AUTORELOAD(&htim14, arr_value);  
        printf("ARR value set successfully.\n");
    } else {
        printf("Error: ARR value out of range.\n");
    }

    // ����PWM���
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
}






/**
 * @brief ��ȡADC��ֵ������ת��Ϊ����ǿ�ȣ�Lux��
 *        ÿ��0.2s��ӡһ�ι���ǿ��
 * @param None
 * @retval None
 */
void Get_Lux(void) {
	//����ADC����
    HAL_ADC_Start(&hadc1);
	//�ȴ��������
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
        AD_Value = HAL_ADC_GetValue(&hadc1);
			//AD��ѹֵ��LUX��ת��
			float lux = 3872 * pow(AD_Value, -0.7);
			//ÿ��0.2���ӡһ�ι�ǿ
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
