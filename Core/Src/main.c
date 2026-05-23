/* USER CODE BEGIN Header */
#include <stdlib.h>
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

// Đồng bộ hóa thời gian giữa HAL và Code Thanh ghi
#define get_tick()  HAL_GetTick()
#define delay_ms(x) HAL_Delay(x)

float Kp = 0.035;
float Ki = 0.0;
float Kd = 0.3;
int finished = 0;
int finish_count = 0;
int basespeed = 40;
int lastError = 0;
int idle_counter = 0;
int position = 2000;
int last_turn = 0;

uint32_t straight_start_time = 0;
uint8_t is_started = 0;
uint8_t finish_sent = 0;

uint8_t is_turning_locked = 1;
uint32_t clear_line_timer = 0;
/* USER CODE END Includes */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN 0 */

/* ================= HÀM KHỞI TẠO BẰNG THANH GHI ================= */
void GPIO_Init_Register(void) {
    RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN);

    // Cảm biến Input: PA0(L2), PA1(L1), PA4(C), PB0(R1), PC1(R2)
    GPIOA->MODER &= ~((3 << 0) | (3 << 2) | (3 << 8));
    GPIOB->MODER &= ~(3 << 0);
    GPIOC->MODER &= ~(3 << 2);

    // Điều khiển Motor Output: PA8, PA9
    GPIOA->MODER &= ~((3 << 16) | (3 << 18));
    GPIOA->MODER |= (1 << 16) | (1 << 18);

    // Điều khiển Motor Output: PB5(STBY), PB10
    GPIOB->MODER &= ~((3 << 10) | (3 << 20));
    GPIOB->MODER |= (1 << 10) | (1 << 20);

    // Điều khiển Motor Output: PC7
    GPIOC->MODER &= ~(3 << 14);
    GPIOC->MODER |= (1 << 14);

    // Kéo chân STBY lên mức cao để bật Driver TB6612/L298
    GPIOB->BSRR = (1 << 5);
}

void Timer4_PWM_Init_Register(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    // Cấu hình PB6 (CH1), PB7 (CH2) sang Alternate Function (AF2)
    GPIOB->MODER &= ~((3 << 12) | (3 << 14));
    GPIOB->MODER |= (2 << 12) | (2 << 14);
    GPIOB->AFR[0] |= (2 << 24) | (2 << 28);

    TIM4->PSC = 72 - 1;   // 1MHz
    TIM4->ARR = 1000 - 1; // PWM 1kHz
    TIM4->CCMR1 |= (6 << 4) | (6 << 12);
    TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;

    TIM4->EGR |= TIM_EGR_UG; // Update Generation
    TIM4->CR1 |= TIM_CR1_CEN;
}

void UART6_Init_Register(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;

    // PC6 (TX) sang AF8
    GPIOC->MODER &= ~(3 << 12);
    GPIOC->MODER |= (2 << 12);
    GPIOC->AFR[0] |= (8 << 24);

    // PA12 (RX) sang AF8
    GPIOA->MODER &= ~(3 << 24);
    GPIOA->MODER |= (2 << 24);
    GPIOA->AFR[1] |= (8 << 16);

    // 9600 Baud @ 72MHz
    USART6->BRR = 0x1D4C;
    USART6->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void UART6_Print(char *msg) {
    while(*msg) {
        while(!(USART6->SR & USART_SR_TXE));
        USART6->DR = *msg++;
    }
}

/* ================= LOGIC ĐIỀU KHIỂN XE ================= */
void motor_control(int speed_left, int speed_right) {
    // LEFT MOTOR: PC7 (IN1), PA9 (IN2)
    if (speed_left >= 0) {
        GPIOC->BSRR = (1 << 7);
        GPIOA->BSRR = (1 << (9 + 16));
        TIM4->CCR1 = speed_left * 10;
    } else {
        GPIOC->BSRR = (1 << (7 + 16));
        GPIOA->BSRR = (1 << 9);
        TIM4->CCR1 = (-speed_left) * 10;
    }

    // RIGHT MOTOR: PA8 (IN1), PB10 (IN2)
    if (speed_right >= 0) {
        GPIOA->BSRR = (1 << 8);
        GPIOB->BSRR = (1 << (10 + 16));
        TIM4->CCR2 = speed_right * 10;
    } else {
        GPIOA->BSRR = (1 << (8 + 16));
        GPIOB->BSRR = (1 << 10);
        TIM4->CCR2 = (-speed_right) * 10;
    }
}

void PID_control(void) {
    if (finished) {
        motor_control(0, 0);
        if (!finish_sent) {
            uint32_t dist = get_tick() - straight_start_time;
            char msg[30];
            sprintf(msg, "F,%lu\nS\n", dist);
            UART6_Print(msg);
            finish_sent = 1;
        }
        return;
    }

    if (!is_started) {
        straight_start_time = get_tick();
        is_started = 1;
    }

    // ĐỌC CẢM BIẾN (Thanh ghi IDR)
    uint8_t L2 = !(GPIOA->IDR & (1 << 0));
    uint8_t L1 = !(GPIOA->IDR & (1 << 1));
    uint8_t C  = !(GPIOA->IDR & (1 << 4));
    uint8_t R1 = !(GPIOB->IDR & (1 << 0));
    uint8_t R2 = !(GPIOC->IDR & (1 << 1));

    int weight[] = {1000, 1500, 2000, 2500, 3000};
    int s[] = {L2, L1, C, R1, R2};
    int pos = 0, active = 0;

    for(int i=0; i<5; i++){
        if(s[i]){
            pos += weight[i];
            active++;
        }
    }

    if (L2 && R2 && (L1 || C || R1)) {
            finish_count++;
            // Đợi xe nằm trên vạch đích khoảng 4-5 chu kỳ để chắc chắn đó là vạch ngang thật
            if (finish_count > 4) {
                finished = 1;
            }
        } else {
            finish_count = 0;
        }


    if (active == 0) {
           idle_counter++;
           if (idle_counter < 5) {
               motor_control(40, 40);
           }
           else {
               if (last_turn == -1) motor_control(50, -50);
               else if (last_turn == 1) motor_control(-50, 50);
               else motor_control(20, -20);
           }
           return;
       } else {
           idle_counter = 0;
           position = pos/active;
       }


    // ===== KIỂM TRA MỞ KHÓA NGÃ RẼ =====
    if (is_turning_locked) {
        if (C && !L2 && !R2) {
            if (clear_line_timer == 0) {
                clear_line_timer = get_tick();
            } else if ((get_tick() - clear_line_timer) > 60) {
                is_turning_locked = 0;
                clear_line_timer = 0;
            }
        } else {
            clear_line_timer = 0;
        }
    }

    // ===== CUA 90 ĐỘ THÔNG MINH (TRÁI) =====
        if (L2 && L1 && C && !R2 && !is_turning_locked) {
            motor_control(-30, -30);
            delay_ms(8);
            motor_control(0, 0);

            if (!(GPIOC->IDR & (1 << 1))) { // R2_now
                finished = 1;
                return;
            }

            uint32_t dist = get_tick() - straight_start_time;
            char msg[30];
            sprintf(msg, "F,%lu\nL\n", dist);
            UART6_Print(msg);

            is_turning_locked = 1;
            clear_line_timer = 0;
            last_turn = -1;
            motor_control(30, -60);

            while (1) {
                if (!(GPIOA->IDR & (1 << 1)) || !(GPIOA->IDR & (1 << 4))) { // L1_now || C_now
                    motor_control(-60, 60);
                    delay_ms(25);
                    break;
                }
                if (!(GPIOA->IDR & (1 << 0)) && !(GPIOC->IDR & (1 << 1))) { // L2_now && R2_now
                    motor_control(0, 0);
                    finished = 1;
                    return;
                }
            }
            lastError = 0;
            motor_control(40, 40);
            delay_ms(20);
            straight_start_time = get_tick();
            return;
        }

        	//Phải
        if (R2 && R1 && C && !L2 && !is_turning_locked) {
                motor_control(-30, -30);
                delay_ms(8);
                motor_control(0, 0);

                if (!(GPIOA->IDR & (1 << 0))) { // L2_now
                    finished = 1;
                    return;
                }

                uint32_t dist = get_tick() - straight_start_time;
                char msg[30];
                sprintf(msg, "F,%lu\nR\n", dist);
                UART6_Print(msg);

                is_turning_locked = 1;
                clear_line_timer = 0;
                last_turn = 1;
                motor_control(-60, 30);

                while (1) {
                    if (!(GPIOB->IDR & (1 << 0)) || !(GPIOA->IDR & (1 << 4))) { // R1_now || C_now
                        motor_control(60, -60);
                        delay_ms(25);
                        break;
                    }
                }
                lastError = 0;
                motor_control(40, 40);
                delay_ms(20);
                straight_start_time = get_tick();
                return;
            }


    // ===== PID ĐI THẲNG =====
    int error = position - 2000;
    int P = error;
    int D = error - lastError;
    lastError = error;

    int speed = P*Kp + D*Kd;
    int base = basespeed;

    if(abs(error) > 500) base = 40;

    int left  = base - speed;
    int right = base + speed;

    if(left > 100) left = 100;
    if(right > 100) right = 100;
    if(left < -100) left = -100;
    if(right < -100) right = -100;

    motor_control(left, right);
}
/* USER CODE END 0 */

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN 2 */
  // Khởi tạo các ngoại vi bằng thanh ghi (Bỏ qua các hàm MX_... mặc định)
  GPIO_Init_Register();
  Timer4_PWM_Init_Register();
  UART6_Init_Register();

  // Khóa bánh trước khi chạy
  TIM4->CCR1 = 0;
  TIM4->CCR2 = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      PID_control();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration (Do CubeMX tự sinh ra)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
