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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rc522.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define x_thuan HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define x_nghich HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)

#define x_ENABLE HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)
#define x_DISABLE HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)

#define x_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define x_LOW HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)

#define y_thuan HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define y_nghich HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)

#define y_ENABLE HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)
#define y_DISABLE HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)

#define y_HIGH HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET)
#define y_LOW HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t status;
uint8_t str[16]; // Max_LEN = 16
uint8_t sNum[5];
uint8_t kq;
char Rx_data[100];
char buffer;
int nhap_hang;
char sp;
int kt, x_axis, y_axis, rfid = -1;
int done = -1, done_step = -1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
struct FILE
{
  int dummy;
};
struct FILE __stdout;
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  // HAL_UART_Receive_IT(&huart1, (uint8_t*) Rx_data, 1);
  // HAL_UART_Transmit(&huart1, (uint8_t *) "AOFFBONC", 8, 1000);
  if (huart->Instance == USART1)
  {
    // HAL_UART_Transmit(&huart1,(uint8_t *)&buffer, sizeof(buffer),100);
    if (buffer == '1')
      nhap_hang = 1; // bang tai quay thuan
    else if (buffer == '2')
      nhap_hang = 2; // dao chieu bang tai
    else if (buffer == '3')
    {
      kt = 1;
      done_step = 0;
      done = 0;
    } // san pham 1
    else if (buffer == '4')
    {
      kt = 2;
      done_step = 0;
      done = 0;
    } // san pham 2
    else if (buffer == '5')
    {
      kt = 3;
      done_step = 0;
      done = 0;
    } // san pham 3
    else if (buffer == '6')
    {
      kt = 4;
      done_step = 0;
      done = 0;
    } // san pham 4
    else if (buffer == '7')
    {
      kt = 5;
      done_step = 0;
      done = 0;
    } // san pham 5
    else if (buffer == '8')
    {
      kt = 6;
      done_step = 0;
      done = 0;
    } // san pham 6
    else if (buffer == '0')
      nhap_hang = 3; // dung bang tai
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&buffer, 1);
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t position = 0;
uint16_t speed;
uint8_t dir = 0;
int Err, Pre_Err = 0;
int u_PWM = 1;
float Kp = 0.9, Ki = 0.12, Kd = 0.005;
// float Kp = 1.2, Ki = 0.245, Kd = 0.0;
float pPart = 0, iPart = 0, dPart = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  position = __HAL_TIM_GET_COUNTER(htim);
}

void PID_control_speed(int sp)
{
  Err = sp - speed;
  // PID
  pPart = Kp * Err;
  dPart = Kd * (Err - Pre_Err) * 1000 / 50; // sample time 100ms -> /100ms = *10
  iPart += Ki * 50 / 1000 * Err;
  u_PWM = pPart + dPart + iPart;
  // saturation
  if (u_PWM >= 100)
    u_PWM = 100 - 1;
  if (u_PWM <= 0)
    u_PWM = 50;
  // update PWM
  TIM1->CCR1 = u_PWM;
  Pre_Err = Err;
}

void delay_us(uint16_t time)
{
  /* change your code here for the delay in microseconds */
  __HAL_TIM_SET_COUNTER(&htim4, 0);
  while ((__HAL_TIM_GET_COUNTER(&htim4)) < time)
    ;
}

void step_motor_x(int x)
{
  x_axis += x;
  if (x >= 0)
    x_nghich;
  else
  {
    x_thuan;
    x = -x;
  }
  x_ENABLE;
  for (int i = 0; i < x; i++)
  {
    x_HIGH;
    delay_us(2000);
    x_LOW;
    delay_us(2000);
  }
  x_DISABLE;
}
void step_motor_y(int y)
{
  y_axis += y;
  if (y >= 0)
    y_thuan;
  else
  {
    y = -y;
    y_nghich;
  }
  y_ENABLE;
  for (int i = 0; i < y; i++)
  {
    y_HIGH;
    delay_us(2000);
    y_LOW;
    delay_us(2000);
  }
  y_DISABLE;
}
void control_speed(int Speed, int direction)
{
  if (direction == 0)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    dir = 0;
    //PID_control_speed(Speed);
		TIM1->CCR1 = 80;
    // printf("%d\t %d\t %d\n", position, speed, u_PWM);
    HAL_Delay(50);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    dir = 1;
    //PID_control_speed(Speed);
		TIM1->CCR1 = 80;
    // printf("%d\t %d\t %d\n", position, speed, u_PWM);
    HAL_Delay(50);
  }
}
void gap_vat()
{
  HAL_Delay(1000);
  htim3.Instance->CCR1 = 190; // 0.5ms   0� gap
  HAL_Delay(2000);
}
void nha_vat()
{
  HAL_Delay(1000);
  htim3.Instance->CCR1 = 135; // nha
  HAL_Delay(2000);
}
void Go_home_x()
{
  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) != 0)
  {
    x_thuan;
    x_ENABLE;
    x_HIGH;
    delay_us(2000);
    x_LOW;
    delay_us(2000);
  }
  x_DISABLE;
  x_axis = 0;
}
void Go_home_y()
{
  while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) != 0)
  {
    y_nghich;
    y_ENABLE;
    y_HIGH;
    delay_us(2000);
    y_LOW;
    delay_us(2000);
  }
  y_DISABLE;
  y_axis = 0;
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
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  MFRC522_Init();
  HAL_TIM_Encoder_Start_IT(&htim2, TIM_CHANNEL_ALL);
  TIM1->CCR1 = 80;
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim4);
  HAL_UART_Receive_IT(&huart1, (uint8_t *)&buffer, 1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  htim3.Instance->CCR1 = 135; // nha vat
  Go_home_y();
  Go_home_x();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*// RFID
    status = MFRC522_Request(PICC_REQIDL, str);
    status = MFRC522_Anticoll(str);
    memcpy(sNum, str, 5);
    HAL_Delay(100);
    if((str[0]==19) && (str[1]==221) && (str[2]==201) && (str[3]==12) && (str[4]==11) )
    {
      kq = 0;
      HAL_Delay(10000);
    }
    else if((str[0]==99) && (str[1]==240) && (str[2]==138) && (str[3]==53) && (str[4]==44) )
    {
      kq = 1;
      HAL_Delay(10000);
    }
    else if((str[0]==0x63) && (str[1]==0xA3) && (str[2]==0x2A) && (str[3]==0xFB) && (str[4]==0x11) )
    {
      kq = 2;
      HAL_Delay(10000);
    }
    else if((str[0]==0x53) && (str[1]==0x16) && (str[2]==0x36) && (str[3]==0xFB) && (str[4]==0x88) )
    {
      kq = 3;
      HAL_Delay(10000);
    }
    else if((str[0]==0x13) && (str[1]==0xD3) && (str[2]==0x38) && (str[3]==0xFB) && (str[4]==0x03) )
    {
      kq = 4;
      HAL_Delay(10000);
    }
    else if((str[0]==0x03) && (str[1]==0x74) && (str[2]==0x1D) && (str[3]==0xFB) && (str[4]==0x91) )
    {
      kq = 5;
      HAL_Delay(10000);
    }
    else if((str[0]==0x83) && (str[1]==0xD3) && (str[2]==0x3B) && (str[3]==0xFB) && (str[4]==0x90) )
    {
      kq = 6;
      HAL_Delay(10000);
    }
    else {
      kq = 44;
    }*/
    //k = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
   /* // PID
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    //PID_control_speed(300);
    printf("%d\t %d\t %d\n", position, speed, u_PWM);
    HAL_Delay(50);
		*/
    // step motor
    // x axis
    /*x_thuan;
    x_ENABLE;
    for (int i = 0; i < 200; i++)
    {
      x_HIGH;
      delay_us(1000);
      x_LOW;
      delay_us(1000);
    }
    x_DISABLE;
    HAL_Delay(1000);

    x_nghich;
    x_ENABLE;
    for (int i = 0; i < 200; i++)
    {
      x_HIGH;
      delay_us(1000);
      x_LOW;
      delay_us(1000);
    }
    x_DISABLE;
    HAL_Delay(1000);*/
    // y axis
    /*gap_vat();
    y_thuan;
    y_ENABLE;
    for (int i = 0; i < 550; i++)
    {
      y_HIGH;
      delay_us(1000);
      y_LOW;
      delay_us(1000);
    }
    y_DISABLE;
    HAL_Delay(1000);
    nha_vat();
    y_nghich;
    y_ENABLE;
    for (int i = 0; i < 550; i++)
    {
      y_HIGH;
      delay_us(1000);
      y_LOW;
      delay_us(1000);
    }
    y_DISABLE;
    HAL_Delay(1000);
    */

    // code
    if (nhap_hang == 1)
    {
      if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == 1)
        control_speed(500, 0);
      else
      {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
        rfid = -1;
        if (sp == '1')
        {
          nha_vat();
          step_motor_y(400 - y_axis);
          Go_home_x();
          step_motor_y(800 - y_axis);
          gap_vat();
          Go_home_y();
          nha_vat();
          sp = 0;
        }
        else if (sp == '2')
        {
          nha_vat();
          step_motor_y(400 - y_axis);
          Go_home_x();
          step_motor_y(800 - y_axis);
          gap_vat();
          step_motor_y(400 - y_axis);
          step_motor_x(600 - x_axis);
          Go_home_y();
          nha_vat();
          sp = 0;
        }
        else if (sp == '3')
        {
          nha_vat();
          step_motor_y(400 - y_axis);
          Go_home_x();
          step_motor_y(800 - y_axis);
          gap_vat();
          step_motor_y(400 - y_axis);
          step_motor_x(1200 - x_axis);
          Go_home_y();
          nha_vat();
          sp = 0;
        }
        else if (sp == '4')
        {
          nha_vat();
          step_motor_y(400 - y_axis);
          Go_home_x();
          step_motor_y(800 - y_axis);
          gap_vat();
          Go_home_y();
          nha_vat();
          sp = 0;
        }
        else if (sp == '5')
        {
          nha_vat();
          step_motor_y(400 - y_axis);
          Go_home_x();
          step_motor_y(800 - y_axis);
          gap_vat();
          step_motor_y(400 - y_axis);
          step_motor_x(600 - x_axis);
          Go_home_y();
          nha_vat();
          sp = 0;
        }
        else if (sp == '6')
        {
          nha_vat();
          step_motor_y(400 - y_axis);
          Go_home_x();
          step_motor_y(800 - y_axis);
          gap_vat();
          step_motor_y(400 - y_axis);
          step_motor_x(1200 - x_axis);
          Go_home_y();
          nha_vat();
          sp = 0;
        }
      }
      status = MFRC522_Request(PICC_REQIDL, str);
      status = MFRC522_Anticoll(str);
      memcpy(sNum, str, 5);
      // HAL_Delay(100);
      if ((str[0] == 19) && (str[1] == 221) && (str[2] == 201) && (str[3] == 12) && (str[4] == 11) && rfid != 0)
      {
        sp = '0';
        rfid = 0;
        HAL_UART_Transmit(&huart1, (uint8_t *)&sp, sizeof(sp), 100);
      }
      else if ((str[0] == 99) && (str[1] == 240) && (str[2] == 138) && (str[3] == 53) && (str[4] == 44) && rfid != 1)
      {
        sp = '1';
        rfid = 1;
        HAL_UART_Transmit(&huart1, (uint8_t *)&sp, sizeof(sp), 100);
      }
      else if ((str[0] == 0x63) && (str[1] == 0xA3) && (str[2] == 0x2A) && (str[3] == 0xFB) && (str[4] == 0x11) && rfid != 2)
      {
        sp = '2';
        rfid = 2;
        HAL_UART_Transmit(&huart1, (uint8_t *)&sp, sizeof(sp), 100);
      }
      else if ((str[0] == 0x53) && (str[1] == 0x16) && (str[2] == 0x36) && (str[3] == 0xFB) && (str[4] == 0x88) && rfid != 3)
      {
        sp = '3';
        rfid = 3;
        HAL_UART_Transmit(&huart1, (uint8_t *)&sp, sizeof(sp), 100);
      }
      else if ((str[0] == 0x13) && (str[1] == 0xD3) && (str[2] == 0x38) && (str[3] == 0xFB) && (str[4] == 0x03) && rfid != 4)
      {
        sp = '4';
        rfid = 4;
        HAL_UART_Transmit(&huart1, (uint8_t *)&sp, sizeof(sp), 100);
      }
      else if ((str[0] == 0x03) && (str[1] == 0x74) && (str[2] == 0x1D) && (str[3] == 0xFB) && (str[4] == 0x91) && rfid != 5)
      {
        sp = '5';
        rfid = 5;
        HAL_UART_Transmit(&huart1, (uint8_t *)&sp, sizeof(sp), 100);
      }
      else if ((str[0] == 0x83) && (str[1] == 0xD3) && (str[2] == 0x3B) && (str[3] == 0xFB) && (str[4] == 0x90) && rfid != 6)
      {
        sp = '6';
        rfid = 6;
        HAL_UART_Transmit(&huart1, (uint8_t *)&sp, sizeof(sp), 100);
      }
    }
    else if (nhap_hang == 2)
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
      if (kt == 1 && done < 100)
      {
        if (done_step == 0)
        {
          nha_vat();
          Go_home_x();
          Go_home_y();
          gap_vat();
          step_motor_y(400);
          step_motor_x(800);
          step_motor_y(400);
          nha_vat();
          done_step = 1;
        }
        done++;
        control_speed(500, 1);
      }
      else if (kt == 2 && done < 100)
      {
        if (done_step == 0)
        {
          nha_vat();
          step_motor_x(600 - x_axis);
          Go_home_y();
          gap_vat();
          step_motor_y(400);
          step_motor_x(800 - x_axis);
          step_motor_y(400);
          nha_vat();
          done_step = 1;
        }
        done++;
        control_speed(500, 1);
      }
      else if (kt == 3 && done < 100)
      {
        if (done_step == 0)
        {
          nha_vat();
          step_motor_x(1200 - x_axis);
          Go_home_y();
          gap_vat();
          step_motor_y(400);
          step_motor_x(800 - x_axis);
          step_motor_y(400);
          nha_vat();
          done_step = 1;
        }
        done++;
        control_speed(500, 1);
      }
      else if (kt == 4 && done < 100)
      {
        if (done_step == 0)
        {
          nha_vat();
          Go_home_x();
          Go_home_y();
          gap_vat();
          step_motor_y(400);
          step_motor_x(800);
          step_motor_y(400);
          nha_vat();
          done_step = 1;
        }
        done++;
        control_speed(500, 1);
      }
      else if (kt == 5 && done < 100)
      {
        if (done_step == 0)
        {
          nha_vat();
          step_motor_x(600 - x_axis);
          Go_home_y();
          gap_vat();
          step_motor_y(400);
          step_motor_x(800 - x_axis);
          step_motor_y(400);
          nha_vat();
          done_step = 1;
        }
        done++;
        control_speed(500, 1);
      }
      else if (kt == 6 && done < 100)
      {
        if (done_step == 0)
        {
          nha_vat();
          step_motor_x(1200 - x_axis);
          Go_home_y();
          gap_vat();
          step_motor_y(400);
          step_motor_x(800 - x_axis);
          step_motor_y(400);
          nha_vat();
          done_step = 1;
        }
        done++;
        control_speed(500, 1);
      }
      if (done == 100)
      {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
      }
    }
    else
    {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 99;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 720-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_3
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB3
                           PB5 PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_3
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
