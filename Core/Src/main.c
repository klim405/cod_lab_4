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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ring_buffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define TX6_BUFFER_SIZE 1000

#define I2C_TIMEOUT 100
#define GPIO_EXPANDER_ADDRESS 0xE2
#define GPIO_EXP_IN_REG 0x00
#define GPIO_EXP_OUT_REG 0x01
#define GPIO_EXP_INV_REG 0x02
#define GPIO_EXP_CONFIG_REG 0x03
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char __uart6_tx_buff[TX6_BUFFER_SIZE];
RingBuffer uart6_tx_buff;
bool UART6_TX_IsReady = true;
uint8_t uart6_tx_byte_buff[4];

enum I2C_Status {
  I2C_WRITE_COMPLETE,
  I2C_READ_COMPLETE = 0,
};
enum I2C_Status keyboard_i2c_status = I2C_READ_COMPLETE;
uint8_t keyboard_i2c_byte_buff[1];
uint8_t keybord_lines_buff[4];
int8_t keyboard_curr_line = -1;

bool KeyBoard_IsRefreshTime = false;

char KeyBoard_PressedSymbol;
bool KeyBoard_HasPressedSymbol = false;

typedef struct __UserInput {
  int16_t operand[2];
  uint16_t curr_op;
  int8_t dec_place;
  char action;
} UserInputTypeDef;

UserInputTypeDef UserInput;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* UART6 ---------------------------------------------------------------------*/
/**
 * @brief Пытается отправить данные из кольцевого буфера
 */
void UART6_TryToTransmit_IT() {
  if (UART6_TX_IsReady && !RingBuffer_IsEmpty(&uart6_tx_buff)) {
    RingBuffer_Read(&uart6_tx_buff, uart6_tx_byte_buff, 1);
    UART6_TX_IsReady = false;
    HAL_UART_Transmit_IT(&huart6, (uint8_t*) uart6_tx_byte_buff, 1);
  } 
}

/**
 * @brief Отправляет байт в режиме прерываний
 * @return true - байт записан в буфер отправки
 * @return false - буфер отправки переполнен
 */
bool UART6_TransmitByte(char byte) {
  bool result = false;
  if (!uart6_tx_buff.isFull) {
      RingBuffer_Write(&uart6_tx_buff, &byte, 1);
      result = true;
    }
  UART6_TryToTransmit_IT();
  return result;
}

/**
 * @brief Получает байт по uart
 * @return true - данные записан в буфер отправки
 * @return false - буфер отправки переполнен
 */
bool UART6_TransmitString(char* str) {
  bool result = false;
  if (!uart6_tx_buff.isFull) {
    RingBuffer_Write(&uart6_tx_buff, str, strlen(str));
  }
  UART6_TryToTransmit_IT();
  return result;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* UartHandle) {
  if (UartHandle == &huart6) {
    if (!RingBuffer_IsEmpty(&uart6_tx_buff)) {
      RingBuffer_Read(&uart6_tx_buff, uart6_tx_byte_buff, 1);
      HAL_UART_Transmit_IT(&huart6, (uint8_t*) uart6_tx_byte_buff, 1);
    } else {
      UART6_TX_IsReady = true;
    }
  }
}

void SetError(void) {
  UART6_TransmitString("\nerror\n");
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
}

void ResetError(void) {
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
}

/* Key Board & I2C -----------------------------------------------------------*/
char KeyBoard_DecodeKey(int8_t key_number) {
  if (key_number >= 0 && key_number < 9) {
    return ((char) key_number) + '1';
  }
  switch (key_number) {
    case 9: return '=';
    case 10: return '0';
    case 11: return '+';
    case 12: return '-';
    case 13: return '*';
    case 14: return '/';
    default: return '\0';
  }
}

void KeyBoard_DefineSymbol() {
  bool cmdIsSet = (keybord_lines_buff[4] == 0x4);
  int8_t total = 0;
  int8_t sym = 0;
  uint8_t curr_line;
  for (int8_t i = 0; i < 4; i++) {
    curr_line = keybord_lines_buff[i];
    for (int8_t j = 0; j < 3; j++) {
      if ((curr_line >> j) & 0x1) {
        ++total;
        sym += i*j;
      }
    }
  }
  char curr_char;
  if ((cmdIsSet && total == 2) || (!cmdIsSet && total == 1)) {
    curr_char = KeyBoard_DecodeKey(sym);
    if (KeyBoard_PressedSymbol != curr_char) {
      KeyBoard_PressedSymbol = curr_char;
      KeyBoard_HasPressedSymbol = true;
    }
  }
}

HAL_StatusTypeDef I2C1_GPIOExpander_Write(uint16_t RegAddess, uint8_t* Buffer, uint16_t Size) {
  return HAL_I2C_Mem_Write(&hi2c1, GPIO_EXPANDER_ADDRESS, RegAddess, 1, Buffer, Size, I2C_TIMEOUT);
}

HAL_StatusTypeDef I2C1_GPIOExpander_Read(uint16_t RegAddess, uint8_t* Buffer, uint16_t Size) {
  return HAL_I2C_Mem_Read(&hi2c1, GPIO_EXPANDER_ADDRESS, RegAddess, 1, Buffer, Size, I2C_TIMEOUT);
}

uint8_t KeyBoard_RefreshLine(int8_t line) {
  HAL_StatusTypeDef status;
  uint8_t tmp;
  tmp = ~(1 << line);
  I2C1_GPIOExpander_Write(GPIO_EXP_INV_REG, &tmp, 1);
  tmp = ~(1 << line);
  I2C1_GPIOExpander_Write(GPIO_EXP_OUT_REG, &tmp, 1);
  tmp = ~(1 << line);
  I2C1_GPIOExpander_Write(GPIO_EXP_CONFIG_REG, &tmp, 1);
  HAL_Delay(100);
  tmp = 0;
  I2C1_GPIOExpander_Read(GPIO_EXP_IN_REG, &tmp, 1);
  return (tmp >> 4) & 0x7;
}

void KeyBoard_Refresh() {
  uint8_t tmp = KeyBoard_RefreshLine(keyboard_curr_line);
  // HAL_UART_Transmit(&huart6, &tmp, 1, 40);
  keybord_lines_buff[keyboard_curr_line] = tmp;
  ++keyboard_curr_line;
  if (keyboard_curr_line > 3) {
    keyboard_curr_line = 0;
    KeyBoard_DefineSymbol();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    KeyBoard_IsRefreshTime = true;
  }
}

/* User Input ----------------------------------------------------------------*/
void ResetInput(void) {
  UserInput.operand[0] = 0;
  UserInput.operand[1] = 0;
  UserInput.curr_op = 0;
  UserInput.dec_place = 0;
}

void PrintResult(void) {
  int16_t a = UserInput.operand[0];
  int16_t b = UserInput.operand[1];
  int32_t result = 0;
  char result_str[32];
  switch (UserInput.action) {
    case '+': 
      result = a + b;
      break;
    case '-':
      result = a - b;
      break;
    case '*': 
      result = a * b;
      break;
    case '/':
      result = a / b;
      break;
  }
  if (result < INT16_MIN || result > INT16_MAX || (UserInput.action == '/' && b == 0)) {
    SetError();
    ResetInput();
  } else {
    UART6_TransmitString(itoa(result, result_str, 10));
    UART6_TransmitByte('\n');
    ResetInput();
  }
}

void ProcessInput(char sym) {
  if (isdigit(sym) && UserInput.dec_place++ < 5) {
    ResetError();
    UserInput.operand[UserInput.curr_op] = UserInput.operand[UserInput.curr_op] * 10 + (int16_t) (sym - '0');
    if (UserInput.operand[UserInput.curr_op] < 0) {
      SetError();
      ResetInput();
    }
  } else if ((sym == '+' || sym == '-' || sym == '/' || sym == '*') && UserInput.curr_op == 0) {
    UserInput.dec_place = 0;
    ++UserInput.curr_op;
    UserInput.action = sym;
  } else if (sym == '=' && UserInput.curr_op == 1) {
    PrintResult();
  } else {
    SetError();
    ResetInput();
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  RingBuffer_Init(&uart6_tx_buff, __uart6_tx_buff, TX6_BUFFER_SIZE);
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
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim6);
  while (1)
  {
    if (KeyBoard_IsRefreshTime) {
      KeyBoard_IsRefreshTime = false;
      KeyBoard_Refresh();
    }
    if (KeyBoard_HasPressedSymbol && KeyBoard_PressedSymbol != '\0') {
      UART6_TransmitByte(KeyBoard_PressedSymbol);
      KeyBoard_HasPressedSymbol = false;
      ProcessInput(KeyBoard_PressedSymbol);
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
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
