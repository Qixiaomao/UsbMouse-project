/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
#include "usbd_hid.h"
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint8_t MouseData1[4] = {0,0,0,0};
unsigned char keyNum = 0;


extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t USBD_HID_SendReport(USBD_HandleTypeDef *pdev,
	                       uint8_t *report,
                           uint16_t len);



//构造一个鼠标事件
typedef struct{
	char mouse_abs_left : 1;  //左键单击
	char mouse_abs_right : 1;  //右键单机
	char mouse_abs_wheel : 1;  //中键单击
	char reserve : 5;  //常量0
	char mouse_rel_x;  //鼠标x轴移动
	char mouse_rel_y;  //鼠标y轴移动
	char mouse_rel_wheel;  //鼠标滚轮移动值
}tyMouse_buf;

/* USER CODE END PTD */

#define KEY0_Press (1<<0)
#define KEY1_Press (1<<1)
#define KEY2_Press (1<<2)
#define KEY3_Press (1<<3)

tyMouse_buf tMouse_buff;

void User_init(void)
{
	tMouse_buff.mouse_abs_left = 0;
	tMouse_buff.mouse_abs_right = 0;
	tMouse_buff.mouse_abs_wheel = 0;
	tMouse_buff.reserve = 0;
	tMouse_buff.mouse_rel_x = 0;
	tMouse_buff.mouse_rel_y = 0;
	tMouse_buff.mouse_rel_wheel = 0;
}
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//读取Key的按键值
unsigned char Get_key_num(void)
{
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0)
	{
		HAL_Delay(20); //按键消抖操作
		while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1)==0);
		HAL_Delay(20);
		keyNum = 1;
	}
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)==0)
	{
		HAL_Delay(20);
		while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)==0);
		HAL_Delay(20);
		keyNum = 2;
	}

	return keyNum;
}

void Send_mouse_msg(void)
{
	USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t *)&tMouse_buff,sizeof(tMouse_buff));
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

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
  
  //unsigned char keyNumLast;
  //int cnt;
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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  keyNum = Get_key_num();
    if(keyNum == 1)
	{
		MouseData1[0] = 0x02; //鼠标右键按下
		USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&MouseData1,sizeof(MouseData1));
	}
    if(keyNum == 2)
	{
		MouseData1[0]=0x01; //鼠标左键按下
	   USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&MouseData1,sizeof(MouseData1));
	}
  }
  
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	//GPIO 接口时钟使能
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	
	//配置GPIO 接口：B11 B1
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_11;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOE,&GPIO_InitStruct);
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
