# UsbMouse-project
Usb通信协议应用

通过USBD_HID_SendReport()函数上报鼠标的右键事件。

```c

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
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* 鼠标右键点击 */
	  MouseData1[0] = 0x02; 
	  
	 USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&MouseData1,sizeof(MouseData1));

	  HAL_Delay(1000);

   /*鼠标右键松开  */
	  MouseData1[0] = 0x00;
	  
	  USBD_HID_SendReport(&hUsbDeviceFS,(uint8_t*)&MouseData1,sizeof(MouseData1));
    
	  HAL_Delay(1000);
	  
  }
  /* USER CODE END 3 */
}

```
这种方式只是简单的实现了鼠标反复点击右键的功能，接下来需要通过STM32f103c8t6芯片实现鼠标的左右按键控制。
