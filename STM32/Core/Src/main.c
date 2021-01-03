/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  insert wifi interface : 2020/12/31 , by DIHASYS, JinHo KANG
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dfsdm.h"
#include "i2c.h"
#include "octospi.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dsl.h"
#include "hts221.h"
#include "b_l4s5i_iot01a_bus.h"
#include <stdio.h>
#include "dwt_stm32_delay.h"
#include "es_wifi_io.h"
#include "wifi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
void HCSR04_Read (void);
void LCD_line1_clear (void);
void  WIFI_init_routine(void);
void TNS_to_Thingspark_per30sec(void);
void LCD_backlight_blue_off(void);
void LCD_backlight_blue_on(void);  // max29
void LCD_backlight_red_off(void);
void LCD_backlight_red_on(void);  // max29

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define COUNTOF(__BUFFER__)  (sizeof(__BUFFER__)/sizeof(*(__BUFFER__)))

#define SOCKET_NUM (0)
#define TIMEOUT (20)
#define BUF_SZ (1024)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	uint8_t  rx_data;

	volatile uint16_t TIM15_10msec_count = 0;
	volatile uint16_t TIM15_wifi_dly_cnt = 0;
	volatile uint16_t TIM15_impact_dly_cnt = 0;
	volatile uint16_t TIM15_gpio_out_dly_cnt = 0;
	volatile uint16_t TIM15_OP_LED_cnt = 0;
	volatile uint16_t TIM15_0W0_cnt = 0;

	LSM6DSL_Object_t MotionSensor;
	volatile uint32_t dataRdyIntReceived;

	HTS221_Object_t HumTempSensor;
	float HTS221_tmp = 0.0;
	float HTS221_hum = 0.0;

//	volatile char LCD_buffer[32+1];  // 16 * 2 line + 1 null
	char LCD_clear[] = "|-";
	char LCD_data1[] = "Hell Maker Team by DIHASYS JinHo\r\n";

	char LCD_line1[17];

	char tempDATA1[80];
	char tempDATA2[80];

	// for sonic distance check
	uint32_t Difference = 0;
	uint32_t Is_First_Captured = 0;  // 0 -> 1 -> 0
	uint8_t Distance  = 0;
	uint8_t Is_TRIG_out = 0;  // 0 -> trig -> 1 -> read -> 0

	WIFI_Status_t Wifi_ret;
	uint8_t Wifi_MAC_Addr[6];
	uint8_t Wifi_IP_Addr[4];
	uint8_t Wifi_ipAddr[4];
	uint8_t TNS_0W0_flag = 0;  // 0: , 1:start, 3:send data, 6:close -> 0

	int  axes_x, axes_y, axes_z ;
	int  axes_x1old, axes_y1old, axes_z1old ;
	int  axes_x2old, axes_y2old, axes_z2old ;

	uint16_t  distance_percent ;  // ?��거함 ?���?? ?��?���?? ?��?��, 거리 25cm?��?�� 0%(비었?��), 거리 0(99%),�???���??(95%?��?��).

	uint8_t  impact_flag = 0 ;  // after 5sec, clear
	uint8_t  PD14_low_open_flag = 0 ;  // after 1sec, clear
	uint8_t  PB0_low_close_flag = 0 ;  // after 1sec, clear
	uint8_t  door_status_flag = 0;  // 0:close status, 1:open status
	uint8_t  door_open_dly_time_cnt = 0;  // per 0.5sec

	uint8_t  uart4_RX_buf[10]; // @ o ? ? ? ? cr lf
	uint8_t  uart4_RX_pnt = 0;
	uint8_t  uart4_RX_flag = 0;

	// for TNS_to_Thingspark_per30sec()
  uint8_t Wifi_close_after_1sec_chk = 0;
  uint8_t req_data[BUF_SZ];
  uint16_t sent_data_len = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void MEMS_Init(void);
static void MEMS_HTS221_Init(void);
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

	LSM6DSL_Axes_t acc_axes;

	int i;
	int uart4_dummy_TNS_dly = 0;  // per 2

	bzero(req_data, BUF_SZ);

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
  MX_ADC1_Init();
  MX_DFSDM1_Init();
  MX_I2C1_Init();
  MX_OCTOSPI1_Init();
  MX_SPI3_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_USB_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, &rx_data, 1);  // initial
  HAL_UART_Receive_IT(&huart3, &rx_data, 1);  // initial
  HAL_UART_Receive_IT(&huart4, &rx_data, 1);  // initial
  HAL_TIM_Base_Start_IT(&htim15);

  LCD_line1_clear();

  printf("by programmed DIHASYS , JinHo KANG\r\nStart-->\r\n");

  strcpy(tempDATA1,"test\r\n");
  HAL_UART_Transmit(&huart4, (uint8_t*)tempDATA1, strlen(tempDATA1),100);   // RPi

  HAL_UART_Transmit(&huart2, (uint8_t *)LCD_clear, (COUNTOF(LCD_clear)-1), 50);
  HAL_Delay(2*2) ; // 2ms * 2char

  for(i=0; i < 80; i++) tempDATA1[i] = tempDATA2[i] = 0;

  HAL_UART_Transmit(&huart2, (uint8_t *)LCD_data1, (COUNTOF(LCD_data1)-1), 50);
  // HAL_Delay(2*32) ; // 2ms * 32char


  if(DWT_Delay_Init())  // 1: not start , 0: counter works
  {
  // Error_Handler(); /* Call Error Handler */
     printf("Not start DWT_Clock_Cycle_Count \r\n");  // 1: Not start
  }

  i = 0;
  dataRdyIntReceived = 0;
  MEMS_Init();

  MEMS_HTS221_Init();

  WIFI_init_routine();

  PB0_low_close_flag = 1;
  TIM15_gpio_out_dly_cnt = 0;
  HAL_GPIO_WritePin(PB0_CLOSE_LOW_GPIO_Port, PB0_CLOSE_LOW_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(TIM15_10msec_count > 100)  // per 1 == 0.01 * 100
	  {
		  TIM15_10msec_count = 0;

		  if (dataRdyIntReceived != 0)
		  {
			  dataRdyIntReceived = 0;
			  LSM6DSL_ACC_GetAxes(&MotionSensor, &acc_axes);
		  }

		  axes_x = (int) acc_axes.x;  axes_y = (int) acc_axes.y;  axes_z = (int) acc_axes.z;
		  axes_x2old = axes_x1old;    axes_y2old = axes_y1old;    axes_z2old = axes_z1old;
		  axes_x1old = axes_x;        axes_y1old = axes_y;        axes_z1old = axes_z;

		  impact_flag = 0;
			  if( DIFF_BS(axes_x2old,axes_x) > 200 ) impact_flag++;
			  if( DIFF_BS(axes_y2old,axes_y) > 200 ) impact_flag++;
			  if( DIFF_BS(axes_z2old,axes_z) > 200 ) impact_flag++;

		  if(impact_flag != 0)
		  {
			  if(TIM15_impact_dly_cnt > 500)
			  {
				  TIM15_impact_dly_cnt = 0;
				  strncpy(LCD_line1, "Impact" , 6);
				  LCD_backlight_red_on();  // max29
			  }
		  }
		  else // == 0
		  {
			  if(TIM15_impact_dly_cnt > 500)
			  {
				  if(LCD_line1[0] != 0x20)
				  {
					  strncpy(LCD_line1, "      ", 6);
					  LCD_backlight_red_off();
				  }
			  }
		  }

		  if(Distance > 25)  Distance = 25;
		  distance_percent = (uint16_t) Distance * 396;
		  distance_percent /= 100;
		  distance_percent = 99 - distance_percent;

		  HTS221_HUM_GetHumidity(&HumTempSensor, &HTS221_hum);
		  HTS221_TEMP_GetTemperature(&HumTempSensor, &HTS221_tmp);

		  HAL_UART_Transmit(&huart2, (uint8_t *)LCD_clear, (COUNTOF(LCD_clear)-1), 50);

		  strcpy(tempDATA1,LCD_line1);
		  sprintf(tempDATA2,"%3d%% h%2.1f,t%2.1f", distance_percent, HTS221_hum, HTS221_tmp );
		  strcat(tempDATA1,tempDATA2);
		  HAL_UART_Transmit(&huart2, (uint8_t *)tempDATA1, strlen(tempDATA1), 50);

		  sprintf(tempDATA1,"axes_x:%5d, y:d%5d, z:d%5d |  ", axes_x, axes_y, axes_z);
		  sprintf(tempDATA2,"2old_x:%5d, y:d%5d, z:d%5d\r\n", axes_x2old, axes_y2old, axes_z2old);
		  strcat(tempDATA1,tempDATA2);
		  HAL_UART_Transmit(&huart1, (uint8_t *)tempDATA1, strlen(tempDATA1), 50);

		  Is_TRIG_out = 0;
	  }
	  else if(TIM15_10msec_count > 90)  // 90 ~ 100 => 100ms
	  {
		  if(Is_TRIG_out == 0)
		  {
			  HCSR04_Read();
			  Is_TRIG_out++;
		  }
	  }

	  TNS_to_Thingspark_per30sec();  // Wifi --> CLOUD , distance_percent, HTS221_tmp, HTS221_hum

	  // TNS_addition http://mask.0-w-0.com/api/collection.php?id=1&action=1&value1=휴지통용량&value2=온도&value3=습도
	  // TNS_to_Thingspark_per30sec CLOSE => 1sec delay => TNS TNS_addition
	  TNS_to_0W0_after_Thingspark();  // Wifi --> another CLOUD , distance_percent, HTS221_tmp, HTS221_hum

	  if(uart4_RX_flag != 0)
	  {
		  uart4_RX_flag = 0;
		  if((uart4_RX_buf[1] == 'o') || (uart4_RX_buf[1] == 'O'))  // ?��거통 ?���?? OPEN명령
		  {
			  PD14_low_open_flag = 1;
			  TIM15_gpio_out_dly_cnt = 0;
			  HAL_GPIO_WritePin(PD14_OPEN_LOW_GPIO_Port, PD14_OPEN_LOW_Pin, GPIO_PIN_RESET);
		  }
		  if((uart4_RX_buf[1] == 'c') || (uart4_RX_buf[1] == 'C'))  // ?��거통 ?���?? CLOSE명령
		  {
			  PB0_low_close_flag = 1;
			  TIM15_gpio_out_dly_cnt = 0;
			  HAL_GPIO_WritePin(PB0_CLOSE_LOW_GPIO_Port, PB0_CLOSE_LOW_Pin, GPIO_PIN_RESET);
		  }
		  uart4_RX_buf[1] = ' ';
	  }

	  if(PD14_low_open_flag != 0)
	  {
		  if(TIM15_gpio_out_dly_cnt > 100)
		  {
			  PD14_low_open_flag = 0;
			  HAL_GPIO_WritePin(PD14_OPEN_LOW_GPIO_Port, PD14_OPEN_LOW_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(PB0_CLOSE_LOW_GPIO_Port, PB0_CLOSE_LOW_Pin, GPIO_PIN_SET);

			  door_open_dly_time_cnt = 0;  // per 0.5sec
			  door_status_flag = 1;  // 0:close status, 1:open status
			  LCD_backlight_blue_on();  // max29
		  }
	  }
	  if(PB0_low_close_flag != 0)
	  {
		  if(TIM15_gpio_out_dly_cnt > 100)
		  {
			  PB0_low_close_flag = 0;
			  HAL_GPIO_WritePin(PB0_CLOSE_LOW_GPIO_Port, PB0_CLOSE_LOW_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(PD14_OPEN_LOW_GPIO_Port, PD14_OPEN_LOW_Pin, GPIO_PIN_SET);

			  door_open_dly_time_cnt = 0;  // per 0.5sec
			  door_status_flag = 0;  // 0:close status, 1:open status
			  LCD_backlight_blue_off();
		  }
	  }

	  if(TIM15_OP_LED_cnt > 50)
	  {
		  TIM15_OP_LED_cnt = 0;
		  HAL_GPIO_TogglePin(USER_LED2_GPIO_Port, USER_LED2_Pin);
		  HAL_GPIO_TogglePin(PB4_UVLED_GPIO_Port, PB4_UVLED_Pin);

		  door_open_dly_time_cnt++;
		  if(door_status_flag != 0)  // 0:close status, 1:open status
		  {
			  if(door_open_dly_time_cnt > 16)  // == 8sec = 0.5 * 16
			  {
				  PB0_low_close_flag = 1;
				  TIM15_gpio_out_dly_cnt = 0;
				  HAL_GPIO_WritePin(PB0_CLOSE_LOW_GPIO_Port, PB0_CLOSE_LOW_Pin, GPIO_PIN_RESET);
				  door_status_flag = 0;  // 0:close status, 1:open status
			  }
		  }

		  uart4_dummy_TNS_dly++;
		  if(uart4_dummy_TNS_dly > 3) // 0 1 2 3
		  {
			  uart4_dummy_TNS_dly = 0;
			  strcpy(tempDATA1,"test\r\n");
			  HAL_UART_Transmit(&huart4, (uint8_t*)tempDATA1, strlen(tempDATA1),100);   // HC-05
		  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_DFSDM1|RCC_PERIPHCLK_USB
                              |RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_OSPI;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.OspiClockSelection = RCC_OSPICLKSOURCE_SYSCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK|RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
	case (ISM43362_DRDY_EXTI1_Pin) :
		SPI_WIFI_ISR();
		break;

	case (LSM6DSL_INT1_EXTI11_Pin) :
		dataRdyIntReceived++;
		break;

	case (BUTTON_EXTI13_Pin) : // user switch

	  PD14_low_open_flag = 1;
	  TIM15_gpio_out_dly_cnt = 0;
	  HAL_GPIO_WritePin(PD14_OPEN_LOW_GPIO_Port, PD14_OPEN_LOW_Pin, GPIO_PIN_RESET);

		break;

	case (PULSE_ECHO_EXTI15_Pin) :
			if (HAL_GPIO_ReadPin(PULSE_ECHO_EXTI15_GPIO_Port, PULSE_ECHO_EXTI15_Pin) == GPIO_PIN_SET)
			{
				DWT->CYCCNT = 0;  // reset counter
				Is_First_Captured = 1; // set the first captured as true
			}
			else if (Is_First_Captured==1)  // if the first is already captured
			{
				Difference = DWT->CYCCNT; // read second value
				Difference /=  (HAL_RCC_GetHCLKFreq() / 1000000);  // 1ms:1000, 1us:1 000 000

				Distance = Difference * .034/2;
				Is_First_Captured = 0; // set it back to false
			}
		break;

	default :
		break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		HAL_UART_Receive_IT(&huart1, &rx_data, 1);
		HAL_UART_Transmit(&huart1, &rx_data, 1,10);
	//	HAL_UART_Transmit(&huart3, &rx_data, 1,10);
	}

	if(huart->Instance == USART3)
	{
		HAL_UART_Receive_IT(&huart3, &rx_data, 1);
		HAL_UART_Transmit(&huart1, &rx_data, 1,10);
	}

	if(huart->Instance == UART4)
	{
		HAL_UART_Receive_IT(&huart4, &rx_data, 1);
		if(rx_data == '@')
		{
			uart4_RX_pnt = 0;
			uart4_RX_buf[0] = rx_data;
		}
		else if(rx_data == 0x0A)
		{
			uart4_RX_pnt++;
			if(uart4_RX_pnt > 10 - 1) uart4_RX_pnt = 5;
			uart4_RX_buf[0] = rx_data;
			uart4_RX_flag = 1;
		}
		else
		{
			uart4_RX_pnt++;
			if(uart4_RX_pnt > 10 - 1) uart4_RX_pnt = 5;
			uart4_RX_buf[uart4_RX_pnt] = rx_data;
		}

		HAL_UART_Transmit(&huart1, &rx_data, 1,10);
	}

}

/*
// for printf()
int __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1,100);
	return ch;
}
*/

// for printf()
int _write(int fd, char * ptr, int len)
{
  HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
  return len;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM15)  // per 0.01sec
	{
		TIM15_10msec_count++;
		TIM15_wifi_dly_cnt++;
		TIM15_impact_dly_cnt++;
		TIM15_gpio_out_dly_cnt++;
		TIM15_OP_LED_cnt++;
		TIM15_0W0_cnt++;
	}
}

static void MEMS_Init(void)
{
  LSM6DSL_IO_t io_ctx;
  uint8_t id;
  LSM6DSL_AxesRaw_t axes;

  /* Link I2C functions to the LSM6DSL driver */
  io_ctx.BusType     = LSM6DSL_I2C_BUS;
  io_ctx.Address     = LSM6DSL_I2C_ADD_L;
  io_ctx.Init        = BSP_I2C2_Init;
  io_ctx.DeInit      = BSP_I2C2_DeInit;
  io_ctx.ReadReg     = BSP_I2C2_ReadReg;
  io_ctx.WriteReg    = BSP_I2C2_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;
  LSM6DSL_RegisterBusIO(&MotionSensor, &io_ctx);

  /* Read the LSM6DSL WHO_AM_I register */
  LSM6DSL_ReadID(&MotionSensor, &id);
  if (id != LSM6DSL_ID) {
    Error_Handler();
  }

  /* Initialize the LSM6DSL sensor */
  LSM6DSL_Init(&MotionSensor);

  /* Configure the LSM6DSL accelerometer (ODR, scale and interrupt) */
  LSM6DSL_ACC_SetOutputDataRate(&MotionSensor, 26.0f); /* 26 Hz */
  LSM6DSL_ACC_SetFullScale(&MotionSensor, 4);          /* [-4000mg; +4000mg] */
  LSM6DSL_ACC_Set_INT1_DRDY(&MotionSensor, ENABLE);    /* Enable DRDY */
  LSM6DSL_ACC_GetAxesRaw(&MotionSensor, &axes);        /* Clear DRDY */

  /* Start the LSM6DSL accelerometer */
  LSM6DSL_ACC_Enable(&MotionSensor);
}

static void MEMS_HTS221_Init(void)
{
  HTS221_IO_t io_ctx;
  uint8_t id;

  /* Link I2C functions to the LSM6DSL driver */
  io_ctx.BusType     = HTS221_I2C_BUS;
  io_ctx.Address     = HTS221_I2C_ADDRESS;
  io_ctx.Init        = BSP_I2C2_Init;
  io_ctx.DeInit      = BSP_I2C2_DeInit;
  io_ctx.ReadReg     = BSP_I2C2_ReadReg;
  io_ctx.WriteReg    = BSP_I2C2_WriteReg;
  io_ctx.GetTick     = BSP_GetTick;
  HTS221_RegisterBusIO(&HumTempSensor, &io_ctx);

  HTS221_ReadID(&HumTempSensor, &id);
  if (id != HTS221_ID) {
    Error_Handler();
  }

  HTS221_Init(&HumTempSensor);

  /* Configure the LSM6DSL accelerometer (ODR, scale and interrupt) */
  HTS221_HUM_SetOutputDataRate(&HumTempSensor, 7.0f); /* 7 Hz */

  // default : disable DRDY output , Not used

  HTS221_HUM_Enable(&HumTempSensor);
  HTS221_TEMP_Enable(&HumTempSensor);
}

void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(PLUSE_TRIG_GPIO_Port, PLUSE_TRIG_Pin, GPIO_PIN_SET);  // pull the PLUSE_TRIG pin HIGH
	DWT->CYCCNT = 0;  // reset counter
	DWT_Delay_us(10);
	HAL_GPIO_WritePin(PLUSE_TRIG_GPIO_Port, PLUSE_TRIG_Pin, GPIO_PIN_RESET);  // pull the PLUSE_TRIG pin low
}

void LCD_line1_clear (void)
{
	uint8_t i;
	for(i=0; i < 16; i++) LCD_line1[i] = 0x20;
	LCD_line1[16] = 0;  //NULL
}

void  WIFI_init_routine(void)
{
  printf("wifi-init ====>>>>>>>>>>>>>>\r\n");
  Wifi_ret = WIFI_Init();
  if(Wifi_ret ==  WIFI_STATUS_OK)
  {
	  if(WIFI_GetMAC_Address(Wifi_MAC_Addr) == WIFI_STATUS_OK)
	  {
		  printf("> es-wifi module MAC Address : %X:%X:%X:%X:%X:%X\n",
				  Wifi_MAC_Addr[0],
				  Wifi_MAC_Addr[1],
				  Wifi_MAC_Addr[2],
				  Wifi_MAC_Addr[3],
				  Wifi_MAC_Addr[4],
				  Wifi_MAC_Addr[5]);
	  } else {
		  printf("> ERROR : CANNOT get MAC address\r\n");
	  }

//	  Wifi_ret = WIFI_Connect("GCAMP0824G", "12345678a", WIFI_ECN_WPA2_PSK);
	  Wifi_ret = WIFI_Connect("DIHASYS", "0221630560", WIFI_ECN_WPA2_PSK);
	  if(Wifi_ret == WIFI_STATUS_OK)
	  {
		  printf("> es-wifi module connected \n");
		  if(WIFI_GetIP_Address(Wifi_IP_Addr) == WIFI_STATUS_OK)
		  {
			  printf("> es-wifi module got IP Address : %d.%d.%d.%d\n",
					  Wifi_IP_Addr[0],
					  Wifi_IP_Addr[1],
					  Wifi_IP_Addr[2],
					  Wifi_IP_Addr[3]);
		  } else
		  {
			  printf("> ERROR : es-wifi module CANNOT get IP address\n");
		  }
	  } else
	  {
		  printf("> ERROR : es-wifi module NOT connected\n");
	  }
  } else
  {
	  printf("> ERROR : WIFI Module cannot be initialized.\n");
  }
}

void TNS_to_Thingspark_per30sec(void)
{
	  if(Wifi_close_after_1sec_chk == 1)
	  {
		  if(TIM15_wifi_dly_cnt > 100)  // per 1 == 0.01 * 100
		  {
			  Wifi_close_after_1sec_chk = 0;
			  WIFI_CloseClientConnection(SOCKET_NUM);

			  TNS_0W0_flag = 1;  // 0: , 1:start, 2:send data, 3:close -> 0
		  }
	  }

	  if(TIM15_wifi_dly_cnt > 3000)  // per 30 == 0.01 * 3000 , for CLOUD
	  {
		  TIM15_wifi_dly_cnt = 0;

		  Wifi_ret = WIFI_GetHostAddress("api.thingspark.co.kr", Wifi_ipAddr);
		  if (Wifi_ret == WIFI_STATUS_OK)
		  {
			  printf("wifi-thingspark-connect\r\n");

			  Wifi_ret = WIFI_OpenClientConnection(SOCKET_NUM, WIFI_TCP_PROTOCOL, "cloud-connect", Wifi_ipAddr, 8480, 0);
			  if (Wifi_ret == WIFI_STATUS_OK)
			  {

				  sprintf(req_data,"GET /update?apiKey=t9PdlZ1AZP63WvTf&field1=%d&field2=%f&field3=%f\r\n", distance_percent, HTS221_tmp, HTS221_hum);

				  WIFI_SendData(SOCKET_NUM, req_data, sizeof(req_data), &sent_data_len, TIMEOUT);

				  TIM15_wifi_dly_cnt = 0;
				  Wifi_close_after_1sec_chk = 1;
			  }
			  else
			  {
				  printf("Failed to connect to server [%d]\r\n", Wifi_ret);
			  }
		  }
		  else
		  {
			  printf("Failed to get host address [%d]\r\n", Wifi_ret);
		  }
	  }
}

void TNS_to_0W0_after_Thingspark(void)  // Wifi --> another CLOUD , distance_percent, HTS221_tmp, HTS221_hum
{
	switch(TNS_0W0_flag)  // 0: , 1:start, 2:send data, 3:close -> 0
	{
	case 0:
		break;

	case 1:
		TIM15_0W0_cnt = 0;
		TNS_0W0_flag++;
		break;

	case 2:
		if(TIM15_0W0_cnt > 200)  // after 2sec
		{
			TNS_0W0_flag++;
		}
		break;

	case 3:
		  Wifi_ret = WIFI_GetHostAddress("mask.0-w-0.com", Wifi_ipAddr);
		  if (Wifi_ret == WIFI_STATUS_OK)
		  {
			  printf("mask.0-w-0.com-connect\r\n");

			  Wifi_ret = WIFI_OpenClientConnection(SOCKET_NUM, WIFI_TCP_PROTOCOL, "cloud-connect", Wifi_ipAddr, 80, 0);
			  if (Wifi_ret == WIFI_STATUS_OK)
			  {

//				  sprintf(req_data,"GET /update?apiKey=t9PdlZ1AZP63WvTf&field1=%d&field2=%f&field3=%f\r\n", distance_percent, HTS221_tmp, HTS221_hum);
			//	  sprintf(req_data,"GET /api/collection.php?id=1&action=1&value1=%d&value2=%f&value3=%f\r\n", distance_percent, HTS221_tmp, HTS221_hum);
				  sprintf(req_data,"GET /api/collection.php?id=1&action=1&value1=%d&value2=%f&value3=%f", distance_percent, HTS221_tmp, HTS221_hum);

				  Wifi_ret = WIFI_SendData(SOCKET_NUM, req_data, sizeof(req_data), &sent_data_len, TIMEOUT);
				  if (Wifi_ret != WIFI_STATUS_OK)
				  {
					  printf("Failed to WIFI_SendData to 0-w-0.com [%d]\r\n", Wifi_ret);
				  }
			  }
			  else
			  {
				  printf("Failed to connect to 0-w-0.com [%d]\r\n", Wifi_ret);
			  }
		  }
		  else
		  {
			  printf("Failed to get 0-w-0.coms [%d]\r\n", Wifi_ret);
		  }

		TNS_0W0_flag++;
		break;

	case 4:
		TIM15_0W0_cnt = 0;
		TNS_0W0_flag++;
		break;

	case 5:
		if(TIM15_0W0_cnt > 100)  // after 1sec
		{
			TNS_0W0_flag++;
		}
		break;

	case 6:
		WIFI_CloseClientConnection(SOCKET_NUM);
		TNS_0W0_flag++;
		break;

	case 7:
	default :
		TNS_0W0_flag = 0;
		break;

	}

}


void LCD_backlight_blue_off(void)
{
	tempDATA1[0] = '|';	tempDATA1[1] = 188;
	tempDATA1[2] = 0;
	HAL_UART_Transmit(&huart2, (uint8_t*)tempDATA1, strlen(tempDATA1),100);
}

void LCD_backlight_blue_on(void)  // max29
{
	tempDATA1[0] = '|';	tempDATA1[1] = 188+29;
	tempDATA1[2] = 0;
	HAL_UART_Transmit(&huart2, (uint8_t*)tempDATA1, strlen(tempDATA1),100);
}

void LCD_backlight_red_off(void)
{
	tempDATA1[0] = '|';	tempDATA1[1] = 128;
	tempDATA1[2] = 0;
	HAL_UART_Transmit(&huart2, (uint8_t*)tempDATA1, strlen(tempDATA1),100);
}

void LCD_backlight_red_on(void)  // max29
{
	tempDATA1[0] = '|';	tempDATA1[1] = 128+29;
	tempDATA1[2] = 0;
	HAL_UART_Transmit(&huart2, (uint8_t*)tempDATA1, strlen(tempDATA1),100);
}

// Made by JinHo of DIHASYS

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
//	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//	  HAL_Delay(50); /* wait 50 ms */
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
