/* USER CODE BEGIN Header */
/*
 * nanoAmmeter version 2
 *
 * Joel Troughton 2021
 * working at the KAUST Solar Center
 *
 * Credit to Phil Salmony (philsal.co.uk) for the FIR code
 */


/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "stdbool.h"
#include "FIRFilter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ADS1115_ADDRESS 0x48 // I2C addresses of ADC
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t UserRxBuffer[2048];
bool usbMessageReceived = false;

bool streaming_data_requested = false;

float amp_gain = 101;
float shunt_resistance = 100000;

// FIR filters for both voltage and current
FIRFilter lpfcurr;
FIRFilter lpfvolt;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Initialise the ADS1115 at boot. Sets the comparator to toggle the alert pin
// when a measurement is completed
void adc_init()
{
	uint8_t conversion_reg = 0x00;
	uint8_t config_reg = 0x01;
	uint8_t lothresh_reg = 0x02;
	uint8_t hithresh_reg = 0x03;

	unsigned char ADSwrite[6];

	ADSwrite[0] = hithresh_reg;
	ADSwrite[1] = 0x80;
	ADSwrite[2] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100);

	ADSwrite[0] = lothresh_reg;
	ADSwrite[1] = 0x7F;
	ADSwrite[2] = 0xFF;
	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100);
}

/* Make a single-ended ADC measurement. Fixed at 128sps, 4.096V FSR
 *
 * For ADC registers:
 * https://www.ti.com/lit/ds/symlink/ads1115.pdf
 */
uint16_t adc_read_singleended(uint8_t channel)
{
	uint8_t conversion_reg = 0x00;
	uint8_t config_reg = 0x01;
	uint8_t lothresh_reg = 0x02;
	uint8_t hithresh_reg = 0x03;

	unsigned char ADSwrite[6];
	int16_t reading;

	switch (channel)
	{
	case 0:
		ADSwrite[1] = 0xC3;
		break;
	case 1:
		ADSwrite[1] = 0xD3;
		break;
	case 2:
		ADSwrite[1] = 0xE3;
		break;
	case 3:
		ADSwrite[1] = 0xF3;
		break;
	}

	HAL_GPIO_WritePin(GPIOA, LED_R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LED_G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, LED_B_Pin, GPIO_PIN_SET);

	ADSwrite[0] = config_reg;
	ADSwrite[2] = 0b10001000; // For Alert pin, set COMP_QUE to anything but 11. Active low

	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 3, 100);

	// Wait for ADC_Alert pin to go high, indicating the measurement is complete
	while (HAL_GPIO_ReadPin(ADC_Alert_GPIO_Port, ADC_Alert_Pin) == 0)
	{
		HAL_GPIO_WritePin(GPIOA, LED_B_Pin, GPIO_PIN_RESET);
	}

	HAL_GPIO_WritePin(GPIOA, LED_B_Pin, GPIO_PIN_SET);

	ADSwrite[0] = conversion_reg;

	HAL_I2C_Master_Transmit(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 1, 100);

	HAL_I2C_Master_Receive(&hi2c1, ADS1115_ADDRESS << 1, ADSwrite, 2, 100);
	reading = (ADSwrite[0] << 8 | ADSwrite[1]);
	if (reading < 0) // Shouldn't be any -ve values
	{
		reading = 0;
	}
	return reading;
}

void single_shot()
{
	//uint16_t v_measure = adc_read_singleended(1);
	//uint16_t i_measure = adc_read_singleended(0);

	//float voltage = (v_measure * 0.125);
	//float current = (i_measure * 0.125) / amp_gain / 0.1;

	float voltage_running = 0;
	float current_running = 0;

	for (int i = 0; i < num_oversamples; i++)
	{
		uint16_t v_measure = adc_read_singleended(1);
		uint16_t i_measure = adc_read_singleended(0);

		/*		float voltage = (v_measure * 0.000625) - 8.190538;
		 float current = (i_measure * 0.125 / 1000) / shunt_resistance
		 / i_amp_gain;
		 current = current + ((3.44E-5 * voltage) - 7.35E-5);*/

		/*		float voltage = v_measure * 0.125;
		 voltage = voltage / 1000;
		 voltage = voltage * 3.9947;
		 voltage = voltage - 6.1312;*/

		float voltage = v_measure * 0.125;
		float current = (i_measure * 0.125) / amp_gain / 0.1;

		voltage_running += voltage;
		current_running += current;
	}

	voltage_running /= num_oversamples;
	current_running /= num_oversamples;

	char vmsg[32];
	char imsg[32];

	memset(vmsg, 0, sizeof(vmsg));
	memset(imsg, 0, sizeof(imsg));

	sprintf(vmsg, "%4.3f", voltage_running);
	sprintf(imsg, "%4.3f", current_running);

	strcat(vmsg, ",");
	strcat(vmsg, imsg);

	strcat(vmsg, "\n");
	CDC_Transmit_FS(vmsg, sizeof(vmsg));
}

/* When the measurement is first started, the output creeps up
 * until the buffer is full. This fills the buffer up before
 * outputting any messages over USB
 */
void fill_fir_buffer()
{
	uint8_t filter_length = FIRFilter_FilterLength(&lpfcurr);

	for (uint8_t n = 0; n <= filter_length; n++)
	{
		uint16_t v_measure = adc_read_singleended(1);
		float voltage = v_measure * 0.125;

		uint16_t i_measure = adc_read_singleended(0);
		float current = (i_measure * 0.125) / amp_gain / 0.1;

		FIRFilter_Update(&lpfvolt, voltage);
		FIRFilter_Update(&lpfcurr, current);
	}
}

// Start outputting data
void run_fir()
{
	uint16_t v_measure = adc_read_singleended(1);
	float voltage = v_measure * 0.125;

	uint16_t i_measure = adc_read_singleended(0);
	float current = (i_measure * 0.125) / amp_gain / 0.1;

	FIRFilter_Update(&lpfvolt, voltage);
	FIRFilter_Update(&lpfcurr, current);

	char vmsg[32];
	char imsg[32];

	memset(vmsg, 0, sizeof(vmsg));
	memset(imsg, 0, sizeof(imsg));

	sprintf(vmsg, "%4.3f", lpfvolt.out);
	sprintf(imsg, "%4.3f", lpfcurr.out);

	strcat(vmsg, ",");
	strcat(vmsg, imsg);

	strcat(vmsg, "\n");
	CDC_Transmit_FS(vmsg, sizeof(vmsg));

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
	MX_I2C1_Init();
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 2 */

	adc_init();
	FIRFilter_Init(&lpfcurr);
	FIRFilter_Init(&lpfvolt);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		HAL_GPIO_WritePin(GPIOA, LED_R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, LED_G_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, LED_B_Pin, GPIO_PIN_SET);

		if (streaming_data_requested)
		{
			run_fir();
		}

		if (usbMessageReceived == true)
		{

			char chRxBuffer[2048];

			// Take the uint8_t buffer and turn it into a char array to parse
			for (int i = 0; i <= 2048; i++)
			{
				chRxBuffer[i] = UserRxBuffer[i];
			}

			usbMessageReceived = false; // Wait untill we have read the incoming message

			char singleshotSearch[] = "single"; 	// Take a single shot
			char startSearch[] = "start"; 			// Begin continuous measurements
			char stopSearch[] = "stop";				// Stop continuous measurements

			char *singleshotPtr = strstr(chRxBuffer, singleshotSearch);
			char *startPtr = strstr(chRxBuffer, startSearch);
			char *stopPtr = strstr(chRxBuffer, stopSearch);

			if (singleshotPtr != NULL)
			{
				run_fir();
			}
			else if (startPtr != NULL)
			{
				streaming_data_requested = true;
				fill_fir_buffer();

			}
			else if (stopPtr != NULL) // Do a little stop light blink
			{
				streaming_data_requested = false;

				HAL_GPIO_WritePin(GPIOA, LED_R_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, LED_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED_B_Pin, GPIO_PIN_SET);

				HAL_Delay(100);

				HAL_GPIO_WritePin(GPIOA, LED_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED_B_Pin, GPIO_PIN_SET);

				HAL_Delay(50);

				HAL_GPIO_WritePin(GPIOA, LED_R_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, LED_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED_B_Pin, GPIO_PIN_SET);

				HAL_Delay(100);

				HAL_GPIO_WritePin(GPIOA, LED_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED_G_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED_B_Pin, GPIO_PIN_SET);

				HAL_Delay(100);

				HAL_GPIO_WritePin(GPIOA, LED_R_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOA, LED_G_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOA, LED_B_Pin, GPIO_PIN_SET);

			}
			else
			{
				char msg[1024] = "Input not understood";
				strcat(msg, "\n");
				CDC_Transmit_FS(msg, sizeof(msg));
			}
		}
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
	RCC_OscInitTypeDef RCC_OscInitStruct =
	{ 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct =
	{ 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInit =
	{ 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSI48;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_I2C1;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x2000090E;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct =
	{ 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LED_R_Pin | LED_G_Pin | LED_B_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LED_R_Pin LED_G_Pin LED_B_Pin */
	GPIO_InitStruct.Pin = LED_R_Pin | LED_G_Pin | LED_B_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : ADC_Alert_Pin */
	GPIO_InitStruct.Pin = ADC_Alert_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADC_Alert_GPIO_Port, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
