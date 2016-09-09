/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */

//635kl; 
#define 		RESET_OFF	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_RESET)   //RDAC power off
#define 		RESET_ON	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14, GPIO_PIN_SET) 		//RDAC power on
#define     SYNC_OFF	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET)  		//дозволити запис в потенціометри
#define     SYNC_ON 	 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET) 	 		//заборонити запис в потенціометри
#define 		WP_OFF		 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET) 		//вимкнути захист від запису
#define 		WP_ON			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET)   		//ввімкнути захист від запису
#define 		LED_ON 		 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11, GPIO_PIN_SET)  		//засвітити світлодіод
#define 		LED_OFF 	 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11, GPIO_PIN_RESET)   //потушити світлодіод
#define			BUTTON		 HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2)

//Хендлери преривань знаходяться в файлі it.c
//#include "pcd8544.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/
uint8_t uartData_RX[7]; 	 //сюди записуються посилки даних з УСАПП 
uint8_t R_EEPROM[8];   		 //сюди зчитуються дані з ЕЕПРОМ
uint8_t W_EEPROM[8];   		 //звідси дані записуються в ЕЕПРОМ
uint8_t i;						  	 //змінна загального призначення (для функції "for")
uint8_t ZAP=0;				 		 // флаг дозволу запису
uint8_t MOD=0; 					   //флаг що сповіщує про модифікацію налаштувань потенціометрів
uint8_t data_valid_flag=1; //по замовчуванню дані вірні, якщо хоч раз за цикл верифікації прийметься з помилкою то флаг зкинеться
uint32_t RES[4]={512,512,512,512}; //початкові значення для передачі на потенціометри (середина діапазону)
uint8_t DefVal[8]={00,02,00,02,00,02,00,02}; //Масив з 4 значеннями 512 (складено з двох байт молодшим вперед) 
uint8_t AD5292_data[2]; //для запису слова даних в потенціометри (ініціалізація) 
volatile uint8_t flag_DMA;  //флаг завершення прийому з УАПП по ДМА 7 байт
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void AD5292_Init(void);							//ввімкнення потенціометрів (RESET_ON та передача на кожен з них 0х1803)
void Write_to_EEPROM(void);  				//запис в еепром RES[4] у 8 ячейок памяті
void Read_from_EEPROM(void);				//читання з еепром RES[4] з 8 ячейок памяті
void SetToDefVal(void);   					//запис DefVal[8] в EEPROM, потім читання з нього та видача на потенціометри
uint8_t Receive_from_UART(void);		//запуск процесу читання даних з початку нової посилки по ДМА, функція поверта результат якщо є передача, а інакше 0хFF
uint8_t valid_data(uint8_t repeat); //функція звіряє дані задану кількість раз і повертає результат (значення якщо воно вірне, а інакше 0хFF )
void sort_data(uint8_t uart_data);  //визначення які кнопки натиснуто, зміна значень даних потенціометрів
void update_RES(void);							//перезапис значень в потенціометрах
void led_blink(void);								//засвітити світлодіод на 2 сек. для індикації завершення запису
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */
HAL_TIM_Base_Start_IT(&htim6); //Запуск таймера 6
__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);//дозвіл преривань по прийому USART
////////////////////////////////////////////////////////////////////////////////////////
	WP_ON; //ввімкнути захист від запису
AD5292_Init(); //ввімкнення потенціометрів

Read_from_EEPROM(); //завантаження збереженої конфігурації з ЕСПЗУ в ОЗУ
update_RES();//оновити значення в потенціометрах
////////////////////////////////////////////////////////////////////////////////////////
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
sort_data(valid_data(5)); 					//отримання інформації,верифікація даних, сортування, підготовка до видачі на потенціометри
update_RES();												//оновити значення в потенціометрах
Write_to_EEPROM();    							//запис поточних значень якщо є на те підстави (супроводжується світінням світлодіоду 2сек.)
if(BUTTON==RESET){SetToDefVal();}   //Встановити значення DefValue якщо натиснута кнопка зкидання (супроводжується світінням світлодіоду 2сек.).
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 10000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 40;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 62500;
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

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STAT_Pin|RES_P_Pin|WP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLEAR_Pin */
  GPIO_InitStruct.Pin = CLEAR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CLEAR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STAT_Pin RES_P_Pin WP_Pin */
  GPIO_InitStruct.Pin = STAT_Pin|RES_P_Pin|WP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Read_from_EEPROM(void)
{
				//	for(i=0;i<8;i++) {R_EEPROM[i]=0;} //очистка в озу попередніх зчитаних з еепром значень
						//									DevAddress MemAddress MemAddSize	data 			Size timeout  )
				HAL_I2C_Mem_Read(&hi2c1, 0xA0, 			0, 					1,			 &R_EEPROM[0], 8,1000); //зчитування 8 байт даних з еепром
				HAL_Delay(10);
						
				RES[0]=R_EEPROM[0]+(R_EEPROM[1]<<8); //сортування зчитаних даних для передачі на потенціометри
				RES[1]=R_EEPROM[2]+(R_EEPROM[3]<<8);
				RES[2]=R_EEPROM[4]+(R_EEPROM[5]<<8);
				RES[3]=R_EEPROM[6]+(R_EEPROM[7]<<8);
}

void Write_to_EEPROM(void)
{
						
							if(ZAP==1) // якщо прийшла команда запису 
							{
									WP_OFF; //вимкнути захист від запису
		//копіювання значень потенціометрів в масив для запису в ЕСПЗУ 
									//Резистор 0	
													W_EEPROM[0]=RES[0]&0xFF;     //Lo byte
													W_EEPROM[1]=RES[0]>>8;       //Hi byte
									//Резистор 2	
													W_EEPROM[2]=RES[1]&0xFF;     //Lo byte
													W_EEPROM[3]=RES[1]>>8;       //Hi byte
									//Резистор 2	
													W_EEPROM[4]=RES[2]&0xFF;     //Lo byte
													W_EEPROM[5]=RES[2]>>8;       //Hi byte
									//Резистор 3	
													W_EEPROM[6]=RES[3]&0xFF;     //Lo byte
													W_EEPROM[7]=RES[3]>>8;       //Hi byte
		//Початок запису в ЕСПЗУ 8 байт з нульової адреси
										//												DevAddress MemAddress MemAddSize	data 					Size  Timeout)
										HAL_I2C_Mem_Write(&hi2c1, 0xA0, 			0, 					1,			 &W_EEPROM[0],  8,1000 );
										HAL_Delay(10); 
								    WP_ON; //ввімкнути захист від запису
										led_blink();				
												
										ZAP=0;              //зкидання флагів
										MOD=0;
							}
}
void SetToDefVal(void)
{
							Read_from_EEPROM(); //зчитали поточні налаштування, що записані в пам'ять
							if (RES[0]!=512|RES[1]!=512|RES[2]!=512|RES[3]!=512)  //якщо хоч один з потенціометрів не в середньому значенні то...
														{
															WP_OFF; //вимкнути захист від запису
															//												DevAddress MemAddress MemAddSize	data 		Size  Timeout)
															HAL_I2C_Mem_Write(&hi2c1, 0xA0, 			0, 					1,			 &DefVal[0],  8,1000 );
															HAL_Delay(10);
															WP_ON; //ввімкнути захист від запису
															led_blink();
															Read_from_EEPROM(); //зчитування з еепром в озу останньоЇ записаної конфігурації
														}
}
				
uint8_t valid_data(uint8_t repeat) //функція звіряє дані задану кількість раз і повертає значення, якщо вони вірні
{
	uint8_t first_value;
	uint8_t stage_valid_flag=0; //флаг ідентичності окремих послідовних посилок
	uint8_t out_data=0xFF; ////по замовчуванню функція повертає 0xFF (тобто не в режимі настройки потенціометрів)
	data_valid_flag=1; //по замовчуванню дані вірні, спробуємо опровергнути це
	
	first_value=Receive_from_UART(); //отримуємо перше значення
				for(i=0;i<repeat;i++) 
					{
						if (Receive_from_UART()==first_value) {stage_valid_flag=1;} else {stage_valid_flag=0;}		//звіряємо з першим
						data_valid_flag=data_valid_flag&stage_valid_flag; //якщо хоч раз не співпаде отримане значення з першим, то дані не вірні.
					}
	if(data_valid_flag==1) {out_data=first_value;} //якщо все співпало, то функція видасть зчитане значення, а інакще по замовчуванню присвоєно 0хFF
	return out_data;
}
				
uint8_t Receive_from_UART(void)
{
					uint8_t out_data=0xFF; //по замовчуванню функція повертає 0xFF (тобто не в режимі настройки потенціометрів)
					while(TIM6->CNT<10){} //очікуємо поки закінчиться стара посилка (висимо тут якщо дані йдуть і таймер систематично перезапускається)
							if(TIM6->CNT>10) //якщо таймер зміг дорахувати до 10мс, значить скоро нова посилка, бо був перерив в прийомі по УСАПП
									{
										uartData_RX[5]=0xFF; //очистка прийомного буфера
										HAL_UART_Receive_DMA(&huart1,uartData_RX,6); //запуск прийому перших 6 байт даних
									  flag_DMA=1;
									  while((flag_DMA==1)&(TIM6->CNT<200)){	}//очікування завершення прийому даних або таймаут 200мс
										if (RES[0]!=512|RES[1]!=512|RES[2]!=512|RES[3]!=512){ HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_11);}  //STATUS LED toggle;
										out_data=uartData_RX[5]&0x0F; //маскування 6 байта, і видача результату 				
									}
							return out_data;	
}
void sort_data(uint8_t uart_data)
{	
							//бітові операції
							// установлений MOD значить що були внесені зміни до налаштувань, які треба записати.
							//Значення регулюються від 0 до 1024, програмно обмежена перестройка
					if(uart_data==1   & RES[0]<1023) {RES[0]++;MOD=1;}  //(Наблюдение)Горизонталь + 
					if(uart_data==2   & RES[0]>0)		 {RES[0]--;MOD=1;}	//(Наблюдение)Горизонталь -
					if(uart_data==3   & RES[1]<1023) {RES[1]++;MOD=1;}  //(Наблюдение)Вертикаль +
					if(uart_data==4   & RES[1]>0)    {RES[1]--;MOD=1;}  //(Наблюдение)Вертикаль -
					if(uart_data==5   & RES[2]<1023) {RES[2]++;MOD=1;}  //(Дубль)Горизонталь+
					if(uart_data==6   & RES[2]>0)    {RES[2]--;MOD=1;}	//(Дубль)Горизонталь-
					if(uart_data==7   & RES[3]<1023) {RES[3]++;MOD=1;}  //(Дубль)Вертикаль +
					if(uart_data==8   & RES[3]>0)    {RES[3]--;MOD=1;}	//(Дубль)Вертикаль -

					if((uartData_RX[5]==0x0F)&(MOD==1) )    {ZAP=1;}	//Якщо Отримали 0x0F і внесені зміни в настройки то дозволяємо запис в ЕСПЗУ
}
									
void AD5292_Init(void) //ініціалізація 4x потенціометрів
{
					RESET_OFF; //RDAC power oFF
					HAL_Delay(100);
					RESET_ON; //RDAC power on
					HAL_Delay(100);
	
					SYNC_OFF;
					AD5292_data[0]=0x03; //LO byte //Ввімкнути можливість зміни опору. 
					AD5292_data[1]=0x18;	//HI byte
					for(i=0;i<4;i++)
						{	
							HAL_SPI_Transmit(&hspi1,&AD5292_data[0], 1, 1);
						}
					SYNC_ON;
					//HAL_Delay(100);
}
void update_RES(void)  //оновлення значень в 4х потенціометрах
{
					  SYNC_OFF; //почати передачу даних
						for(i=0;i<4;i++)
						{
							AD5292_data[0]=(RES[i] & 0xFF); //LO byte  									//формування слова для запису в потенціометри
							AD5292_data[1]=((RES[i]>>8)+4);	//HI byte
							HAL_SPI_Transmit(&hspi1,&AD5292_data[0], 1, 1);							//передача слова даних
						}
				  	SYNC_ON;	//завершити передачу даних
}
void led_blink(void)
{
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11, GPIO_PIN_SET);  //STATUS LED ON;//Індикація операції запису в память
		HAL_Delay(2000);
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11, GPIO_PIN_RESET);  //STATUS LED OFF;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
