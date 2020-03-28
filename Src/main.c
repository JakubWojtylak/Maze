/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "crc.h"
#include "dma2d.h"
#include "gfxsimulator.h"
#include "i2c.h"
#include "ltdc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_lcd.h"
#include "../Drivers/BSP/STM32F429I-Discovery/stm32f429i_discovery_gyroscope.h"
#include "../Drivers/BSP/STM32F429I-Discovery/Fonts/fonts.h"
#include "Hehe.h"
#include "Menu.h"
#include "Menu_kontynuuj.h"
#include "Menu_nowagra.h"
#include "Menu_poziomy.h"
#include "mapa1.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile ZyroskopDane DataOld, DataNow;
volatile int32_t AngleX, AngleY, AngleZ;

//Dane dla czesci testowej*****************
volatile ZyroskopDane DataTempRom[4];
volatile ZyroskopDane DataTempRich;

volatile int32_t CalkaTrapX, CalkaTrapY;
volatile int32_t CalkaRichaX, CalkaRichaY;
volatile int32_t CalkaRombX, CalkaRombY;
volatile int32_t CalkaPosrednia1X, CalkaPosrednia1Y;
volatile int32_t CalkaPosrednia2X, CalkaPosrednia2Y;
volatile int32_t CalkaPosredniaRomX[4];
volatile int32_t CalkaPosredniaRomY[4];
volatile int32_t CalkaPomocniczaRomX[3];
volatile int32_t CalkaPomocniczaRomY[3];
volatile int32_t CalkaPomocnicza2RomX[2];
volatile int32_t CalkaPomocnicza2RomY[2];

volatile uint8_t LicznikPomocniczy;
volatile uint8_t LicznikPomocniczyRomberg;
//*****************************************

//Zmienne stanu gry************************
volatile eStanGry StanGry;
volatile eStanMenu StanMenu;
volatile uint8_t ZmienionoStanMenu;
volatile uint8_t ZmienionoStanPoziomow;
volatile uint8_t WybranyPoziom;
volatile uint32_t czasZmiany;
//*****************************************

//Zmiana stanu przycisku*******************
GPIO_PinState StanPrzycisku;
GPIO_PinState PoprzedniStanPrzycisku = GPIO_PIN_RESET;
uint32_t PoprzedniCzasPrzycisku;
//*****************************************

float dT;
uint8_t Animacja;
volatile uint16_t X, Y;
volatile uint8_t Direction;
volatile uint8_t fMovedX;
volatile uint8_t fMovedY;
volatile uint16_t ResetTimeX;
volatile uint16_t ResetTimeY;

DMA2D_HandleTypeDef hdma2d;
I2C_HandleTypeDef hi2c3;
LTDC_HandleTypeDef hltdc;
SPI_HandleTypeDef hspi5;
SDRAM_HandleTypeDef hsdram1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t spi5_sendrecv(uint8_t byte) {
	uint8_t answer;

	HAL_SPI_TransmitReceive(&hspi5, &byte, &answer, 1, HAL_MAX_DELAY);

	return answer;
}

uint8_t SPI5_read(uint8_t address) {
	uint8_t dane;
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	spi5_sendrecv(address | 0x80);
	dane = spi5_sendrecv(0xFF);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

	return dane;
}

void SPI5_write(uint8_t address, uint8_t data) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
	spi5_sendrecv(address);
	data = spi5_sendrecv(data);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);

}

uint8_t OurL3GD20_Init() {
	if (SPI5_read(0x0F) != 0b11010100) {

		return 1;
	}

	//Enable L3GD20 Power bit
	SPI5_write(0x20, 0xFF);

	//Set L3GD20 scale

	SPI5_write(0x23, 0x00);

	//Set high-pass filter settings
	SPI5_write(0x21, 0x00);

	//Enable high-pass filter
	SPI5_write(0x24, 0x10);

	//Everything OK
	return 0;
}

void OurL3GD20_Read() {
	float s;
	short temp1, temp2, temp3;

	// Read X axis

	temp1 = (SPI5_read(0x28) | SPI5_read(0x29) << 8);
	temp2 = (SPI5_read(0x2A) | SPI5_read(0x2B) << 8);
	temp3 = (SPI5_read(0x2C) | SPI5_read(0x2D) << 8);

	// Sensitivity at 250 range = 8.75 mdps/digit
	s = 8.75 * 0.001;

	DataNow.OsX = (short) ((float) temp1 * s);
	DataNow.OsY = (short) ((float) temp2 * s);
	DataNow.OsZ = (short) ((float) temp3 * s);

}

void send_char(char c) {
	HAL_UART_Transmit(&huart1, (uint8_t*) &c, 1, 1000);
}

int __io_putchar(int ch) {
	send_char(ch);
	return ch;
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_CRC_Init();
	MX_DMA2D_Init();
	MX_FMC_Init();
	MX_LTDC_Init();
	MX_SPI5_Init();
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	MX_I2C3_Init();
	MX_TIM10_Init();
	MX_TIM11_Init();
	MX_GFXSIMULATOR_Init();
	/* USER CODE BEGIN 2 */

	__HAL_SPI_ENABLE(&hspi5);

	//Do testu*************
	LicznikPomocniczy = 0;
	LicznikPomocniczyRomberg = 0;
	//*********************

	//Poczatkowy stan gry**
	StanGry = Menu;
	StanMenu = NowaGra;
	ZmienionoStanMenu = 1;
	ZmienionoStanPoziomow = 1;
	WybranyPoziom = 0;
	//*********************

	Animacja = 0;
	Direction = 1;
	X = 120;
	Y = 170;
	dT = 0.001;
	fMovedX = 0;
	fMovedY = 0;
	ResetTimeX = 0;
	ResetTimeY = 0;

	BSP_LCD_Init();
	//BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER+1024*1024*4);
	BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);

	BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);

	BSP_LCD_DisplayOn();
	BSP_LCD_Clear(LCD_COLOR_BLACK);

	//Zakomentowac po uruchomieniu menu
	//BSP_LCD_DrawBitmap(0, 0, (uint8_t*) image_data_Menu);
//	HAL_Delay(2000);
//	BSP_LCD_DrawBitmap(0, 0, (uint8_t*) image_data_Menu_nowagra);
//	HAL_Delay(2000);
//	BSP_LCD_DrawBitmap(0, 0, (uint8_t*) image_data_Menu_kontynuuj);
//	HAL_Delay(2000);
//	BSP_LCD_DrawBitmap(0, 0, (uint8_t*) image_data_Menu_poziomy);
//	//*********************************

	//BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
	//BSP_LCD_DisplayStringAtLine(5, (uint8_t*) "Hello");

//	HAL_Delay(2000);
	BSP_LCD_ClearStringLine(5);

	BSP_LCD_Clear(LCD_COLOR_WHITE);

//	//Zakomentowac po uruchomieniu menu
//	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
//	BSP_LCD_FillRect(0, 0, 240, 15);
//	BSP_LCD_FillRect(0, 305, 240, 15);
//	BSP_LCD_FillRect(0, 15, 15, 290);
//	BSP_LCD_FillRect(225, 15, 15, 290);
	//*********************************

	OurL3GD20_Init();
czasZmiany=0;
	//HAL_TIM_Base_Start_IT(&htim10);
	HAL_TIM_Base_Start_IT(&htim11);


	GPIO_PinState OdczytanyStanPrzycisku;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

if(HAL_GetTick()-czasZmiany > 2000){
		if (StanGry == Menu) {

				StanMenu = (StanMenu + 1) % 3;
				ZmienionoStanMenu = 1;

			} else if (StanGry == WyborPoziomu) {
				WybranyPoziom = (WybranyPoziom + 1) % 3;
				ZmienionoStanPoziomow = 1;
			}

		czasZmiany = HAL_GetTick();
}
		//Sprawdzanie stanu przycisku*****************
		OdczytanyStanPrzycisku = HAL_GPIO_ReadPin(BB_GPIO_Port, BB_Pin);

		if (OdczytanyStanPrzycisku != PoprzedniStanPrzycisku) {
			PoprzedniCzasPrzycisku = HAL_GetTick();
		}

		if ((HAL_GetTick() - PoprzedniCzasPrzycisku) > 10) {
			if (OdczytanyStanPrzycisku != StanPrzycisku) {
				StanPrzycisku = OdczytanyStanPrzycisku;

				//	if (StanPrzycisku == GPIO_PIN_RESET) { //StanGry = Gra;
//					if (StanGry == Menu) {
//						StanMenu = (StanMenu + 1) % 3;
//						ZmienionoStanMenu = 1;
//
//					} else if (StanGry == WyborPoziomu) {
//						WybranyPoziom = (WybranyPoziom + 1) % 3;
//						ZmienionoStanPoziomow = 1;
//					}
				//}

				//Przetestowac !!!!!!!!!!!!!!!!!
				//Jezeli przytrzymamy przycisk dluzej nastapi przejscie miedzy gra, a menu
				if (StanPrzycisku == GPIO_PIN_SET) {

					if (StanGry == Menu && StanMenu == NowaGra) {
						StanGry = Gra;

					} else if (StanGry == Menu && StanMenu == ZmienPoziom) {
						StanGry = WyborPoziomu;
						WybranyPoziom = 0;
						ZmienionoStanPoziomow = 1;


					} else if (StanGry == WyborPoziomu) {
						StanGry = Menu;


					} else if (StanGry == Gra) {
						StanGry = Menu;
					}

					PoprzedniCzasPrzycisku = HAL_GetTick();

				}
			}
		}

		PoprzedniStanPrzycisku = OdczytanyStanPrzycisku;

		//Koniec sprawdzania stanu przycisku**********

		//Petla gry***********************************

		if (StanGry == Menu) {
			if (ZmienionoStanMenu == 1) {
				switch (StanMenu) {
				case NowaGra:
					BSP_LCD_DrawBitmap(0, 0,
							(uint8_t*) image_data_Menu_nowagra);
					break;

				case KontynuujGre:
					BSP_LCD_DrawBitmap(0, 0,
							(uint8_t*) image_data_Menu_kontynuuj);
					break;

				case ZmienPoziom:

					BSP_LCD_DrawBitmap(0, 0,
							(uint8_t*) image_data_Menu_poziomy);
					break;
				}

				ZmienionoStanMenu = 0;
			}

		} else if (StanGry == WyborPoziomu) {
			if (ZmienionoStanPoziomow == 1) {
				switch (WybranyPoziom) {
				case 0:
					BSP_LCD_Clear(LCD_COLOR_WHITE);
					BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
					BSP_LCD_FillRect(20, 20, 60, 60);
					BSP_LCD_DisplayChar(30, 30, 49);
					BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

					BSP_LCD_FillRect(90, 20, 60, 60);
					BSP_LCD_DisplayChar(100, 30, 50);

					BSP_LCD_FillRect(160, 20, 60, 60);
					BSP_LCD_DisplayChar(170, 30, 51);
					break;

				case 1:
					BSP_LCD_Clear(LCD_COLOR_WHITE);
					BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
					BSP_LCD_FillRect(20, 20, 60, 60);
					BSP_LCD_DisplayChar(30, 30, 49);

					BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
					BSP_LCD_FillRect(90, 20, 60, 60);
					BSP_LCD_DisplayChar(100, 30, 50);

					BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
					BSP_LCD_FillRect(160, 20, 60, 60);
					BSP_LCD_DisplayChar(170, 30, 51);
					break;

				case 2:
					BSP_LCD_Clear(LCD_COLOR_WHITE);
					BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
					BSP_LCD_FillRect(20, 20, 60, 60);
					BSP_LCD_DisplayChar(30, 30, 49);

					BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
					BSP_LCD_FillRect(90, 20, 60, 60);
					BSP_LCD_DisplayChar(100, 30, 50);

					BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
					BSP_LCD_FillRect(160, 20, 60, 60);
					BSP_LCD_DisplayChar(170, 30, 51);
					break;

				default:
					break;
				}

				ZmienionoStanPoziomow = 0;
			}
		} else if (StanGry == Gra) {
			BSP_LCD_DrawBitmap(0, 0, (uint8_t*) image_data_mapa1);

		}

		//Koniec petli gry****************************

		printf("Angle X: %li\n\r", AngleX);
		printf("Angle Y: %li\n\r", AngleY);
		printf("Angle Z: %li\n\r", AngleZ);
		printf("CzasX: %d\n\r", ResetTimeX);
		printf("CzasY: %d\n\r", ResetTimeY);
		printf("Predkosc X: %d\n\r", DataNow.OsX);
		printf("Predkosc Y: %d\n\r", DataNow.OsY);

		//Do testu************************************
		printf("CalkaTrap X: %li\n\r", CalkaTrapX);
		printf("CalkaTrap Y: %li\n\r", CalkaTrapY);

		printf("CalkaRicha X: %li\n\r", CalkaRichaX);
		printf("CalkaRicha Y: %li\n\r", CalkaRichaY);

		printf("CalkaRomb X: %li\n\r", CalkaRombX);
		printf("CalkaRomb Y: %li\n\r", CalkaRombY);
		//********************************************


		HAL_Delay(200);
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM10) //Przerwanie pochodzi od timera 10
	{
		//Zamienic na if(StanGry == Gra) przy uruchomieniu menu
		if (StanGry == Gra) {
			if ((Y < 300) && (Direction == 1)) {

				BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);

				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);

				if (X <= 25 && Y <= 25)
					BSP_LCD_FillRect(X - 10, Y - 10, 40, 40);
				else if (X >= 215 && Y >= 295)
					BSP_LCD_FillRect(X - 20, Y - 20, 30, 30);
				else if (X <= 25 && Y >= 295)
					BSP_LCD_FillRect(X - 10, Y - 20, 40, 30);
				else if (X >= 215 && Y <= 25)
					BSP_LCD_FillRect(X - 20, Y - 10, 30, 40);
				else if (X <= 25)
					BSP_LCD_FillRect(X - 10, Y - 20, 40, 40);
				else if (X >= 215)
					BSP_LCD_FillRect(X - 20, Y - 20, 30, 40);
				else if (Y >= 295)
					BSP_LCD_FillRect(X - 20, Y - 20, 40, 30);
				else if (Y <= 25)
					BSP_LCD_FillRect(X - 20, Y - 10, 40, 40);
				else
					BSP_LCD_FillRect(X - 20, Y - 20, 40, 40);

				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

				//Y += 10;
				BSP_LCD_FillCircle(X, Y, 10);
				BSP_LCD_FillRect(0, 0, 240, 15);
				BSP_LCD_FillRect(0, 305, 240, 15);
				BSP_LCD_FillRect(0, 15, 15, 290);
				BSP_LCD_FillRect(225, 15, 15, 290);

			} else if ((Y >= 300) && (Direction == 1)) {
				Direction = 0;

				BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);

				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				if (X <= 25 && Y <= 25)
					BSP_LCD_FillRect(X - 10, Y - 10, 40, 40);
				else if (X >= 215 && Y >= 295)
					BSP_LCD_FillRect(X - 20, Y - 20, 30, 30);
				else if (X <= 25 && Y >= 295)
					BSP_LCD_FillRect(X - 10, Y - 20, 40, 30);
				else if (X >= 215 && Y <= 25)
					BSP_LCD_FillRect(X - 20, Y - 10, 30, 40);
				else if (X <= 25)
					BSP_LCD_FillRect(X - 10, Y - 20, 40, 40);
				else if (X >= 215)
					BSP_LCD_FillRect(X - 20, Y - 20, 30, 40);
				else if (Y >= 295)
					BSP_LCD_FillRect(X - 20, Y - 20, 40, 30);
				else if (Y <= 25)
					BSP_LCD_FillRect(X - 20, Y - 10, 40, 40);
				else
					BSP_LCD_FillRect(X - 20, Y - 20, 40, 40);

				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

				//Y -= 10;
				BSP_LCD_FillCircle(X, Y, 10);
				BSP_LCD_FillRect(0, 0, 240, 15);
				BSP_LCD_FillRect(0, 305, 240, 15);
				BSP_LCD_FillRect(0, 15, 15, 290);
				BSP_LCD_FillRect(225, 15, 15, 290);

			} else if ((Y > 20) && (Direction == 0)) {

				BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);

				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				if (X <= 25 && Y <= 25)
					BSP_LCD_FillRect(X - 10, Y - 10, 40, 40);
				else if (X >= 215 && Y >= 295)
					BSP_LCD_FillRect(X - 20, Y - 20, 30, 30);
				else if (X <= 25 && Y >= 295)
					BSP_LCD_FillRect(X - 10, Y - 20, 40, 30);
				else if (X >= 215 && Y <= 25)
					BSP_LCD_FillRect(X - 20, Y - 10, 30, 40);
				else if (X <= 25)
					BSP_LCD_FillRect(X - 10, Y - 20, 40, 40);
				else if (X >= 215)
					BSP_LCD_FillRect(X - 20, Y - 20, 30, 40);
				else if (Y >= 295)
					BSP_LCD_FillRect(X - 20, Y - 20, 40, 30);
				else if (Y <= 25)
					BSP_LCD_FillRect(X - 20, Y - 10, 40, 40);
				else
					BSP_LCD_FillRect(X - 20, Y - 20, 40, 40);

				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

				//Y -= 10;
				BSP_LCD_FillCircle(X, Y, 10);
				BSP_LCD_FillRect(0, 0, 240, 15);
				BSP_LCD_FillRect(0, 305, 240, 15);
				BSP_LCD_FillRect(0, 15, 15, 290);
				BSP_LCD_FillRect(225, 15, 15, 290);

			} else if ((Y <= 20) && (Direction == 0)) {
				Direction = 1;

				BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);

				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				if (X <= 25 && Y <= 25)
					BSP_LCD_FillRect(X - 10, Y - 10, 40, 40);
				else if (X >= 215 && Y >= 295)
					BSP_LCD_FillRect(X - 20, Y - 20, 30, 30);
				else if (X <= 25 && Y >= 295)
					BSP_LCD_FillRect(X - 10, Y - 20, 40, 30);
				else if (X >= 215 && Y <= 25)
					BSP_LCD_FillRect(X - 20, Y - 10, 30, 40);
				else if (X <= 25)
					BSP_LCD_FillRect(X - 10, Y - 20, 40, 40);
				else if (X >= 215)
					BSP_LCD_FillRect(X - 20, Y - 20, 30, 40);
				else if (Y >= 295)
					BSP_LCD_FillRect(X - 20, Y - 20, 40, 30);
				else if (Y <= 25)
					BSP_LCD_FillRect(X - 20, Y - 10, 40, 40);
				else
					BSP_LCD_FillRect(X - 20, Y - 20, 40, 40);

				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);

				//Y += 10;
				BSP_LCD_FillCircle(X, Y, 10);
				BSP_LCD_FillRect(0, 0, 240, 15);
				BSP_LCD_FillRect(0, 305, 240, 15);
				BSP_LCD_FillRect(0, 15, 15, 290);
				BSP_LCD_FillRect(225, 15, 15, 290);
			}

		}
	}

	if (htim->Instance == TIM11) {
		OurL3GD20_Read();

		//To mozna wsumie przy testowaniu wywalic za czesc testowa
		if ((DataNow.OsX >= 25 && DataNow.OsY <= 20)
				|| (DataNow.OsX <= -25 && DataNow.OsY >= -20))
			AngleX +=
					(long) (DataNow.OsX + ((DataOld.OsX - DataNow.OsX) * 0.5));
		else if ((DataNow.OsX >= 25 && DataNow.OsY >= 25)
				|| (DataNow.OsX <= -25 && DataNow.OsY <= -25))
			AngleX +=
					(long) (DataNow.OsX + ((DataOld.OsX - DataNow.OsX) * 0.5));

		if ((DataNow.OsY >= 25 && DataNow.OsX <= 20)
				|| (DataNow.OsY <= -25 && DataNow.OsX >= -20))
			AngleY +=
					(long) (DataNow.OsY + ((DataOld.OsY - DataNow.OsY) * 0.5));
		else if ((DataNow.OsX >= 25 && DataNow.OsY >= 25)
				|| (DataNow.OsX <= -25 && DataNow.OsY <= -25))
			AngleY +=
					(long) (DataNow.OsY + ((DataOld.OsY - DataNow.OsY) * 0.5));

//*************TESTOWA CZESC*****************************************************************************************

		//Liczenie calki metoda wietu trapezow
		CalkaTrapX += (long) (4 * ((DataOld.OsX + DataNow.OsX) * 0.5)); //Czas miedzy kolejnymi pomiarami rowny 0.004 s, pomnozony przez 1000 zeby nie miec liczby z przecinkiem
		CalkaTrapY += (long) (4 * ((DataOld.OsY + DataNow.OsY) * 0.5));
		//koniec

		//Liczenie calki za pomoca ekstrapolacji Richardsona
		if (LicznikPomocniczy == 0) {
			CalkaPosrednia2X = (long) (4 * ((DataOld.OsX + DataNow.OsX) * 0.5));
			CalkaPosrednia2Y = (long) (4 * ((DataOld.OsY + DataNow.OsY) * 0.5));

			DataTempRich = DataOld;

			LicznikPomocniczy = 1;

		} else {
			CalkaPosrednia2X +=
					(long) (4 * ((DataOld.OsX + DataNow.OsX) * 0.5));
			CalkaPosrednia2Y +=
					(long) (4 * ((DataOld.OsY + DataNow.OsY) * 0.5));

			CalkaPosrednia1X = (long) (8
					* ((DataTempRich.OsX + DataNow.OsX) * 0.5));
			CalkaPosrednia1Y = (long) (8
					* ((DataTempRich.OsY + DataNow.OsY) * 0.5));

			CalkaRichaX += (long) (CalkaPosrednia2X)
					+ ((CalkaPosrednia2X - CalkaPosrednia1X) / 3);
			CalkaRichaY += (long) (CalkaPosrednia2Y)
					+ ((CalkaPosrednia2Y - CalkaPosrednia1Y) / 3);
			LicznikPomocniczy = 0;
		}

		//koniec

		//Liczenie calki za pomoca metody Romberga
		if (LicznikPomocniczyRomberg == 0) {
			DataTempRom[0] = DataOld;

			CalkaPosredniaRomX[3] = (long) (4
					* ((DataOld.OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[3] = (long) (4
					* ((DataOld.OsY + DataNow.OsY) * 0.5));

			LicznikPomocniczyRomberg = 1;

		} else if (LicznikPomocniczyRomberg == 1) {
			CalkaPosredniaRomX[3] += (long) (4
					* ((DataOld.OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[3] += (long) (4
					* ((DataOld.OsY + DataNow.OsY) * 0.5));

			CalkaPosredniaRomX[2] = (long) (8
					* ((DataTempRom[0].OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[2] = (long) (8
					* ((DataTempRom[0].OsY + DataNow.OsY) * 0.5));

			LicznikPomocniczyRomberg = 2;

		} else if (LicznikPomocniczyRomberg == 2) {
			DataTempRom[2] = DataOld;

			CalkaPosredniaRomX[3] += (long) (4
					* ((DataOld.OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[3] += (long) (4
					* ((DataOld.OsY + DataNow.OsY) * 0.5));

			LicznikPomocniczyRomberg = 3;

		} else if (LicznikPomocniczyRomberg == 3) {
			CalkaPosredniaRomX[3] += (long) (4
					* ((DataOld.OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[3] += (long) (4
					* ((DataOld.OsY + DataNow.OsY) * 0.5));

			CalkaPosredniaRomX[2] += (long) (8
					* ((DataTempRom[2].OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[2] += (long) (8
					* ((DataTempRom[2].OsY + DataNow.OsY) * 0.5));

			CalkaPosredniaRomX[1] = (long) (16
					* ((DataTempRom[0].OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[1] = (long) (16
					* ((DataTempRom[0].OsY + DataNow.OsY) * 0.5));

			LicznikPomocniczyRomberg = 4;

		} else if (LicznikPomocniczyRomberg == 4) {
			DataTempRom[1] = DataOld;

			CalkaPosredniaRomX[3] += (long) (4
					* ((DataOld.OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[3] += (long) (4
					* ((DataOld.OsY + DataNow.OsY) * 0.5));

			LicznikPomocniczyRomberg = 5;

		} else if (LicznikPomocniczyRomberg == 5) {
			CalkaPosredniaRomX[3] += (long) (4
					* ((DataOld.OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[3] += (long) (4
					* ((DataOld.OsY + DataNow.OsY) * 0.5));

			CalkaPosredniaRomX[2] += (long) (8
					* ((DataTempRom[1].OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[2] += (long) (8
					* ((DataTempRom[1].OsY + DataNow.OsY) * 0.5));

			LicznikPomocniczyRomberg = 6;

		} else if (LicznikPomocniczyRomberg == 6) {
			DataTempRom[3] = DataOld;

			CalkaPosredniaRomX[3] += (long) (4
					* ((DataOld.OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[3] += (long) (4
					* ((DataOld.OsY + DataNow.OsY) * 0.5));

			LicznikPomocniczyRomberg = 7;

		} else if (LicznikPomocniczyRomberg == 7) {
			CalkaPosredniaRomX[3] += (long) (4
					* ((DataOld.OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[3] += (long) (4
					* ((DataOld.OsY + DataNow.OsY) * 0.5));

			CalkaPosredniaRomX[2] += (long) (8
					* ((DataTempRom[3].OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[2] += (long) (8
					* ((DataTempRom[3].OsY + DataNow.OsY) * 0.5));

			CalkaPosredniaRomX[1] += (long) (16
					* ((DataTempRom[1].OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[1] += (long) (16
					* ((DataTempRom[1].OsY + DataNow.OsY) * 0.5));

			CalkaPosredniaRomX[0] = (long) (32
					* ((DataTempRom[0].OsX + DataNow.OsX) * 0.5));
			CalkaPosredniaRomY[0] = (long) (32
					* ((DataTempRom[0].OsY + DataNow.OsY) * 0.5));

			CalkaPomocniczaRomX[0] = (long) (CalkaPosredniaRomX[1]
					+ ((CalkaPosredniaRomX[1] - CalkaPosredniaRomX[0]) / 3));
			CalkaPomocniczaRomX[1] = (long) (CalkaPosredniaRomX[2]
					+ ((CalkaPosredniaRomX[2] - CalkaPosredniaRomX[1]) / 3));
			CalkaPomocniczaRomX[2] = (long) (CalkaPosredniaRomX[3]
					+ ((CalkaPosredniaRomX[3] - CalkaPosredniaRomX[2]) / 3));

			CalkaPomocniczaRomY[0] = (long) (CalkaPosredniaRomY[1]
					+ ((CalkaPosredniaRomY[1] - CalkaPosredniaRomY[0]) / 3));
			CalkaPomocniczaRomY[1] = (long) (CalkaPosredniaRomY[2]
					+ ((CalkaPosredniaRomY[2] - CalkaPosredniaRomY[1]) / 3));
			CalkaPomocniczaRomY[2] = (long) (CalkaPosredniaRomY[3]
					+ ((CalkaPosredniaRomY[3] - CalkaPosredniaRomY[2]) / 3));

			CalkaPomocnicza2RomX[0] = (long) (CalkaPomocniczaRomX[1]
					+ ((CalkaPomocniczaRomX[1] - CalkaPomocniczaRomX[0]) / 15));
			CalkaPomocnicza2RomX[1] = (long) (CalkaPomocniczaRomX[2]
					+ ((CalkaPomocniczaRomX[2] - CalkaPomocniczaRomX[1]) / 15));

			CalkaPomocnicza2RomY[0] = (long) (CalkaPomocniczaRomY[1]
					+ ((CalkaPomocniczaRomY[1] - CalkaPomocniczaRomY[0]) / 15));
			CalkaPomocnicza2RomY[1] = (long) (CalkaPomocniczaRomY[2]
					+ ((CalkaPomocniczaRomY[2] - CalkaPomocniczaRomY[1]) / 15));

			CalkaRombX =
					(long) (CalkaPomocnicza2RomX[1]
							+ ((CalkaPomocnicza2RomX[1]
									- CalkaPomocnicza2RomX[0]) / 63));
			CalkaRombY =
					(long) (CalkaPomocnicza2RomY[1]
							+ ((CalkaPomocnicza2RomY[1]
									- CalkaPomocnicza2RomY[0]) / 63));

			LicznikPomocniczyRomberg = 0;
		}
		//koniec

//*************KONIEC TESTOWEJ CZESCI*****************************************************************************************

		DataOld = DataNow;
// movement of the ball
		if (AngleY > 10000) {
			fMovedX = 1;
			fMovedY = 1;

			if (X < 215)
				X += 1;

		} else if (AngleY < -10000) {
			fMovedX = 1;
			fMovedY = 1;

			if (X > 25)
				X -= 1;

		}

		if (AngleX > 10000) {
			fMovedX = 1;
			fMovedY = 1;

			if (Y < 295)
				Y += 1;

		} else if (AngleX < -10000) {
			fMovedX = 1;
			fMovedY = 1;

			if (Y > 25)
				Y -= 1;

		}

		if (fMovedY == 1 && (AngleY <= 10000 && AngleY >= -10000)
				|| (AngleY > 20000 || AngleY < -20000)) {
			ResetTimeY += 1;

		} else if (fMovedY == 1 && (AngleY > 10000 || AngleY < -10000)) {
			ResetTimeY = 0;
		}

		if (fMovedX == 1 && (AngleX <= 10000 && AngleX >= -10000)
				|| (AngleX > 20000 || AngleX < -20000)) {
			ResetTimeX += 1;

		} else if (fMovedX == 1 && (AngleX > 10000 || AngleX < -10000)) {
			ResetTimeX = 0;
		}

		if (ResetTimeX >= 1000) {
			AngleX = 0;

			ResetTimeX = 0;
			fMovedX = 0;
		}

		if (ResetTimeY >= 1000) {
			AngleY = 0;

			ResetTimeY = 0;
			fMovedY = 0;
		}

	}

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
