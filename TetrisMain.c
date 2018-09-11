/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef struct _tetronimo {
	int type;
	// Type 1 Square
	// _ _ _ _
	// _ O O _
	// _ O O _
	// _ _ _ _
	// Type 2 Line
	// _ O _ _
	// _ O _ _
	// _ O _ _
	// _ O _ _
	// Type 3 L
	// _ O _ _
	// _ O _ _
	// _ O O _
	// _ _ _ _
	// Type 4 Reverse L
	// _ O _ _
	// _ O _ _
	// O O _ _
	// _ _ _ _
	// Type 5 Squiggly
	// O O _ _
	// _ O O _
	// _ _ _ _
	// _ _ _ _
	// Type 6 Reverse Squiggly
	// _ O O _
	// O O _ _
	// _ _ _ _
	// _ _ _ _
	// Type 7 T
	// _ O _ _
	// O O O _
	// _ _ _ _
	// _ _ _ _
	int yCord; //Y Cord, the top left corner of the 4x4 array
	int xCord; //X Cord, the top left corner of the 4x4 array
	int pix[4][4]; //A 4x4 array filled in with 1's wherever there is a part of the block
} tetronimo;
int dotSet[64]; //Values stores for setting a line of the LED matrix
int dotValues[64][4]; //Lines to set to a section of the LED matrix
int mscounter = 0; //Milliseconds counter
int upCoolDown = 0; //Controls how often the user can enter an upward input
int moveCoolDown = 0; //Controls how often the user can enter a sideways input
int nextType = 0; //The next shape of the object
int difficulty = 1000; //Controls how fast the blocks move
int seconds = 0; //Used for cooldowns
int downCoolDown = 0; //Controls how often the user can enter a downward input
int refreshCoolDown = 0; //Controls how often the square refreshes
static int ledMatrix[18][16] = { 0 }; //The matrix that stores where the landed tetronimos are
static int score = 0; //The score, currently in points according to tetris wiki
int rotate = 0; //An interrupt for the button will set this to 1 when a rotation is desired
static int adcX = 0; //The adc X value
static int adcY = 0; //The adc Y value
static tetronimo block; //The tetris block on the screen
unsigned char disp_buf[2];
char transmitChar[2]; //Used for the two micro controllers to talk to each other
char str[100];
uint8_t aRxBuffer[20]; //Used for the micro controller to read in variables
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void scan_line(int i);
void send(unsigned char data);
void microDelay(int Delay);
void adcEnableChannel(int channel);
void setCor(int y, int x, int val);
int getArrIdx(int x, int y);
void rotateBlock(); //Rotates he block
int move(int, int, int); //Moves the block in the given direction
void writeToLED(); //Adds the block to the LED matrix
void spawnBlock(); //Spawn a new block at the top of the screen
void checkForLines();
int checkFailure();
int start = 0;
int startInitialize = 0;
int resetDelay = 0;
void display(int ary[][16]);
void initializeMatrix();
int __io_putchar(int ch);
void transmitString(char*);
uint32_t adcRead(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
//void transmitString(char string[], UART_HandleTypeDef* huart);
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart);
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start_IT(&htim1);
	//HAL_NVIC_EnableIRQ(USART1_IRQn);
	int i = 1;
	for (; i <= 16; i++) {
		int j = 1;
		for (; j <= 16; j++) {
			setCor(i, j, 0);
		}
	}
	//time_t t;
	nextType = rand() % 7 + 1;
	HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 1);
	//HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
	//HAL_NVIC_EnableIRQ(USART1_IRQn);
	//initializeMatrix(); //Set everything in the matrix to 0
	//spawnBlock();   //Spawn the first block
	//USART1->CR1=USART_CR1_RE|USART_CR1_TE|USART_CR1_UE|USART_CR1_RXNEIE;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		for (int i = 0; i < 16; i++) { //This loop writes the Array to the LED Matrix
			GPIOB->ODR = 0x8;				//Set G High, Prevents Ghosting
			int j = 2 * i;
			disp_buf[0] = 16 * dotSet[j] + dotSet[j + 1];		//Get Upper LEDs
			disp_buf[1] = 16 * dotSet[j + 32] + dotSet[j + 33]; //Get Lower LEDs
			//Send Data
			send(disp_buf[1]);			//Write Data
			send(disp_buf[0]);
			GPIOC->ODR = 0x400;	//Set LAT High (Don't know what this does, but code doesn't work w/o it
			microDelay(1);
			GPIOC->ODR = 0x0;				//Set LAT Low
			microDelay(1);
			scan_line(i);					//Switch Selected Row in Dot Matrix
			GPIOB->ODR &= 0xFFF0;			//Set G Low
			microDelay(100);
		}
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if(start == 1) { //Depending on the game mode, alters what is displayed
			if(startInitialize) { //This section is the main game logic
				initializeMatrix();
				spawnBlock();
				startInitialize = 0;
				difficulty = 1000;
			}
			if (rotate == 1) {
				rotate = 0;
				rotateBlock();
			}
			if (moveCoolDown == 0) { //Reads the ADC inputs from the joysticks to determine if movement should be done
				adcEnableChannel(3);
				HAL_ADC_Start(&hadc);
				HAL_ADC_PollForConversion(&hadc, 1500);
				adcX = HAL_ADC_GetValue(&hadc);
				HAL_ADC_Stop(&hadc);
				adcEnableChannel(2);
				HAL_ADC_Start(&hadc);
				HAL_ADC_PollForConversion(&hadc, 1500);
				adcY = HAL_ADC_GetValue(&hadc);
				HAL_ADC_Stop(&hadc);
				
				if (adcX > 4000) { //Moves the block a direction based on input
					move(0, -1, 1);
					moveCoolDown = 100;
				} else if (adcX < 1000) {
					move(0, 1, 1);
					moveCoolDown = 100;
				}
				if (adcY < 1000) {
					move(1, 0, 1);
					moveCoolDown = 100;
				} else if (adcY > 4000 && adcX > 1000 && adcX < 4000 && upCoolDown == 0) { //Pressing up moves the block all the way down
					while (move(1, 0, 1)) {}
					upCoolDown = 200;
					checkForLines();
					writeToLED();
					spawnBlock();
				}
				checkForLines();
			}
			if (downCoolDown == 0) { //Periodic movement of block downwards
				downCoolDown = difficulty;
				if (!move(1, 0, 1)) { //If the block is not able to move down it returns false, or else it just moves the block down
					writeToLED(); //Write the block permanently onto the ledMatrix
					spawnBlock(); //Spawn a new block
				}
				if (checkFailure()) { //Check if the player has lost the game, this happens when there are blocks in the top two rows of the array
					start = 2;
					resetDelay = 10000;
					char* sWord = "W";
					transmitString(sWord);
					startInitialize = 1;
				}
				checkForLines(); //Check if there are any lines
			}
			if (refreshCoolDown == 0) { //Random # like 30~ fps not sure what should be good
				//This part puts the ledMatrix and block together into a matrix to be printed
				refreshCoolDown = 10;
				int printArry[16][16] = { 0 };
				for (int i = 2; i < 18; i++) {
					for (int j = 0; j < 10; j++) {
						printArry[i - 2][j] = ledMatrix[i][j];
					}
				}
				for (int i = 0; i < 4; i++) {
					for (int j = 0; j < 4; j++) {
						if (block.pix[i][j] == 1) { //If there is a pixel in the blocks pix, write it to the corresponding temp spot
							if (block.yCord + i - 2 >= 0) {
								printArry[block.yCord + i - 2][block.xCord + j] = 1;
							}
						}
					}
				}
				if(nextType == 1) {          //SQUARE
					printArry[7][13] = 1;printArry[7][14] = 1;printArry[8][13] = 1;printArry[8][14] = 1;
				} else if(nextType == 2) {   //LINE
					printArry[6][14] = 1;printArry[7][14] = 1;printArry[8][14] = 1;printArry[9][14] = 1;
				} else if(nextType == 3) {   //L
					printArry[8][14] = 1;printArry[8][13] = 1;printArry[7][13] = 1;printArry[6][13] = 1;
				} else if(nextType == 4) {   //REVERSE L
					printArry[8][13] = 1;printArry[8][14] = 1;printArry[7][14] = 1;printArry[6][14] = 1;
				} else if(nextType == 5) {   //SQUIGGLY
					printArry[7][13] = 1;printArry[7][12] = 1;printArry[8][13] = 1;printArry[8][14] = 1;
				} else if(nextType == 6) {   //REVERSE SQUIGGLY
					printArry[7][13] = 1;printArry[7][14] = 1;printArry[8][13] = 1;printArry[8][12] = 1;
				} else {                       //T
					printArry[7][13] = 1;printArry[8][13] = 1;printArry[8][12] = 1;printArry[8][14] = 1;
				}

				printArry[0 + seconds][10] = 1;
				printArry[3 + seconds][10] = 1;
				printArry[6 + seconds][10] = 1;
				printArry[9 + seconds][10] = 1;
				printArry[12 + seconds][10] = 1;
				if(seconds == 0) {
					printArry[15][10] = 1;
				}
				display(printArry);
			}
		} else if(start == 0) { //Has Left display write TET and right display write RIS
			if(refreshCoolDown == 0) {
				refreshCoolDown = 10;
				int printArry[16][16] = { 0 };
				printArry[1][1] = 1;
				printArry[1][2] = 1;
				printArry[1][3] = 1;
				printArry[2][2] = 1;
				printArry[3][2] = 1;
				printArry[4][2] = 1;
				printArry[5][2] = 1;

				printArry[1][6] = 1;
				printArry[1][7] = 1;
				printArry[1][8] = 1;
				printArry[2][6] = 1;
				printArry[3][6] = 1;
				printArry[3][7] = 1;
				printArry[3][8] = 1;
				printArry[4][6] = 1;
				printArry[5][6] = 1;
				printArry[5][7] = 1;
				printArry[5][8] = 1;

				printArry[1][11] = 1;
				printArry[1][12] = 1;
				printArry[1][13] = 1;
				printArry[2][12] = 1;
				printArry[3][12] = 1;
				printArry[4][12] = 1;
				printArry[5][12] = 1;

				/*printArry[1][1] = 1;
				printArry[1][2] = 1;
				printArry[2][1] = 1;
				printArry[2][3] = 1;
				printArry[3][1] = 1;
				printArry[3][2] = 1;
				printArry[4][1] = 1;
				printArry[4][3] = 1;
				printArry[5][1] = 1;
				printArry[5][3] = 1;

				printArry[1][7] = 1;
				printArry[2][7] = 1;
				printArry[3][7] = 1;
				printArry[4][7] = 1;
				printArry[5][7] = 1;

				printArry[1][11] = 1;
				printArry[1][12] = 1;
				printArry[1][13] = 1;
				printArry[2][11] = 1;
				printArry[3][11] = 1;
				printArry[3][12] = 1;
				printArry[3][13] = 1;
				printArry[4][13] = 1;
				printArry[5][11] = 1;
				printArry[5][12] = 1;
				printArry[5][13] = 1;*/
				display(printArry);
			}

		} else if(start == 2) { //Winner's Display
			if(resetDelay == 0) {
				start = 0;
			}
			int printArry[16][16] = {0};
			for(int i = 3; i < 13; i++) {
				for(int j = 3; j < 6; j++) {
					printArry[i][j] = 1;
				}
			}
			for(int i = 10; i < 13; i++) {
				for(int j = 3; j < 10; j++) {
					printArry[i][j] = 1;
				}
			}
			printArry[5][11] = 1;
			printArry[5][13] = 1;
			printArry[7][11] = 1;
			printArry[7][12] = 1;
			printArry[7][13] = 1;
			printArry[8][10] = 1;
			printArry[8][14] = 1;
			display(printArry);
		} else if(start == 3) { //Loser's Display
			if(resetDelay == 0) {
				start = 0;
			}
			int printArry[16][16] = {0};
			for(int i = 3; i < 15; i++) {
				for(int j = 3; j < 13; j++) {
					printArry[i][j] = 1;
				}
			}
			printArry[3][3] = 0;
			printArry[3][5] = 0;
			printArry[3][6] = 0;
			printArry[3][9] = 0;
			printArry[3][10] = 0;
			printArry[3][12] = 0;
			printArry[4][6] = 0;
			printArry[4][9] = 0;
			printArry[5][6] = 0;
			printArry[5][9] = 0;
			printArry[6][6] = 0;
			printArry[6][9] = 0;
			printArry[7][6] = 0;
			printArry[7][9] = 0;
			printArry[8][6] = 0;
			printArry[8][9] = 0;
			printArry[9][6] = 0;
			printArry[9][9] = 0;
			printArry[13][3] = 0;
			printArry[13][7] = 0;
			printArry[13][8] = 0;
			printArry[13][12] = 0;
			printArry[14][3] = 0;
			printArry[14][4] = 0;
			printArry[14][6] = 0;
			printArry[14][7] = 0;
			printArry[14][8] = 0;
			printArry[14][9] = 0;
			printArry[14][11] = 0;
			printArry[14][12] = 0;
			display(printArry);
		}
	}

  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> SharedAnalog_PA2
     PA3   ------> SharedAnalog_PA3
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD4_Pin|LD3_Pin|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PF4 PF5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin PC10 PC11 
                           PC12 */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|GPIO_PIN_10|GPIO_PIN_11 
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 PB6 
                           PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //How the two micro controller's communicate
{
	if(*aRxBuffer == 'L') {
		for(int i = 1; i < 18; i++) {
			for(int j = 0; j < 10; j++) {
				ledMatrix[i-1][j] = ledMatrix[i][j];
			}
		}
		for(int j = 0; j < 10; j++) {
				ledMatrix[17][j] = rand() % 2;
		}
	} else if (*aRxBuffer == 'W') {
		start = 3;
		resetDelay = 10000;
		startInitialize = 1;
	}
	HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 1);
}
int __io_putchar(int ch) { //Sends one character
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}
void transmitString(char* str) { //Sends an entire string
    while(*str != '\0') {
        __io_putchar(*str);
        str++;
    }
}
// Counts from 0 to 15 where each number represents a row
void scan_line(int i) {
	switch (i) {
	case 0:
		GPIOB->ODR = 0x8;
		break;
	case 1:
		GPIOB->ODR = 0x18;
		break;
	case 2:
		GPIOB->ODR = 0x28;
		break;
	case 3:
		GPIOB->ODR = 0x38;
		break;
	case 4:
		GPIOB->ODR = 0x48;
		break;
	case 5:
		GPIOB->ODR = 0x58;
		break;
	case 6:
		GPIOB->ODR = 0x68;
		break;
	case 7:
		GPIOB->ODR = 0x78;
		break;
	case 8:
		GPIOB->ODR = 0x88;
		break;
	case 9:
		GPIOB->ODR = 0x98;
		break;
	case 10:
		GPIOB->ODR = 0xA8;
		break;
	case 11:
		GPIOB->ODR = 0xB8;
		break;
	case 12:
		GPIOB->ODR = 0xC8;
		break;
	case 13:
		GPIOB->ODR = 0xD8;
		break;
	case 14:
		GPIOB->ODR = 0xE8;
		break;
	default:
		GPIOB->ODR = 0xF8;
		break;
	}
}
void send(unsigned char data) {
	int i = 0;
	for (; i < 8; i++) {
		if (data & 0x01) {
			GPIOC->ODR &= 0x0FFF; //Turn DI register on
			GPIOC->ODR |= 0x1000;
		} else {
			GPIOC->ODR &= 0x0FFF; //Turn DI register off
		}
		microDelay(1);
		GPIOC->ODR |= 0x0800;	//Clock Rising Edge (send data on rising edge)
		microDelay(1);
		GPIOC->ODR &= 0xF0FF; //Clock Falling Edge
		microDelay(1);
		data >>= 1;
	}
}

void microDelay(int Delay) { //A microsecond delay needed for the program
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while (((uint16_t) __HAL_TIM_GET_COUNTER(&htim2)) < Delay) {
	}
}

uint32_t adcRead(void) {
	uint32_t adcValue = 0;
	// Wait for ADC to be ready
	while ((ADC1->ISR & ADC_ISR_ADRDY) == 0)
		;
	// Start the ADC (ADCStart = 1)
	ADC1->CR |= ADC_CR_ADSTART;
	// Wait for end of conversion
	while ((ADC1->ISR & ADC_ISR_EOC) == 0)
		;
	adcValue = ADC1->DR;
	return adcValue;
}
void adcEnableChannel(int channel) {
	// Wait for ADC to be ready
	while (ADC_ISR_ADRDY == 0)
		;
	// Ensure ADCStart = 0
	while ((ADC1->CR & ADC_CR_ADSTART) == 1)
		;
	ADC1->CHSELR = 0;
	ADC1->CHSELR |= 1 << channel;
}

int getArrIdx(int x, int y) {
	int array_val;
	if (y < 5) {
		array_val = 33 + 2 * (x - 1);
	} else if (y < 9) {
		array_val = 32 + 2 * (x - 1);
	} else if (y < 13) {
		array_val = 1 + 2 * (x - 1);
	} else {
		array_val = 2 * (x - 1);
	}
	return array_val;
}

void setCor(int y, int x, int val) {
	int array_val = getArrIdx(x, y);
	int array_pos = (y - 1) % 4;
	dotValues[array_val][array_pos] = val;
	dotSet[array_val] =
			0xF
					- (dotValues[array_val][0] + 2 * dotValues[array_val][1]
							+ 4 * dotValues[array_val][2]
							+ 8 * dotValues[array_val][3]);
}

void checkForLines() { //Check to see if any lines are completed
    int rowsCleared = 0;
    for(int i = 0; i < 18; i++) {
        int completeRow = 1; //Assume row is a completed row
        for(int j = 0; j < 10; j++) {
            if(ledMatrix[i][j] == 0) {
                completeRow = 0; //If any row values are 0, this becomes false
            }
        }
        if(completeRow) { //If all values in the row were indeed 1, add a point and shift down all above rows
            rowsCleared++;
            for(int k = i; k > 0; k--) { //Take all rows above the completed one and shift them down
                for(int l = 0; l < 10; l++) {
                    ledMatrix[k][l] = ledMatrix[k-1][l];
                }
            }
        }
    }

    if(rowsCleared == 1) {
    	char* tWord = "L";
		transmitString(tWord);
    } else if (rowsCleared == 2) {
        score += 100;
        char* tWord = "L";
		transmitString(tWord);
    } else if (rowsCleared == 3) {
        score += 300;
        char* tWord = "LL";
		transmitString(tWord);
    } else if (rowsCleared == 4) {
        score += 1200;
        char* tWord = "LLL";
		transmitString(tWord);
    }
}

int checkFailure() { //Check if the player has lost
    for(int i = 0; i < 2; i++) {
        for(int j = 0; j < 10; j++) {
            if(ledMatrix[i][j] == 1) {
                return 1;
            }
        }
    }
    return 0;
}

void initializeMatrix() { //Initialize Matrix
    for(int i = 0; i < 18; i++) {
        for(int j = 0; j < 10; j++) {
            ledMatrix[i][j] = 0;
        }
    }
}

void writeToLED() { //Writes the Array to the LED Matrix
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            if(block.pix[i][j] == 1) { //If there is a pixel in the blocks pix, write it to the corresponding ledMatrix spot
                ledMatrix[block.yCord + i][block.xCord + j] = 1;
            }
        }
    }
}

void rotateBlock() { //Rotates a block
    int temp[4][4] = {0}; //A temporary array to store rotated value in case rotation is not possible.
    if(block.type == 1) {
    	temp[1][1] = 1;
    	temp[1][2] = 1;
    	temp[2][1] = 1;
    	temp[2][2] = 1;
    } else if(block.type == 2) { //Special rotation for line block
        if(block.pix[0][1]) {
            temp[1][0] = 1; temp[1][1] = 1; temp[1][2] = 1; temp[1][3] = 1;
        } else if (block.pix[1][3]) {
            temp[0][2] = 1; temp[1][2] = 1; temp[2][2] = 1; temp[3][2] = 1;
        } else if (block.pix[3][2]) {
            temp[2][0] = 1; temp[2][1] = 1; temp[2][2] = 1; temp[2][3] = 1;
        } else {
            temp[0][1] = 1; temp[1][1] = 1; temp[2][1] = 1; temp[3][1] = 1;
        }
    } else if (block.type > 2) { // Rotation for every other kind of block
        temp[0][0] = block.pix[2][0];
        temp[0][1] = block.pix[1][0];
        temp[0][2] = block.pix[0][0];
        temp[1][0] = block.pix[2][1];
        temp[1][2] = block.pix[0][1];
        temp[2][0] = block.pix[2][2];
        temp[2][1] = block.pix[1][2];
        temp[2][2] = block.pix[0][2];
        temp[1][1] = 1;
    }
    for(int i = 0; i < 4; i++) { //Check is the rotated block has a valid place it can rotate to
        for(int j = 0; j < 4; j++) {
            if(temp[i][j] == 1) { //Checks if that pixel in the block is a part of the current tetronimo
                if(block.xCord + j > 9 || block.xCord + j < 0 || ledMatrix[i][j] == 1 || block.yCord + i > 19) { //Check bounds or if it another block is in the way
                    return; //Return if something is in the way
                }
            }
        }
    }
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            block.pix[i][j] = temp[i][j];
        }
    }
}

int move(int yCord,int xCord, int move) { //the coordinates of what direction the block wants to move.
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            if(block.pix[i][j] == 1) { //Checks if that pixel in the block is a part of the current tetronimo
                if(block.xCord + j + xCord > 9 || block.xCord + j + xCord < 0 || ledMatrix[i + block.yCord + yCord][j + block.xCord + xCord] == 1 || block.yCord + i + yCord > 17) { //Check bounds or if it another block is in the way
                    return 0; //Return 0 for something is in the way
                }
            }
        }
    }
    if(move) { //A toggle for if I want to block to move or if im just having this program check if the block can be moved, may not need
        block.xCord = block.xCord + xCord; //Update xCord
        block.yCord = block.yCord + yCord; //Update yCord
    }
    return 1; //Return true if the block can move in the requested direction
}

void spawnBlock() { //Spawns a new block at the top
    block.xCord = 3;
    block.yCord = 0;
    block.type = nextType;
    nextType = mscounter % 7 + 1;

    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            block.pix[i][j] = 0;
        }
    }
    if(block.type == 1) {          //SQUARE
        block.pix[1][1] = 1;block.pix[1][2] = 1;block.pix[2][1] = 1;block.pix[2][2] = 1;
    } else if(block.type == 2) {   //LINE
        block.pix[0][1] = 1;block.pix[1][1] = 1;block.pix[2][1] = 1;block.pix[3][1] = 1;
    } else if(block.type == 3) {   //L
        block.pix[0][1] = 1;block.pix[1][1] = 1;block.pix[2][1] = 1;block.pix[2][2] = 1;
    } else if(block.type == 4) {   //REVERSE L
        block.pix[0][1] = 1;block.pix[1][1] = 1;block.pix[2][1] = 1;block.pix[2][0] = 1;
    } else if(block.type == 5) {   //SQUIGGLY
        block.pix[0][0] = 1;block.pix[0][1] = 1;block.pix[1][1] = 1;block.pix[1][2] = 1;
    } else if(block.type == 6) {   //REVERSE SQUIGGLY
        block.pix[1][0] = 1;block.pix[1][1] = 1;block.pix[0][1] = 1;block.pix[0][2] = 1;
    } else {                       //T
        block.pix[0][1] = 1;block.pix[1][0] = 1;block.pix[1][1] = 1;block.pix[1][2] = 1;
    }
}
void display(int ary[][16]) {
	int i = 0;
	for(; i < 16; i++) {
		int j = 0;
		for(; j < 16; j++) {
			setCor(i + 1, j + 1, ary[i][j]);
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */ 
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
