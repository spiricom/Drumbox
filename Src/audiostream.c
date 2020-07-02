/*
 * audiostream.c
 *
 *  Created on: Aug 30, 2019
 *      Author: jeffsnyder
 */


/* Includes ------------------------------------------------------------------*/
#include "audiostream.h"
#include "main.h"
#include "leaf.h"
#include "codec.h"
#include "tim.h"
#include "ui.h"
#include "adc.h"
#define NUM_BUTTONS 3


//the audio buffers are put in the D2 RAM area because that is a memory location that the DMA has access to.
int32_t audioOutBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
int32_t audioInBuffer[AUDIO_BUFFER_SIZE] __ATTR_RAM_D2;
uint16_t ADC3_values[NUM_EXT_ADC_CHANNELS * AUDIO_FRAME_SIZE] __ATTR_RAM_D3;
void audioFrame(uint16_t buffer_offset);
float audioTickL(float audioIn, int sampleNum);
float audioTickR(float audioIn, int sampleNum);
void buttonCheck(void);
tHighpass dcBlock[3];


HAL_StatusTypeDef transmit_status;
HAL_StatusTypeDef receive_status;


uint8_t codecReady = 0;

volatile uint8_t buttonValues[NUM_BUTTONS];
volatile uint8_t buttonValuesPrev[NUM_BUTTONS];
volatile uint32_t buttonCounters[NUM_BUTTONS];
volatile uint32_t buttonPressed[NUM_BUTTONS];

volatile float audioADCInputs[NUM_EXT_ADC_CHANNELS][ADC_RING_BUFFER_SIZE];
uint64_t currentADC3BufferPos = 0;
uint64_t frameCounter2 = 0;


float sample = 0.0f;

uint16_t frameCounter = 0;

//audio objects
tRamp adc[6];
tNoise noise;
tNoise noise2;
//tCycle mySine[6];
tVZFilter shelf1;
tVZFilter shelf2;
tVZFilter bell1;
tVZFilter bell2;

//MEMPOOLS
#define SMALL_MEM_SIZE 5000
char smallMemory[SMALL_MEM_SIZE];

#define MEDIUM_MEM_SIZE 500000
char mediumMemory[MEDIUM_MEM_SIZE] __ATTR_RAM_D1;

#define LARGE_MEM_SIZE 33554432 //32 MBytes - size of SDRAM IC
char largeMemory[LARGE_MEM_SIZE] __ATTR_SDRAM;

tMempool smallPool;
tMempool largePool;


/**********************************************/

typedef enum BOOL {
	FALSE = 0,
	TRUE
} BOOL;

int sampleNumGlobal = 0;


tCycle sine;


void audioInit(I2C_HandleTypeDef* hi2c, SAI_HandleTypeDef* hsaiOut, SAI_HandleTypeDef* hsaiIn)
{
	// Initialize LEAF.

	LEAF_init(SAMPLE_RATE, AUDIO_FRAME_SIZE, mediumMemory, MEDIUM_MEM_SIZE, &randomNumber);

	tMempool_init (&smallPool, smallMemory, SMALL_MEM_SIZE);
	tMempool_init (&largePool, largeMemory, LARGE_MEM_SIZE);

	for (int i = 0; i < 6; i++)
	{
		tRamp_initToPool(&adc[i],7.0f, 1, &smallPool); //set all ramps for knobs to be 7ms ramp time and let the init function know they will be ticked every sample

	}
	tNoise_initToPool(&noise, PinkNoise, &smallPool);
	tNoise_initToPool(&noise2, PinkNoise, &smallPool);
	for (int i = 0; i < 6; i++)
	{
		//tCycle_initToPool(&mySine[i], &smallPool);
		//tCycle_setFreq(&mySine[i], 440.0f);
	}

	HAL_Delay(10);

	tVZFilter_init(&shelf1, Lowshelf, 80.0f, 4.0f);
	tVZFilter_init(&shelf2, Highshelf, 12000.0f, 4.0f);
	tVZFilter_init(&bell1, Bell, 1000.0f, 1.9f);
	tVZFilter_init(&bell2, Bell, 6000.0f, 1.9f);

	tHighpass_init(&dcBlock[0], 30.0f);
	tHighpass_init(&dcBlock[1], 30.0f);
	tHighpass_init(&dcBlock[2], 30.0f);

	for (int i = 0; i < AUDIO_BUFFER_SIZE; i++)
	{
		audioOutBuffer[i] = 0;
	}


	tCycle_init(&sine);
	tCycle_setFreq(&sine, 440.0f);



	if (HAL_ADC_Start_DMA(&hadc3,(uint32_t*)&ADC3_values,NUM_EXT_ADC_CHANNELS * AUDIO_FRAME_SIZE) != HAL_OK)
	{
	  Error_Handler();
	}
	//HAL_Delay(1);
	// set up the I2S driver to send audio data to the codec (and retrieve input as well)
	transmit_status = HAL_SAI_Transmit_DMA(hsaiOut, (uint8_t *)&audioOutBuffer[0], AUDIO_BUFFER_SIZE);
	receive_status = HAL_SAI_Receive_DMA(hsaiIn, (uint8_t *)&audioInBuffer[0], AUDIO_BUFFER_SIZE);

	// with the CS4271 codec IC, the SAI Transmit and Receive must be happening before the chip will respond to
	// I2C setup messages (it seems to use the masterclock input as it's own internal clock for i2c data, etc)
	// so while we used to set up codec before starting SAI, now we need to set up codec afterwards, and set a flag to make sure it's ready

	//now to send all the necessary messages to the codec
	AudioCodec_init(hi2c);

}


void audioFrame(uint16_t buffer_offset)
{
	int i;
	int32_t current_sample = 0;
	frameCounter2++;
	//HAL_ADC_Start_DMA(&hadc3,(uint32_t*)&ADC3_values[(buffer_offset > 0)], 3*AUDIO_FRAME_SIZE);
/*
	currentADC3BufferPos = 0;
	for (int j= 0; j < AUDIO_FRAME_SIZE; j++)
	{

		//tempInt = ADC3_values[(buffer_offset == 0)][currentADC3BufferPos];
		tempInt = ADC3_values[currentADC3BufferPos];
		tempInt2 = tempInt - TWO_TO_15;
		tempFloat = (float)tempInt2;
		tempFloat = tempFloat * INV_TWO_TO_15;
		audioADCInputs[0][j] = tempFloat;
		if (audioADCInputs[0][j] > 0.1f)
		{
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		}
		currentADC3BufferPos = currentADC3BufferPos+3;
	}
*/

	buttonCheck();

	//read the analog inputs and smooth them with ramps
	for (i = 0; i < 6; i++)
	{
		tRamp_setDest(&adc[i], (ADC_values[i] * INV_TWO_TO_16));
	}


	//if the codec isn't ready, keep the buffer as all zeros
	//otherwise, start computing audio!

	if (codecReady)
	{
		for (i = 0; i < (HALF_BUFFER_SIZE); i++)
		{
			if ((i & 1) == 0)
			{
				current_sample = (int32_t)(audioTickR((float) ((audioInBuffer[buffer_offset + i] << 8) * INV_TWO_TO_31), i/2) * TWO_TO_23);
			}
			else
			{
				current_sample = (int32_t)(audioTickL((float) ((audioInBuffer[buffer_offset + i] << 8) * INV_TWO_TO_31), i/2) * TWO_TO_23);
			}

			audioOutBuffer[buffer_offset + i] = current_sample;
		}
	}
}
float rightIn = 0.0f;

volatile int dummy;
int inAttack;

float mic0;
float mic1;
float mic2;

float peak[3];

float audioTickL(float audioIn, int sampleNum)
{

	sample = 0.0f;

	mic0 = audioADCInputs[0][sampleNumGlobal];
	mic1 = audioADCInputs[1][sampleNumGlobal];
	mic2 = audioADCInputs[2][sampleNumGlobal];

	int position = currentADC3BufferPos;

	if (currentADC3BufferPos < sampleNumGlobal)
	{
		position += ADC_RING_BUFFER_SIZE;
	}


	if (mic0 >= 0.1f || mic1 >= 0.1f || mic2 >= 0.1f)
	{
		//read this until currentADC3BufferPos, check for peak
		for (int i = sampleNumGlobal; i <= position; i++)
				{
					if (fabs(audioADCInputs[0][i+1]) > fabs(audioADCInputs[0][i]))
					{
						peak[0] = fabs(audioADCInputs[0][i]);
					}
					if (fabs(audioADCInputs[1][i+1]) > fabs(audioADCInputs[1][i]))
					{
						peak[1] = fabs(audioADCInputs[1][i]);
					}
					if (fabs(audioADCInputs[2][i+1]) > fabs(audioADCInputs[2][i]))
					{
						peak[2] = fabs(audioADCInputs[2][i]);
					}
				}
		dummy++;
	}

	if (peak[0] > peak[1] && peak[0] > peak[2])
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_Delay(15);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
	}
	else if (peak[1] > peak[0] && peak[1] > peak[2])
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		HAL_Delay(15);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
	}
	else if (peak[2] > peak[0] && peak[2] > peak[1])
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
		HAL_Delay(15);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	}


	//A2db db2a funtions > decibels


	// buttons: G6 G7 D11
	// leds: B15 C6 A8 D12 A9 // brainboard led: B4


	sampleNumGlobal++;
	if (sampleNumGlobal >= 2048)
	{
		sampleNumGlobal = 0;
	}
	return sample;

/*
	float myFreq = tRamp_tick(&adc[0])*880.0f;
	tCycle_setFreq(&sine, myFreq);
	sample = tCycle_tick(&sine);
*/


	return sample;
}


uint32_t myCounter = 0;

float audioTickR(float audioIn, int sampleNum)
{
	rightIn = audioIn;
	//sample = (audioADCInputs[1][sampleNumGlobal] + audioADCInputs[2][sampleNumGlobal]) * 0.5f;
	sample = audioADCInputs[2][sampleNumGlobal];
	//sample = 0.0f;
	return sample;
}


volatile uint8_t LED_States[3] = {0,0,0};
void buttonCheck(void)
{
	buttonValues[0] = !HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_6);
	buttonValues[1] = !HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_7);
	//buttonValues[2] = !HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11);
	for (int i = 0; i < NUM_BUTTONS; i++)
	{
	  if ((buttonValues[i] != buttonValuesPrev[i]) && (buttonCounters[i] < 10))
	  {
		  buttonCounters[i]++;
	  }
	  if ((buttonValues[i] != buttonValuesPrev[i]) && (buttonCounters[i] >= 10))
	  {
		  if (buttonValues[i] == 1)
		  {
			  buttonPressed[i] = 1;
		  }
		  buttonValuesPrev[i] = buttonValues[i];
		  buttonCounters[i] = 0;
	  }
	}

	if (buttonPressed[0] == 1)
	{


		if (LED_States[0] == 0)
		{
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
			LED_States[0] = 1;
		}
		else
		{
			//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
			LED_States[0] = 0;
		}
		buttonPressed[0] = 0;

	}
	if (buttonPressed[1] == 1)
	{
		if (LED_States[1] == 0)
		{
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			LED_States[1] = 1;
		}
		else
		{
			//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			LED_States[1] = 0;
		}
		buttonPressed[1] = 0;
	}

	if (buttonPressed[2] == 1)
	{
		if (LED_States[2] == 0)
		{

			LED_States[2] = 1;
		}
		else
		{

			LED_States[2] = 0;
		}
		buttonPressed[2] = 0;
	}
}

void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai)
{
	;
}

void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai)
{
	;
}

void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
  ;
}


void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai)
{
	audioFrame(HALF_BUFFER_SIZE);
}

void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai)
{
	audioFrame(0);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) //buffer just filled
{
	if (hadc == &hadc3)
	{
		for (int i = 0; i < AUDIO_FRAME_SIZE; i++)
		{
			for (int j = 0; j < NUM_EXT_ADC_CHANNELS; j++)
			{
				int tempInt = ADC3_values[(i*NUM_EXT_ADC_CHANNELS) + j];
				audioADCInputs[j][currentADC3BufferPos] = tHighpass_tick(&dcBlock[j],((float)(tempInt - TWO_TO_15) * INV_TWO_TO_15));
			}
			currentADC3BufferPos++;
			if (currentADC3BufferPos >= 2048)
			{
				currentADC3BufferPos = 0;
			}
		}
	}
}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
	;
}
void HAL_ADC_Error(ADC_HandleTypeDef *hadc)
{
	;
}
