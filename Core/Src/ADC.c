/*
 * ADC.c
 *
 *  Created on: 2022/02/16
 *      Author: leopi
 */

#include "ADC.h"

#include <stdio.h>
uint16_t adcarien[5]={0};//ADC1_CH_NUM

void ADCStart()
{  //AD値のDMA
	//printf("p\r\n");
	if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adcarien, ADC1_CH_NUM) != HAL_OK)
	{
		//printf("s\r\n");
		Error_Handler();
	}
	//printf("q\r\n");
}
void ADCStop()
{
	if (HAL_ADC_Stop_DMA(&hadc1) != HAL_OK)
	{
		printf("な\r\n");
		Error_Handler();
		printf("に\r\n");
	}

}


