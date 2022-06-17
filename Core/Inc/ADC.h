/*
 * ADC.h
 *
 *  Created on: Feb 12, 2022
 *      Author: leopi
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <main.h>

//adcの大元

//壁センサデータ四つ、バッテリのADC一つ
extern ADC_HandleTypeDef hadc1;

#define ADC1_CH_NUM		5


extern uint16_t adcarien[ADC1_CH_NUM];


void ADCStart();  //AD値のDMA

void ADCStop();
#endif /* INC_ADC_H_ */
