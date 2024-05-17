/*
 * adc.h
 *
 *  Created on: Apr 21, 2024
 *      Author: Ikechukwu Ofili
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

extern uint16_t micAverages[4];

void initADC1(void);
void initADC0(void);

void setDigCompRange(uint8_t channel, uint16_t low, uint16_t high);

#endif /* ADC_H_ */
