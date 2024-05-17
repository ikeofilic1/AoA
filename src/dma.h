/*
 * dma.c
 *
 *  Created on: Apr 19, 2024
 *      Author: Ikechukwu Ofili
 */

#ifndef DMA_H_
#define DMA_H_
#include <stdint.h>
#include <stdbool.h>

// Leave this define here
#define NUM_DMA_SAMPLES 32

extern volatile uint16_t DMABufferA[];
extern volatile uint16_t DMABufferB[];
extern volatile uint16_t *DMACurrentBuffer; // Will either be DMABufferA or DMABufferB

void initDMA();
void stopDMA();
void reEnableDMA();
uint16_t DMAGetCurrentIndex();

#endif
