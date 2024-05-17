#include "tm4c123gh6pm.h"
#include "nvic.h"
#include "dma.h"

#ifndef NUM_DMA_SAMPLES
#error "Please define the number of samples to write"
#endif

#define XFER_SIZE (8*NUM_DMA_SAMPLES)

typedef struct
{
    uint32_t *src_ptr;
    uint32_t *dst_ptr;
    uint32_t  control;

    uint32_t reserved;
} dma_channel;

#pragma DATA_SECTION(table, ".chtable");
volatile dma_channel table[64];

volatile dma_channel *primary = &table[14];
volatile dma_channel *alternate = &table[32+14];

volatile uint16_t DMABufferA[XFER_SIZE];
volatile uint16_t DMABufferB[XFER_SIZE];

volatile uint16_t *DMACurrentBuffer;

// Function to initialize DMA
void initDMA()
{
    // Enable clock for uDMA
    SYSCTL_RCGCDMA_R |= SYSCTL_RCGCDMA_R0;
    _delay_cycles(3);

    // Enable the µDMA controller
    UDMA_CFG_R |= UDMA_STAT_MASTEN;

    // Set base address register for the control table
    UDMA_CTLBASE_R = (uint32_t)table;

    // Select Primary Control Structure
    UDMA_ALTCLR_R |= (1 << 14);

    // Enable Request Recognition
    UDMA_REQMASKCLR_R |= (1 << 14);

    // Assign DMA channel 14 to ADC0 SS0
    UDMA_CHMAP1_R &= ~UDMA_CHMAP1_CH14SEL_M;

    primary->src_ptr = (uint32_t*)&ADC0_SSFIFO0_R;
    primary->dst_ptr = (uint32_t*)&DMABufferA[XFER_SIZE - 1];
    primary->control = UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_16
        | UDMA_CHCTL_ARBSIZE_8 | (XFER_SIZE-1) << UDMA_CHCTL_XFERSIZE_S | UDMA_CHCTL_XFERMODE_PINGPONG;
    primary->reserved = 0;

    alternate->src_ptr = (uint32_t*)&ADC0_SSFIFO0_R;
    alternate->dst_ptr = (uint32_t*)&DMABufferB[XFER_SIZE - 1];
    alternate->control = UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_16
        | UDMA_CHCTL_ARBSIZE_8 | (XFER_SIZE-1) << UDMA_CHCTL_XFERSIZE_S | UDMA_CHCTL_XFERMODE_PINGPONG;
    alternate->reserved = 0;

    // Enable the channel by setting bit 8 of DMAENASET register
    UDMA_ENASET_R = (1 << 14);
}

uint16_t indexx = 0;

void DMA_Isr(void)
{
    if ((primary->control & UDMA_CHCTL_XFERMODE_M) == 0)
    {
        primary->control = UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_16
            | UDMA_CHCTL_ARBSIZE_8 | (XFER_SIZE-1) << UDMA_CHCTL_XFERSIZE_S | UDMA_CHCTL_XFERMODE_PINGPONG;

        DMACurrentBuffer = DMABufferB;
        indexx = 0;
    }

    if ((alternate->control & UDMA_CHCTL_XFERMODE_M) == 0) // Then it is the alternate channel that is filled
    {
        // Reset the alternate for reading next time
        alternate->control = UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_16
            | UDMA_CHCTL_ARBSIZE_8 | (XFER_SIZE-1) << UDMA_CHCTL_XFERSIZE_S | UDMA_CHCTL_XFERMODE_PINGPONG;

        DMACurrentBuffer = DMABufferB;
        indexx = 0;
    }

    UDMA_CHIS_R = (1 << 14);
    UDMA_ENASET_R = (1 << 14);

    //ADC0_ISC_R = ADC_ISC_IN0;
    indexx += 8;
}

void stopDMA()
{
    UDMA_ENACLR_R = (1 << 14);
    primary->control = alternate->control = UDMA_CHCTL_XFERMODE_STOP;
}

void reEnableDMA()
{
    primary->control = alternate->control = UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_SRCINC_NONE | UDMA_CHCTL_SRCSIZE_16
             | UDMA_CHCTL_ARBSIZE_8 | (XFER_SIZE-1) << UDMA_CHCTL_XFERSIZE_S | UDMA_CHCTL_XFERMODE_PINGPONG;
    UDMA_ENASET_R = (1 << 14);

    indexx = 0;
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN0;
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN0;
}

uint16_t DMAGetCurrentIndex()
{
    if (DMACurrentBuffer == DMABufferA)
        return XFER_SIZE - 1 -  ((primary->control & UDMA_CHCTL_XFERSIZE_M) >> UDMA_CHCTL_XFERSIZE_S);
    else
        return XFER_SIZE - 1 - ((alternate->control & UDMA_CHCTL_XFERSIZE_M) >> UDMA_CHCTL_XFERSIZE_S);
}
