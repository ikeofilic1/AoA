#include <stdint.h>
#include "adc.h"
#include "nvic.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"

void initADC0(void)
{
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    _delay_cycles(16);

    // Set up sequence sampler
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN0; // Disable the sequencer for configuration

    ADC0_EMUX_R &= ~ADC_EMUX_EM0_M;
    ADC0_EMUX_R |= ADC_EMUX_EM0_ALWAYS; // SS0 is always reading

    // Sample Order:
    // AIN1 -> AIN2 -> AIN4 -> AIN0
    ADC0_SSMUX0_R = 1 << 0 | 2 << 4 | 4 << 8 | 0 << 12
            | 1 << 16 | 2 << 20 | 4 << 24 | 0 << 28;
    ADC0_SSCTL0_R = ADC_SSCTL0_END7 | ADC_SSCTL0_IE7; // Only read 7 samples, send an interrupt after

    // Enable NVIC
    enableNvicInterrupt(INT_ADC0SS0);
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN0; // Enable the sequencer
}

void initADC1(void)
{
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R1;
    _delay_cycles(16);

    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN1; // Disable the sequencer

    // Digital comparator stuff
    ////////////////////////////////////////////////////

//    ADC1_SSMUX0_R = 1 << 0 | 2 << 4 | 4 << 8 | 0 << 12
//            | 1 << 16 | 2 << 20 | 4 << 24 | 0 << 28;
//
//    ADC1_EMUX_R &= ~ADC_EMUX_EM0_M;
//    ADC1_EMUX_R |= ADC_EMUX_EM0_ALWAYS; // SS0 is always reading
//
//    ADC1_SSCTL0_R = ADC_SSCTL0_END3;
//
//    //Enable Digital Comparator for ADC 1 --> HIGH BAND
//    ADC1_SSOP0_R = ADC_SSOP0_S3DCOP | ADC_SSOP0_S2DCOP | ADC_SSOP0_S1DCOP | ADC_SSOP0_S0DCOP; // Use dig comparator, not FIFO
//    ADC1_SSDC0_R = 0 << 0 | 1 << 4 | 2 << 8 | 3 << 12; // use DCn for each n=0,1,2,3 sample in the SS
//
//    ADC1_DCCTL0_R = ADC_DCCTL0_CIE | ADC_DCCTL0_CIM_ALWAYS | ADC_DCCTL0_CIC_HIGH;        //Digital Comparator Configs
//    ADC1_DCCTL1_R = ADC_DCCTL1_CIE | ADC_DCCTL1_CIM_ALWAYS | ADC_DCCTL1_CIC_HIGH;        //CIE - Interrupt Enable
//    ADC1_DCCTL2_R = ADC_DCCTL2_CIE | ADC_DCCTL2_CIM_ALWAYS | ADC_DCCTL2_CIC_HIGH;        //ALWAYS - interrupt as long as past high
//    ADC1_DCCTL3_R = ADC_DCCTL3_CIE | ADC_DCCTL3_CIM_ALWAYS | ADC_DCCTL3_CIC_HIGH;        //HIGH - Mid Band
//
//    // TODO: fix default value
//    ADC1_DCCMP0_R = (0xFFF << ADC_DCCMP0_COMP1_S) | 0xFFF;
//    ADC1_DCCMP1_R = (0xFFF << ADC_DCCMP0_COMP1_S) | 0xFFF;
//    ADC1_DCCMP2_R = (0xFFF << ADC_DCCMP0_COMP1_S) | 0xFFF;
//    ADC1_DCCMP3_R = (0xFFF << ADC_DCCMP0_COMP1_S) | 0xFFF;
//
//    ADC1_ISC_R = ADC_ISC_DCINSS0;

    ADC1_SSMUX1_R = 1 << 0 | 2 << 4 | 4 << 8;

    ADC1_EMUX_R &= ~ADC_EMUX_EM1_M;
    ADC1_EMUX_R |= ADC_EMUX_EM1_ALWAYS; // SS1 is always reading

    ADC1_SSCTL1_R = ADC_SSCTL1_END2;
    ADC1_SSOP1_R = ADC_SSOP1_S2DCOP | ADC_SSOP1_S1DCOP | ADC_SSOP1_S0DCOP; // Use dig comparator, not FIFO
    ADC1_SSDC1_R = 0 << 0 | 1 << 4 | 2 << 8; // use DCn for each n=0,1,2,3 sample in the SS

    ADC1_DCCTL0_R = ADC_DCCTL0_CIE | ADC_DCCTL0_CIM_ALWAYS | ADC_DCCTL0_CIC_HIGH;        //Digital Comparator Configs
    ADC1_DCCTL1_R = ADC_DCCTL1_CIE | ADC_DCCTL1_CIM_ALWAYS | ADC_DCCTL1_CIC_HIGH;        //CIE - Interrupt Enable
    ADC1_DCCTL2_R = ADC_DCCTL2_CIE | ADC_DCCTL2_CIM_ALWAYS | ADC_DCCTL2_CIC_HIGH;        //ALWAYS - interrupt as long as past high
    //ADC1_DCCTL3_R = ADC_DCCTL3_CIE | ADC_DCCTL3_CIM_ALWAYS | ADC_DCCTL3_CIC_HIGH;        //HIGH - Mid Band

    // TODO: fix default value
    ADC1_DCCMP0_R = (900 << ADC_DCCMP0_COMP1_S);
    ADC1_DCCMP1_R = (1100 << ADC_DCCMP0_COMP1_S);
    ADC1_DCCMP2_R = (900 << ADC_DCCMP0_COMP1_S);
    //ADC1_DCCMP3_R = (600 << ADC_DCCMP0_COMP1_S);

    ADC1_IM_R |= ADC_IM_DCONSS1;
    enableNvicInterrupt(INT_ADC1SS1);

    ADC1_ACTSS_R |= ADC_ACTSS_ASEN1; // Enable the sequencer

    ////////////////////////////////////////////////////
    // Hardware Average mode (controlled by software)

//    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN0;
//
//    ADC1_EMUX_R &= ~ADC_EMUX_EM0_M;
//    ADC1_EMUX_R |= ADC_EMUX_EM0_PROCESSOR;
//
//    ADC1_SSMUX0_R = 1 << 0 | 2 << 4 | 4 << 8 | 0 << 12 | 1 << 16 | 2 << 20 | 4 << 24 | 0 << 28;
//    ADC1_SSCTL0_R = ADC_SSCTL0_END7 | ADC_SSCTL0_IE7;
//
//    ADC1_IM_R |= ADC_IM_MASK0;
//    enableNvicInterrupt(INT_ADC1SS0);
//
//    ADC1_ACTSS_R |= ADC_ACTSS_ASEN0; // Enable the sequencer
}

void setDigCompRange(uint8_t channel, uint16_t low, uint16_t high)
{
    if (channel > 3) return;

    if (channel == 0)
        ADC1_DCCMP0_R = high << 16 | low;
    else if (channel == 1)
        ADC1_DCCMP1_R = high << 16 | low;
    else if (channel == 2)
        ADC1_DCCMP2_R = high << 16 | low;
    else
        ADC1_DCCMP3_R = high << 16 | low;
}

#define MIN(x, y) (x < y ? x : y)
#define MAX(x, y) (x > y ? x : y)

uint16_t micAvgOffsets[4] = {250, 150, 50, 80};
uint16_t micAverages[4];

void avgTimerISR(void)
{
    // Enable averaging, turn off sequencer 1 if needed!!
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN1;
//    ADC1_DCCTL0_R = 0;        //Digital Comparator Configs
//    ADC1_DCCTL1_R = 0;        //CIE - Interrupt Enable
//    ADC1_DCCTL2_R = 0;        //ALWAYS - interrupt as long as past high
//    ADC1_DCCTL3_R = 0;        //HIGH - Mid Band


    ADC1_SAC_R = ADC_SAC_AVG_64X;
    ADC1_PSSI_R = ADC_PSSI_SS0; // start averaging

    TIMER4_ICR_R = TIMER_ICR_TATOCINT;
}

void ADC1SS0ISR()
{
    int i, j;

    ADC1_SAC_R &= ~ADC_SAC_AVG_M;

    // Reset Average registers
    for (j = 0; j < 4; ++j)
        micAverages[j] = 0;

    // Get the average
    for (i = 0; i < 2; ++i)
        for (j = 0; j < 4; ++j)
            micAverages[j] += ADC1_SSFIFO0_R;

    for (j = 0; j < 4; ++j)
    {
        //int16_t lo = avg[j]/2 - micAvgOffsets[j];
        uint16_t hi = micAverages[j]/2 + micAvgOffsets[j];

        // TODO: set all 4 at once
        setDigCompRange(j, 0, MIN(hi, 0xFFF));
    }

    // Turn on sequencer 1 here
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN1;
//    ADC1_DCCTL0_R = ADC_DCCTL0_CIE | ADC_DCCTL0_CIM_ALWAYS | ADC_DCCTL0_CIC_HIGH;        //Digital Comparator Configs
//    ADC1_DCCTL1_R = ADC_DCCTL1_CIE | ADC_DCCTL1_CIM_ALWAYS | ADC_DCCTL1_CIC_HIGH;        //CIE - Interrupt Enable
//    ADC1_DCCTL2_R = ADC_DCCTL2_CIE | ADC_DCCTL2_CIM_ALWAYS | ADC_DCCTL2_CIC_HIGH;        //ALWAYS - interrupt as long as past high
//    ADC1_DCCTL3_R = ADC_DCCTL3_CIE | ADC_DCCTL3_CIM_ALWAYS | ADC_DCCTL3_CIC_HIGH;        //HIGH - Mid Band

    ADC1_ISC_R = ADC_ISC_IN0;
}
