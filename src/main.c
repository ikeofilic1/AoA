#include <adc.h>
#include <tm4c123gh6pm.h>
#include <gpio.h>
#include <dma.h>
#include <clock.h>
#include <wait.h>
#include <nvic.h>
#include <uart0.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#define RED_LED     PORTF,1
#define GREEN_LED   PORTF,3

#define T_OVER_2_US (8*NUM_DMA_SAMPLES)
#define TAU_DIV_2_TICKS (40*T_OVER_2_US)
#define DMA_SIZE (8*NUM_DMA_SAMPLES)

#define N ((2*T_OVER_2_US)/4)
#define L 32

int32_t yx_corr, xz_corr, yz_corr; // if yx_corr > 0 means y comes first

uint32_t corrs_yx[N-L+1];
uint32_t corrs_xz[N-L+1];
uint32_t corrs_yz[N-L+1];

uint16_t x[N], y[N], z[N];
float AoA;

bool tdoA = false;
bool show_waves = true;

int32_t TDOA[4];

// Analog pins
#define AIN0 PORTE,3
#define AIN1 PORTE,2
#define AIN2 PORTE,1
#define AIN4 PORTD,3

bool inUse = false;
bool validevents = false;
bool xcorr = false;

volatile uint16_t *StartBuffer;
volatile uint16_t *StopBuffer;
uint32_t StartIndex = 0;
uint32_t StopIndex = 0;

const float k1 = 0.32374;
//const float k2 = -0.000058218;

//void setUpADC1DigCompMode(void)
//{
//    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN0; // Disable the sequencer
//
//    ADC1_EMUX_R &= ~ADC_EMUX_EM0_M;
//    ADC1_EMUX_R |= ADC_EMUX_EM0_ALWAYS; // SS0 is always reading
//
//    ADC1_SSCTL0_R = ADC_SSCTL0_END3;
//    ADC1_SAC_R = ADC_SAC_AVG_OFF;
//
//    //Enable Digital Comparator for ADC 1 --> HIGH BAND
//    ADC1_SSOP0_R = ADC_SSOP0_S3DCOP | ADC_SSOP0_S2DCOP | ADC_SSOP0_S1DCOP | ADC_SSOP0_S0DCOP; // Use dig comparator, not FIFO
//    ADC1_SSDC0_R = 0 << 0 | 1 << 4 | 2 << 8 | 3 << 12; // use DCn for each n=0,1,2,3 sample in the SS
//
//    ADC1_DCCMP1_R = (DC1Comp1 << ADC_DCCMP0_COMP1_S) | DC1Comp0;
//    ADC1_DCCMP0_R = (DC0Comp1 << ADC_DCCMP0_COMP1_S) | DC0Comp0;
//    ADC1_DCCMP2_R = (DC2Comp0 << ADC_DCCMP0_COMP1_S) | DC2Comp0;
//    ADC1_DCCMP3_R = (DC3Comp0 << ADC_DCCMP0_COMP1_S) | DC3Comp0;
//
//    ADC1_DISC_R = ADC_ISC_DCINSS0;
//    ADC1_IM_R |= ADC_IM_DCONSS0;
//
//    ADC1_ACTSS_R |= ADC_ACTSS_ASEN0; // Enable the sequencer
//}
//
//void setUpADC1AvgMode(void)
//{
//    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN0; // Disable the sequencer
//
//    ADC1_EMUX_R &= ~ADC_EMUX_EM0_M;
//    ADC1_EMUX_R |= ADC_EMUX_EM0_PROCESSOR; // SS0 is always reading
//
//    // Disable Digital comparator interrupts
//    ADC1_IM_R &= ~ADC_IM_DC0NSS0;
//    ADC1_IM_R |= ADC_IM_MASK0;
//
//    ADC1_SSCTL0_R = ADC_SSCTL0_END7 | ADC_SSCTL0_IE7;
//    ADC1_SAC_R = ADC_SAC_AVG_64X;
//
//    ADC1_SSOP0_R  = 0; // Use FIFO not the comparator
//
//    ADC1_ACTSS_R |= ADC_ACTSS_ASEN0; // Enable the sequencer
//
//    // Ask the sampler to start the averaging
//    ADC1_PSSI_R |= ADC_PSSI_SS0;
//}

void startTimerTOver2(void)
{
    // Configure Timer 4 for 1 sec tick
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_16_BIT;                 // configure as 16-bit timer
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = TAU_DIV_2_TICKS;                       // set load value (200us)
    TIMER1_IMR_R |= TIMER_IMR_TATOIM;                // turn-on interrupt

    TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
}

#define MAX_CHARS 80
char strInput[MAX_CHARS+1];
char* token;
uint8_t count = 0;

void processShell()
{
    bool end;
    char c;

    char str[40];

    if (kbhitUart0())
   {
       c = getcUart0();

       end = (c == 13) || (count == MAX_CHARS);
       if (!end)
       {
           if ((c == 8 || c == 127) && count > 0)
               count--;
           if (c >= ' ' && c < 127)
               strInput[count++] = c;
       }
       else
       {
           strInput[count] = '\0';
           count = 0;
           token = strtok(strInput, " ");

           if (strcmp(token, "reset") == 0)
           {
               snprintf(str, sizeof(str), "Hardware reset... \n");
               putsUart0(str);

               NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
           }
           else if (strcmp(token, "aoa") == 0)
           {
               snprintf(str, sizeof(str), "Last AoA was %f\n", AoA);
               putsUart0(str);
           }
           else if (strcmp(token, "tdoa") == 0)
           {
               token = strtok(NULL, " ");
               if (strcmp(token, "on") == 0)
               {
                   tdoA = true;
               }
               else if (strcmp(token, "off") == 0)
               {
                   tdoA = false;
               }
               else
                   putsUart0("Invalid subcommand\n");
           }
           else if (strcmp(token, "waves") == 0)
           {
                token = strtok(NULL, " ");
                if (strcmp(token, "on") == 0)
                {
                  show_waves = true;
                }
                else if (strcmp(token, "off") == 0)
                {
                  show_waves = false;
                }
                else
                  putsUart0("Invalid subcommand\n");
           }
           //else if (strcmp(token, "backo"))
           else
               putsUart0("Invalid command\n");
       }
   }
}

#define DMA_BUFFER_SIZE (8*NUM_DMA_SAMPLES)

void saveWaveToOutput(int16_t start, int16_t time)
{
    char buffer[100];
    // "A","B","C"
    // trigger point
    // aval0, bval0, cval0, dval0
    //  ... ,  ... ,  ... ,  ...
    // avaln, bvaln, cvaln, dvaln

    volatile uint16_t *bufferA = DMABufferA, *bufferB = DMABufferB;

    int32_t print_start = start;
    if (print_start < 0)
    {
        print_start += DMA_BUFFER_SIZE;
        bufferA = DMABufferB;
        bufferB = DMABufferA;
    }
    print_start &= ~3;

    putsUart0("A,B,C,D\n");
    snprintf(buffer, sizeof(buffer), "%hu,%hu,%hu,%hu\n", start, start, start, start);
    putsUart0(buffer);

    int i;
    for (i = print_start; time >= 4 && i <= DMA_BUFFER_SIZE-4; i += 4) {
        snprintf(buffer, sizeof(buffer), "%hu,%hu,%hu,%hu\n", bufferA[i], bufferA[i+1], bufferA[i+2], bufferA[i+3]);
        putsUart0(buffer);
        time -= 4;
    }

    for (i = 0; i <= DMA_BUFFER_SIZE && time >= 4; i += 4) {
        snprintf(buffer, sizeof(buffer), "%hu,%hu,%hu,%hu\n", bufferB[i], bufferB[i+1], bufferB[i+2], bufferB[i+3]);
        putsUart0(buffer);
        time -= 4;
    }

    if (time >= 4)
    {
        for (i = 0; i <= time-4; i += 4) {
            snprintf(buffer, sizeof(buffer), "%hu,%hu,%hu,%hu\n", bufferA[i], bufferA[i+1], bufferA[i+2], bufferA[i+3]);
            putsUart0(buffer);
        }
    }

}

extern uint16_t indexx;
bool correct = true;

void digCompISR(void)
{
    // Clear interrupts
    ADC1_DCISC_R = ADC1_DCISC_R;

    // TODO check for if we are not calculating xcorr currently
    // and if holdoff time has expired before you retrigger another xcorr

    if(!inUse)
   {
        StartIndex = DMAGetCurrentIndex();
        StartBuffer = DMACurrentBuffer;
        inUse = true;

        startTimerTOver2();
   }
}

// TODO: This is where we do most work
void TOver2ISR(void)
{
    stopDMA();
    xcorr = true;
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

void initAvgTimer(void)
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R4;
    _delay_cycles(3);

    // Configure Timer 4 for 1 sec tick
    TIMER4_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER4_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER4_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
    TIMER4_TAILR_R = 4000000;                       // set load value (10 Hz rate)
    TIMER4_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer
    TIMER4_IMR_R |= TIMER_IMR_TATOIM;                // turn-on interrupt

    NVIC_EN2_R |= 1 << (INT_TIMER4A-80);             // turn-on interrupt 86 (TIMER4A)
}

void initHW(void)
{
    initSystemClockTo40Mhz();

    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTE);
    enablePort(PORTF);

    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);

    selectPinAnalogInput(AIN0);
    selectPinAnalogInput(AIN1);
    selectPinAnalogInput(AIN2);
    selectPinAnalogInput(AIN4);

    // Timer 1 one_shot
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    _delay_cycles(3);

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    enableNvicInterrupt(INT_TIMER1A);
}

void readDMA(uint16_t buffer[], uint8_t channel, int16_t start_idx, uint16_t len)
{
    int32_t index = start_idx;
    uint16_t *start_buf = (uint16_t*)StartBuffer;

    if (index < 0)
    {
        // Swap buffers
        start_buf = (start_buf == DMABufferA) ? (uint16_t*)DMABufferB : (uint16_t*)DMABufferA;
        index += DMA_BUFFER_SIZE;
    }
    index &= ~3; // Align to size of 4 samples
    int i;
    for (i = 0; i < len; ++i)
    {
        // Wrap around if the end of the buffer is reached
        if (index >= DMA_BUFFER_SIZE)
        {
            index = 0;  // Reset index to the start of the buffer
            // Swap to the other buffer
            start_buf = (start_buf == DMABufferA) ? (uint16_t*)DMABufferB : (uint16_t*)DMABufferA;
        }
        buffer[i] = start_buf[index + channel];
        index += 4;  // Move to the next set of samples
    }
}

// Find y in x
uint32_t crossCorrelation(const uint16_t y[], const uint16_t x[], uint32_t xcorr[], int window_size, int total_samples)
{
    int i, j;

    int j_offset = 0;

    for (i = 0; i < total_samples - window_size + 1; i++) {
        xcorr[i] = 0;

        for (j = 0; j < window_size && ((i + j) < total_samples); j++) {
            int index = i + j;
            xcorr[i] += x[index] * y[j_offset + j];
        }
    }

    // Find the index of maximum correlation
    int max_index = 0;
    int max_correlation = xcorr[0];

    for (i = 1; i < total_samples - window_size + 1; ++i) {
        if (xcorr[i] > max_correlation) {
            max_index = i;
            max_correlation = xcorr[i];
        }
    }

    return max_index;
}

float calculateAoA()
{
    // x -> mic1
    // y -> mic2
    // z -> mic0

    float theta0;
    int32_t tdoa;

    int16_t start = StartIndex - T_OVER_2_US;

    //  yx => 6, yz => -4, xz => 14

    // TODO: combine to one readDMA
    readDMA(x, 2, start, N);
    readDMA(y, 1, start, N);
    readDMA(z, 0, start, N);

    yx_corr = crossCorrelation(y, x, corrs_yx, L, N) - T_OVER_2_US/4;
    yz_corr = crossCorrelation(y, z, corrs_yz, L, N) - T_OVER_2_US/4;
    xz_corr = crossCorrelation(x, z, corrs_xz, L, N) - T_OVER_2_US/4;

    correct = true;
    if (yx_corr >= 0 && yz_corr >= 0)
    {
        // y is first
        theta0 = 217;
        TDOA[2] = tdoa = xz_corr;

        putsUart0("First: mic 2\n");

        if (yx_corr < yz_corr)
        {
            putsUart0("Second: mic 1\n");
            putsUart0("Third: mic 0\n");
        }
        else
        {
            putsUart0("Second: mic 0\n");
            putsUart0("Third: mic 1\n");
        }

    }
    else if (yz_corr <= 0 && xz_corr <= 0)
    {
        // z is first
        theta0 = 0;
        TDOA[0] = tdoa = yx_corr;

        putsUart0("First: mic 0\n");

        if (yz_corr < xz_corr)
        {
            putsUart0("Second: mic 1\n");
            putsUart0("Third: mic 2\n");
        }
        else
        {
            putsUart0("Second: mic 2\n");
            putsUart0("Third: mic 1\n");
        }
    }
    else if (yx_corr <= 0 && xz_corr >= 0)
    {
        // x is first
        theta0 = 120;
        TDOA[1] = tdoa = yz_corr;

        putsUart0("First: mic 1\n");

        if (-yx_corr < xz_corr)
        {
            putsUart0("Second: mic 2\n");
            putsUart0("Third: mic 0\n");
        }
        else
        {
            putsUart0("Second: mic 0\n");
            putsUart0("Third: mic 2\n");
        }
    }
    else
    {
        //char buffer[60];
//        putsUart0("Cross correlation is wrong!!\n");
//        snprintf(buffer, sizeof(buffer), "TDOAs are: yx => %d, yz => %d, xz => %d\n", yx_corr, yz_corr, xz_corr);
//        putsUart0(buffer);
        correct = false;
    }

    return theta0 + k1*tdoa*4; //+ k2*tdoa*tdoa*16;
}

int main()
{
    initUart0();
    setUart0BaudRate(115200, 40000000);

    initHW();
    initDMA();
    initADC0();
    //initAvgTimer();
    initADC1();

    char buffer[30];

    while (1)
    {
        processShell();

        if (xcorr)
        {
//            if (show_waves)
//            {
//                saveWaveToOutput(StartIndex - 20, 60);
//            }

            AoA = calculateAoA();

            if (correct && show_waves)
            {
                snprintf(buffer, sizeof(buffer), "AoA : %.3f \n\n", AoA);
                putsUart0(buffer);
            }

            reEnableDMA();
            waitMicrosecond(T_OVER_2_US);

            xcorr = false;
            inUse = false; // Move to backoff ISR
        }

        if (tdoA)
        {
            tdoA = false;
            snprintf(buffer, sizeof(buffer), "T1-T2 = %d\n", TDOA[0]);
            putsUart0(buffer);

            snprintf(buffer, sizeof(buffer), "T0-T2 = %d\n", TDOA[1]);
            putsUart0(buffer);

            snprintf(buffer, sizeof(buffer), "T0-T1 = %d\n", TDOA[2]);
            putsUart0(buffer);

            ///waitMicrosecond(100000);
        }
    }
}
