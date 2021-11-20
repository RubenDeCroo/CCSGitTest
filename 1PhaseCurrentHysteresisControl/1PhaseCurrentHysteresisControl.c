/*
 * This program features a hysteresis current control.
 * A timer generates an interrupt which trigger an ADC conversion.
 * The timer is also used to generate a time based reference signal.
 * The end of the ADC conversion trigger another interrupt in which the ADC result is compared to the hysteresis reference band.
 * This comparison generate a switchstate for the converter.
 * This switchstate determines the output of the gate drive pins
 *
 * Current is sensed on ADCINA2 (Pin 29).
 * The output signals are connected to GPIO0 and GPIO1 (Pin 40 and 39).
 *
 * C
 */

#include "IQmathLib.h"
#include "F2837xD_device.h"
#include "F28x_Project.h"
#include "DCLF32.h"
#include "DCL_fdlog.h"
#include "F2837xD_Adc_defines.h"
#include "stdio.h"

/* Defines */
#define CPUFREQ_MHZ         200     // Clock frequency in megaherz
#define DATA_LENGTH         1000    // Data length datalog
#define PI                  3.1415  // pi

/* Prototype variables, functions and ISR's*/

// Current results array
#pragma DATA_SECTION(Current_array, "DataLogSection");
float Current_array[(int)(1000)];                          // float Duty_array[DATA_LENGTH] not working?
FDLOG Buffer2 = FDLOG_DEFAULTS;

// Functions
void ConfigureADC(void);
void ConfigureADCTimer(void);
void ConfigureGPIO(void);

float32_t output;
float32_t GenerateSquareReference(long cnt);
float32_t GenerateSineReference(long cnt);

float32_t GenerateHysteresisBand(void);
void HysteresisControl1(void);
void SwitchGPIO(void);

// ISR's
interrupt void CPU_TIMER_0_ISR(void);
interrupt void ADC_EOC_ISR(void);

/* Global variables*/

// With current linker configuration, real-time controllable variables need to be initialized
long IdleLoopCount = 0;

// ISR counters
long CPU_TIMER_0_ISR_Count = 0;
long ADC_EOC_ISR_Count = 0;

// Initial value for control reference
float I_ref = 0.0f;
// Input voltage: 312.5 mV/A -> 10.56 A = 4096 for old setup
//TMCS3: 200mV/A -> 16.5 = 4096, EXP: 335 mV offset = 475 -> 2772 mV = 4096 = 13.86
float I_scale = 15/4096.0f;
float I_offset = 470.0;
float I_res;

// Timer variables
Uint32 TimerInterruptFreq_hz = 50000;
float cpuPeriod_us = 0.0f;
Uint32 interruptCycles = 0;
float interruptDuration_us = 0.0f;
float samplingPeriod_us = 0.0f;

//Current Reference;
float I_ref_low = 0.0f;
float I_ref_high = 1.0f;
Uint32 ISRCount_timer = 0;
float WaveFreq_hz = 1.0f;
long WaveFreq_count = 0;
long Count = 0;
float32_t phase = 0;

// Signal type
int square_enable = 0;
int sine_enable = 1;

//Hysteresis controller;
float32_t band = 0.2f;

//int switchstate = 0;
float32_t testres=0.5f;

int switchstate[2] = {0, 0};
//switchstate[0] = Low side
//switchstate[1] = High side



/* main */
main(){
    /* Initialize system */
    InitSysCtrl();                              // [F2837xD_SysCtrl.c]
    /* Initialize GPIO */
    //InitGpio();                                 // [F2837xD_Gpio.c]
    /* Disable CPU interrupts*/
    DINT;
    /* Initialize PIE block to default (all interrupts disabled and flags cleared)*/
    InitPieCtrl();
    /*Disable CPU interrupts and clear all flags*/
    IER = 0x0000;
    IFR = 0x0000;
    /*  Initialize the PIE vector table with pointers to shell ISR's */
    InitPieVectTable();
    /* Map ISR functions*/
    EALLOW;
    PieVectTable.ADCA1_INT = &ADC_EOC_ISR;
    PieVectTable.TIMER0_INT = &CPU_TIMER_0_ISR;
    EDIS;

    /* Initialize CPU period in µs */
    cpuPeriod_us = (1.0/CPUFREQ_MHZ);

    /* Initialize Cpu Timers */
    InitCpuTimers();

    /* Configure Cpu Timer0 to interrupt at specified sampling frequency */
    ConfigCpuTimer(&CpuTimer0, CPUFREQ_MHZ, 1000000.0/TimerInterruptFreq_hz);

    /* Configure ADC */
    ConfigureADC();

    /* Configure GPIO*/
    ConfigureGPIO();


    /* Initialize data logger */
    DCL_initLog(&Buffer2, Current_array, (int)(1000));
    DCL_clearLog(&Buffer2);

    /* Set up ADC channel 2 with Timer 0 trigger*/
    ConfigureADCTimer();

    /* Enable global interrupt and higher priority real-time debug events*/
    /* Group 1 interrupts (for ADCINT1)*/
    IER |= M_INT1;
    /* Enable Timer interrupt */
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    /* Enable global interrupt INTM */
    EINT;
    /* Enable global real-time interrupt DBGM */
    ERTM;

    /* Enable PIE interrupt */
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;


    // START EPWM and ADC

    // START TIMER
    CpuTimer0Regs.TCR.all = 0x4000;



    while(1){
        IdleLoopCount++;
        asm(" NOP");
    }
}

float32_t GenerateSquareReference(long cnt){
    WaveFreq_count = TimerInterruptFreq_hz/WaveFreq_hz;
    if(cnt >= WaveFreq_count){
        Count = 0;
        if(WaveFreq_hz == 0){
            return I_ref_high;
        }
        else{
            if(I_ref==I_ref_low){
                return I_ref_high;
            }
            else{
                return I_ref_low;
            }
        }
    }
    else {
        return I_ref;
    }
}

float32_t GenerateSineReference(long cnt){
    WaveFreq_count = TimerInterruptFreq_hz/WaveFreq_hz;
    phase = phase + 2.0*PI*(float32_t)1.0/WaveFreq_count;
    if(WaveFreq_hz == 0){
        output = I_ref_high;
    }
    else{
        output =  I_ref_low + (I_ref_high-I_ref_low)*(1+sinf(phase))/2;
    }
    return output;
}

void HysteresisControl1(void){
    if(I_res<I_ref-band/2.0f){
        switchstate[0] = 1;
        switchstate[1] = 1;
    }
    if(I_res>I_ref+band/2.0f){
        switchstate[0] = 0;
        switchstate[1] = 1;
    }
}

void SwitchGPIO(void){
    if (switchstate[0] != GpioDataRegs.GPADAT.bit.GPIO0){
        GpioDataRegs.GPADAT.bit.GPIO0 = switchstate[0];
    }
    if (switchstate[1] != GpioDataRegs.GPADAT.bit.GPIO1){
        GpioDataRegs.GPADAT.bit.GPIO1 = switchstate[1];
    }
}

void ConfigureGPIO(void){

    EALLOW;
    //Set GPIO0 as only GPIO0, no multiplexing with e.g. EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;
    //Set GPIO0 as output
    GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;

    //Set GPIO1 as only GPIO0, no multiplexing with e.g. EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
    //Set GPIO1 as output
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
    EDIS;
}

void ConfigureADC(void){

    EALLOW;

    // Clock Prescaler and ADC Configuration
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;             //Prescale 0110 = ADCCLOCK/4.0
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    // Set Pulse Position to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;         //Interrupt at end of conversion, 1 cycle before result latching into result register

    // Power up ADC
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    // Delay 1ms to allow start up
    DELAY_US(1000);

    EDIS;
}

void ConfigureADCTimer(void){

    Uint16 acqps;

    // Determine acquisition window (in SYSCLKS) based on resolution
    if(ADC_RESOLUTION_12BIT == AdcaRegs.ADCCTL2.bit.RESOLUTION)
    {
        acqps = 14; //75ns
    }
    else //resolution is 16-bit
    {
        acqps = 63; //320ns
    }

    // Select channels to converter and end of conversion flag

    EALLOW;

    // Select channel A2 for SOC0
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2;

    // Set acquisition window for SOC0
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;

    // Select Timer 0 TINT0n trigger for SOC0
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 1;

    // EOC0 is trigger for ADCINT1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;

    // Enable ADCINT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;

    // Clear flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    EDIS;
}


// CPU_TIMER_0_ISR - Timer0 ISR
interrupt void CPU_TIMER_0_ISR(void){
    //
    // Start Cpu Timer1 to indicate begin of interrupt
    //
    CpuTimer1Regs.TCR.all = 0x0000;

    // Increase ISR counter
    CPU_TIMER_0_ISR_Count++;

//    //
//    // Add count and write current reference value if WaveFreq_count is reached
//    //
//    Count++;
//
//    //I_ref = GenerateSquareReference(Count);
//    I_ref = GenerateSineReference(Count);

    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

    //
    // Stop Cpu Timer1 to indicate end of interrupt
    //
    CpuTimer1Regs.TCR.all = 0x0010;

    //
    // Calculate interrupt duration in cycles
    //
    interruptCycles = 0xFFFFFFFFUL - CpuTimer1Regs.TIM.all;

    //
    // Calculate interrupt duration in micro seconds
    //
    interruptDuration_us = cpuPeriod_us * interruptCycles;
    CpuTimer1Regs.TCR.all = 0x0030;
    ISRCount_timer++;
}

interrupt void ADC_EOC_ISR(void){
    // Increase ISR counter
    ADC_EOC_ISR_Count++;

    // Clear Flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1=1;

    // read ADC channel
    I_res=((float) (AdcaResultRegs.ADCRESULT0)-I_offset)*I_scale;

    // Add count and write current reference value if WaveFreq_count is reached
    Count++;

    // Generate reference signal
    if(square_enable){
        I_ref = GenerateSquareReference(Count);
    }
    if(sine_enable){
        I_ref = GenerateSineReference(Count);
    }

    HysteresisControl1();
    SwitchGPIO();

    // Fill Buffer, then stop data logging
    if(Buffer2.dptr<Buffer2.lptr){
        DCL_writeLog(&Buffer2, I_res);
    }
    else
    {
        // place break-point here
        asm(" NOP");
    }

    // Acknowledge Interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
