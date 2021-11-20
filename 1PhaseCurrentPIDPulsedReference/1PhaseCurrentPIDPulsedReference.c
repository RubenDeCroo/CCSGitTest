/*
 * This program features a PID control of the current in a winding of the machine by
 * controlling the duty cycle of the asymmetric bridge converter.
 * Current is sensed on ADCINA2 (Pin 29).
 * The EPWM outputs are EPWM1A (low side) and EPWM1B (high side).
 *
 * The EPWM is configured to trigger an ADC SOC at the midpoint of
 * the pulse (to measure the average current with 1 measurement).
 * The default PWM frequency is 10 kHz which corresponds with a TBPRD = 2500.
 * The frequency can be controlled by altering this value.
 * An ADC EOC interrupt is implemented with an ISR containing the PID controller.
 * The PID controller calculates a new control action based on the ADC result which is
 * converted to a duty cycle.
 * The duty cycle of the EPWM module is then updated.
 *
 * The reference current I_ref is initialized to 1 A and can be changed in the expressions window.
 */



#include "F2837xD_device.h"
//#include "F2837xD_Examples.h";
//#include "F2837xD_GlobalPrototypes.h";
#include "F28x_Project.h"
#include "DCLF32.h"
#include "DCL_fdlog.h"
#include "F2837xD_Adc_defines.h"


//
// Defines
//
#define CPUFREQ_MHZ          200
#define DATA_LENGTH          1000;



// Duty cycle results
#pragma DATA_SECTION(Duty_array, "DataLogSection");
float Duty_array[(int)(1000)];                          // float Duty_array[DATA_LENGTH] not working?
FDLOG Buffer1 = FDLOG_DEFAULTS;

// Current results
#pragma DATA_SECTION(Current_array, "DataLogSection");
float Current_array[(int)(1000)];                          // float Duty_array[DATA_LENGTH] not working?
FDLOG Buffer2 = FDLOG_DEFAULTS;





// Prototype functions and ISR's
void ConfigureADC(void);
void ConfigureEPWM(void);
void ConfigureGPIO(void);
void SetupADCEPWM();


extern interrupt void control_ISR(void);


// Global variables
// With current linker configuration, real-time controllable variables need to be initialized
long IdleLoopCount = 0;
long ISRCount = 0;
// Initial value for control reference
float I_ref = 0;
// Input voltage: 312.5 mV/A -> 10.56 A = 4096 for old setup
//TMCS3: 200mV/A -> 16.5 = 4096, EXP: 335 mV offset = 475 -> 2772 mV = 4096 = 13.86

float I_scale = 15/4096.0f;
float I_offset = 470.0;
float I_res;
float lk;
float Duty;
DCL_PID pid1 = PID_DEFAULTS;
float CompA;
float CompB;
float upperlim = 1.0f;
float lowerlim = 0.0f;
unsigned int clampactive;


//
// From '1PhaseCurrentReference.c'
//

//
// Globals
//
Uint32 TimerInterruptFreq_hz = 100000;
float cpuPeriod_us = 0;
Uint32 interruptCycles = 0;
float interruptDuration_us = 0;
float samplingPeriod_us = 0;

//Current Reference;
float I_ref_low = 0;
float I_ref_high = 1.0f;
Uint32 ISRCount_timer = 0;
float WaveFreq_hz = 1;
float WaveFreq_count = 0;
Uint32 Count = 0;

interrupt void cpu_timer0_isr(void);

//
// End of  '1PhaseCurrentReference.c'
//






/* main */
main()
{
    /* Initialize system */
    InitSysCtrl();                              // [F2837xD_SysCtrl.c]
    /* Initialize GPIO */
    InitGpio();                                 // [F2837xD_Gpio.c]
    /* Initialize GPIO for EPWM1 */
    InitEPwm1Gpio();
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
    PieVectTable.ADCA1_INT = &control_ISR;
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    EDIS;

    /* Initialize CPU period in µs */
    cpuPeriod_us = (1.0/CPUFREQ_MHZ);

    /* Initialize Cpu Timers */
    InitCpuTimers();

    /* Configure Cpu Timer0 to interrupt at specified sampling frequency */
    ConfigCpuTimer(&CpuTimer0, CPUFREQ_MHZ, 1000000.0/TimerInterruptFreq_hz);

    /* Configure ADC */
    ConfigureADC();

    /* Initialize controller variables */
    pid1.Kp = 0.01f;
    pid1.Ki = 0.001f;
    pid1.Kd = 0.0f;
    pid1.Kr = 1.0f;
    pid1.c1 = 1.0/3.0*1000000.0f; // 2/(sample period + 2*tau) = 2/(0.0001 + 2*0.00001)=+/- 1/2 * 100 000
    pid1.c2 = 2.0/3.0f; // (T-2*tau)/(T+2*tau)
    pid1.d2 = 0.0f;
    pid1.d3 = 0.0f;
    pid1.i10 = 0.0f;
    pid1.i14 = 1.0f;
    pid1.Umax = 1.0f;
    pid1.Umin = 0.0f;

    /* Control not saturated */
    lk = 1.0f;


    /* Initialize data logger */
    DCL_initLog(&Buffer1, Duty_array, (int)(1000));
    DCL_clearLog(&Buffer1);

    /* Initialize data logger */
    DCL_initLog(&Buffer2, Current_array, (int)(1000));
    DCL_clearLog(&Buffer2);


    /* Disable EPWM1 */
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC=0;
    /* Configure EPWM */
    ConfigureEPWM();
    /* Enable EPWM1 */
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC=1;

    /* Set up ADC channel 2 with EPWM trigger*/
    SetupADCEPWM();


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


    /* Start EPWM1 in count up mode */
    EALLOW;
    EPwm1Regs.TBCTL.bit.CTRMODE=1;
    EDIS;



    // Enable SOC on B group:
    EPwm1Regs.ETSEL.bit.SOCBEN = 1;
    // Unfreeze and up count mode:
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;

    // START TIMER
    CpuTimer0Regs.TCR.all = 0x4000;

    while(1){
        IdleLoopCount++;
        asm(" NOP");
    }
}



void ConfigureEPWM(void){

    //EPWMCLK = SYSCLKOUT/2 = 200 MHz/2 = 100 MHz by default

    EALLOW;

    // ASSUMES EPWM CLOCK IS ENABLED!//


    /* Configure EPWM to trigger SOC B*/
    // Disable SOC B
    EPwm1Regs.ETSEL.bit.SOCBEN = 0;

    // ADC SOC B at CMPB up count
    EPwm1Regs.ETSEL.bit.SOCBSEL = 6;

    // ADC on first event
    EPwm1Regs.ETPS.bit.SOCBPRD = 1;

    /* Configure Time-base  */
    // Clock Division = 4 -> 100MHz/4 = 25 MHz clock
    EPwm1Regs.TBCTL.bit.CLKDIV = 2;

    // Free run on emulation suspend
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 2;

    // TBCLK = SYSCLKOUT
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;

    // Freeze counter
    EPwm1Regs.TBCTL.bit.CTRMODE = 3;

    // Disable SYNCOUT signal
    EPwm1Regs.TBCTL.bit.SYNCOSEL = 3;

    // Immediate load
    EPwm1Regs.TBCTL.bit.PRDLD = 1;

    //Disable phase loading
    EPwm1Regs.TBCTL.bit.PHSEN = 0;

    // Phase = 0
    EPwm1Regs.TBPHS.all = 0;

    // Initialize time-base counter value to 0
    EPwm1Regs.TBCTR = 0;

    // Set counter period to 2500 -> 10kHz at 25MHz clock;
    EPwm1Regs.TBPRD = 0x09C4;

    // Initialize Compare A value
    EPwm1Regs.CMPA.bit.CMPA = EPwm1Regs.TBPRD/2;

    // Initialize Compare B value
    EPwm1Regs.CMPB.bit.CMPB = EPwm1Regs.TBPRD/2;

    // Enable Shadow Mode CMPA
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = 1;

    // Enable Shadow Mode CMPB
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = 1;

    // Load on CTR = 0 CMPA
    EPwm1Regs.CMPCTL.bit.LOADAMODE = 2;

    // Load on CTR = 0 CMPB
    EPwm1Regs.CMPCTL.bit.LOADBMODE = 2;

    //Set Low on CMPA match on upcount
    EPwm1Regs.AQCTLA.bit.CAU = 1;

    //Set High on zero match
    EPwm1Regs.AQCTLA.bit.ZRO = 2;

    //Set Low on CMPB match on upcount
    EPwm1Regs.AQCTLB.bit.CBU = 1;

    //Set High on zero match
    EPwm1Regs.AQCTLB.bit.ZRO = 2;

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

void SetupADCEPWM(){

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

    // Set acquisition window
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = acqps;

    // Select EPWM1 SOCB trigger
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 6;

    // EOC0 is trigger for ADCINT1
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0;

    // Enable ADCINT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;

    // Clear flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;

    EDIS;
}


//
// cpu_timer0_isr - Timer0 ISR
//
interrupt void cpu_timer0_isr(void)
{
    //
    // Start Cpu Timer1 to indicate begin of interrupt
    //
    CpuTimer1Regs.TCR.all = 0x0000;

    //
    // Add count and write current reference value if WaveFreq_count is reached
    //
    Count++;
    if(WaveFreq_hz != 0){
        WaveFreq_count = TimerInterruptFreq_hz/WaveFreq_hz;
    }
    if(Count >= WaveFreq_count){
        if(I_ref==I_ref_low){
            I_ref = I_ref_high;
        }
        else{
            I_ref = I_ref_low;
        }
        Count = 0;
    }
    if(WaveFreq_hz == 0){
        I_ref = I_ref_high;
    }

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

interrupt void control_ISR(void){

    ISRCount++;
    // Clear Flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1=1;

    // read ADC channel
    I_res=((float) (AdcaResultRegs.ADCRESULT0)-I_offset)*I_scale;

    // run PID controller
    Duty = DCL_runPID_C4(&pid1, I_ref, I_res, lk);


    // external clamp for anti-windup reset
    clampactive = DCL_runClamp_C1(&Duty, upperlim, lowerlim);


    // write to CMPA and CMPB
    CompA = Duty * (float) EPwm1Regs.TBPRD;
    //CompB = (1.0f+Pid_out)/2.0f * (float) EPwm1Regs.TBPRD; // Mid pulse measurement
    CompB = Duty/2.0f * (float) EPwm1Regs.TBPRD; // Mid pulse measurement
    EPwm1Regs.CMPA.bit.CMPA = (Uint16) CompA;
    EPwm1Regs.CMPB.bit.CMPB = (Uint16) CompB;


    // Fill Buffer, then stop data logging
    if(Buffer1.dptr<Buffer1.lptr){
        DCL_writeLog(&Buffer1, Duty);
    }
    else
    {
        // place break-point here
        asm(" NOP");
    }

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
