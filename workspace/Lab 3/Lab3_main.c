//#############################################################################
// FILE:   LABstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################

// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "F28379dSerial.h"
#include "LEDPatterns.h"
#include "song.h"
#include "dsp.h"
#include "fpu32/fpu_rfft.h"

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
// The Launchpad's CPU Frequency set to 200 you should not change this value
#define LAUNCHPAD_CPU_FREQUENCY 200


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

uint16_t upCounting = 1; //Lab 2: Create upcounting variable
float motorPower = 0; //create motor power
uint16_t upCountingMotor = 0; // create motor cycle counter

void setEPWM2A(float controleffort){
    if(controleffort > 10){ //lab2: cap from -10 to positive 10
        controleffort = 10;
    }
    else if(controleffort < -10){//lab2: cap from -10 to positive 10
        controleffort = -10;
    }

    EPwm2Regs.CMPA.bit.CMPA = ((controleffort + 10.0) / 20.0) * EPwm2Regs.TBPRD; //Lab2: shift control effort to positive then scale it to 0-2500

}
void setEPWM2B(float controleffort){
    if(controleffort > 10){ //lab2: cap from -10 to positive 10
        controleffort = 10;
    }
    else if(controleffort < -10){//lab2: cap from -10 to positive 10
        controleffort = -10;
    }
    EPwm2Regs.CMPB.bit.CMPB = ((controleffort + 10.0) / 20.0) * EPwm2Regs.TBPRD; //Lab2: shift control effort to positive then scale it to 0-2500
}

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();
	
	// Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

	// Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

	// LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;
	
	// LED2
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

	// LED3
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

	// LED4
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

	// LED5
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

	// LED6
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

	// LED7	
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

	// LED8
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

	// LED9
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

	// LED10
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

	// LED11	
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

	// LED12	
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

	// LED13
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

	// LED14
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;
	
	// LED15
    GPIO_SetupPinMux(159, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(159, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO159 = 1;

	// LED16
    GPIO_SetupPinMux(160, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(160, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPFCLEAR.bit.GPIO160 = 1;

    //WIZNET Reset
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO0 = 1;

    //ESP8266 Reset
    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO1 = 1;

	//SPIRAM  CS  Chip Select
    GPIO_SetupPinMux(19, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(19, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO19 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;
	
    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
	
	//WIZNET  CS  Chip Select
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDSET.bit.GPIO125 = 1;
	
    //PushButton 1
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_INPUT, GPIO_PULLUP);
	
	//Joy Stick Pushbutton
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_INPUT, GPIO_PULLUP);


    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;
    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIB_RX_INT = &RXBINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIB_TX_INT = &TXBINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers


    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 10000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 20000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

	init_serialSCIA(&SerialA,115200);

	EPwm12Regs.TBCTL.bit.CTRMODE = 0; //Lab2: Set counter mode to count up
    EPwm12Regs.TBCTL.bit.FREE_SOFT = 2; //Lab2: Set to free run
    EPwm12Regs.TBCTL.bit.PHSEN = 0; //Lab2: Disable phase loading
    EPwm12Regs.TBCTL.bit.CLKDIV = 0; //Lab2: Set clock div to 1
    EPwm12Regs.TBCTR = 0; //Lab 2: Clear CTR reg
    EPwm12Regs.TBPRD = 2500; //Lab 2: set carrier freq to to 20 khz (50 * 10^6 / (20 * 10^3))
    EPwm12Regs.CMPA.bit.CMPA = 0; //Lab 2: Set duty cycle to 0
    EPwm12Regs.AQCTLA.bit.CAU = 1; //Lab 2: Clear output once ctr reaches compare
    EPwm12Regs.AQCTLA.bit.ZRO = 2; //Lab 2: Set output high once ctr reaches compare
    EPwm12Regs.TBPHS.bit.TBPHS = 0; //Lab 2: just in case
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 5); //Lab 2: Setup GPIO22 to EPWM12A (select 5)

    EPwm2Regs.TBCTL.bit.CTRMODE = 0; //Lab2: Set counter mode to count up
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; //Lab2: Set to free run
    EPwm2Regs.TBCTL.bit.PHSEN = 0; //Lab2: Disable phase loading
    EPwm2Regs.TBCTL.bit.CLKDIV = 0; //Lab2: Set clock div to 1
    EPwm2Regs.TBCTR = 0; //Lab 2: Clear CTR reg
    EPwm2Regs.TBPRD = 2500; //Lab 2: set carrier freq to to 20 khz (50 * 10^6 / (20 * 10^3))
    EPwm2Regs.CMPA.bit.CMPA = 0; //Lab 2: Set duty cycle to 0
    EPwm2Regs.AQCTLA.bit.CAU = 1; //Lab 2: Clear output once ctr reaches compare
    EPwm2Regs.AQCTLA.bit.ZRO = 2; //Lab 2: Set output high once ctr reaches compare

    EPwm2Regs.AQCTLB.bit.CBU = 1; //Lab 2: Clear output once ctr reaches compare
    EPwm2Regs.AQCTLB.bit.ZRO = 2; //Lab 2: Set output high once ctr reaches compare
    EPwm2Regs.CMPB.bit.CMPB = 0; //Lab 2: Set duty cycle to 0

    EPwm2Regs.TBPHS.bit.TBPHS = 0; //Lab 2: just in case
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); //Lab 2: Setup GPIO2 to EPWM2A (select 1)
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); //Lab 2: Setup GPIO3 to EPWM2B (select 1)

    EPwm8Regs.TBCTL.bit.CTRMODE = 0; //Lab2: Set counter mode to count up
    EPwm8Regs.TBCTL.bit.FREE_SOFT = 2; //Lab2: Set to free run
    EPwm8Regs.TBCTL.bit.PHSEN = 0; //Lab2: Disable phase loading
    EPwm8Regs.TBCTL.bit.CLKDIV = 0; //Lab2: Set clock div to 1
    EPwm8Regs.TBCTR = 0; //Lab 2: Clear CTR reg
    EPwm8Regs.TBPRD = 2500; //Lab 2: set carrier freq to to 20 khz (50 * 10^6 / (20 * 10^3))
    EPwm8Regs.CMPA.bit.CMPA = 0; //Lab 2: Set duty cycle to 0
    EPwm8Regs.AQCTLA.bit.CAU = 1; //Lab 2: Clear output once ctr reaches compare
    EPwm8Regs.AQCTLA.bit.ZRO = 2; //Lab 2: Set output high once ctr reaches compare
    EPwm8Regs.TBPHS.bit.TBPHS = 0; //Lab 2: just in case

    EPwm8Regs.AQCTLB.bit.CBU = 1; //Lab 2: Clear output once ctr reaches compare
    EPwm8Regs.AQCTLB.bit.ZRO = 2; //Lab 2: Set output high once ctr reaches compare
    EPwm8Regs.CMPB.bit.CMPB = 0; //Lab 2: Set duty cycle to 0

    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 1); //Lab 2: Setup GPIO14 to EPWM8A (select 1)
    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 1); //Lab 2: Setup GPIO15 to EPWM8B (select 1)

    EPwm9Regs.TBCTL.bit.CTRMODE = 0; //Lab2: Set counter mode to count up
    EPwm9Regs.TBCTL.bit.FREE_SOFT = 2; //Lab2: Set to free run
    EPwm9Regs.TBCTL.bit.PHSEN = 0; //Lab2: Disable phase loading
    EPwm9Regs.TBCTL.bit.CLKDIV = 0; //Lab2: Set clock div to 1
    EPwm9Regs.TBCTR = 0; //Lab 2: Clear CTR reg
    EPwm9Regs.TBPRD = 2500; //Lab 2: set carrier freq to to 20 khz (50 * 10^6 / (20 * 10^3))
    EPwm9Regs.CMPA.bit.CMPA = 0; //Lab 2: Set duty cycle to 0
    EPwm9Regs.AQCTLA.bit.CAU = 1; //Lab 2: Clear output once ctr reaches compare
    EPwm9Regs.AQCTLA.bit.ZRO = 2; //Lab 2: Set output high once ctr reaches compare
    EPwm9Regs.TBPHS.bit.TBPHS = 0; //Lab 2: just in case
    GPIO_SetupPinMux(16, GPIO_MUX_CPU1, 5); //Lab 2: Setup GPIO16 to EPWM9A (select 5)

    //Lab 2: Disable internal pullups on EPWM
    EALLOW; // Below are protected registers
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1; // For EPWM2A
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1; // For EPWM2B
    GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1; // For EPWM8A
    GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1; // For EPWM8B
    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1; // For EPWM9A
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1; // For EPWM12A
    EDIS;


    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
	// Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;
	
	init_serialSCIB(&SerialB,115200);
	init_serialSCIC(&SerialC,115200);
	init_serialSCID(&SerialD,115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
			serial_printf(&SerialA,"Num Timer2:%ld Num SerialRX: %ld\r\n",CpuTimer2.InterruptCount,numRXA);
            UARTPrint = 0;
        }
    }
}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
	// making it lower priority than all other Hardware interrupts.  
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts
	
	
	
    // Insert SWI ISR Code here.......
	
	
    numSWIcalls++;
    
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if ((numTimer0calls%50) == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }

    if ((numTimer0calls%25) == 0) {
        //displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF) {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    if ((numTimer0calls%50) == 0) {
		// Blink LaunchPad Red LED
		GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
		
    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{
	// Blink LaunchPad Blue LED
    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    if( EPwm12Regs.CMPA.bit.CMPA == EPwm12Regs.TBPRD){  //Lab 2: Check to see if CMPA has reached the period
        upCounting = 0; //Lab 2: set to count down after hitting
    }
    else if (EPwm12Regs.CMPA.bit.CMPA == 0){ // Lab 2: Check to see if it hit 0 while downcounting
        upCounting = 1; //Lab 2: Swap to upcounting
    }

    if(upCounting){ //Lab 2: If upcounting right now
        EPwm12Regs.CMPA.bit.CMPA++; //Lab 2: increment CMPA

    }
    else{ //Lab 2: Otherwise (if downcounting)
        EPwm12Regs.CMPA.bit.CMPA--; ///Lab 2: Decrement CMPA
    }


    if(motorPower >= 10){  //Lab 2: Check to see if CMPA has reached the period
        upCountingMotor = 0; //Lab 2: set to count down after hitting
    }
    else if (motorPower <= -10){ // Lab 2: Check to see if it hit -10 while downcounting
        upCountingMotor = 1; //Lab 2: Swap to upcounting
    }

    if(upCountingMotor){ //Lab 2: If upcounting right now
        motorPower+=.005; //interrupt is called every 1ms, every 5s go from 0 to 10 lab 2
    }
    else{ //Lab 2: Otherwise (if downcounting)
        motorPower-=.005;//interrupt is called every 1ms, every 5s go from 0 to 10 lab 2
    }
    setEPWM2A(motorPower);

    setEPWM2B(motorPower);

    CpuTimer2.InterruptCount++;
	
	if ((CpuTimer2.InterruptCount % 10) == 0) {
		UARTPrint = 1;
	}
}

