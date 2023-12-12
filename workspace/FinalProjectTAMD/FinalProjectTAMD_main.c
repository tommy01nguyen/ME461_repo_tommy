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
__interrupt void SPIB_isr(void);
__interrupt void ADCA_ISR(void);

//amp dbc lab 6 predefinitions:
float readEncRight(void);
float readEncLeft(void);
void init_eQEPs(void);
void calculateRobotPose(void);

void setupSpib(void);

void setEPWM2A(float controleffort);
void setEPWM2B(float controleffort);

//robot comm
float printLV3 = 0;
float printLV4 = 0;
float printLV5 = 0;
float printLV6 = 0;
float printLV7 = 0;
float printLV8 = 0;
float x = 0;
float y = 0;
float bearing = 0;
extern uint16_t NewLVData;
extern float fromLVvalues[LVNUM_TOFROM_FLOATS];
extern LVSendFloats_t DataToLabView;
extern char LVsenddata[LVNUM_TOFROM_FLOATS * 4 + 2];
extern uint16_t newLinuxCommands;
extern float LinuxCommands[CMDNUM_FROM_FLOATS];

int32_t numADCD1calls = 0;

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numTimer1calls = 0;
uint32_t numSWIcalls = 0;
extern uint32_t numRXA;
uint16_t UARTPrint = 0;
uint16_t LEDdisplaynum = 0;

//amp dbc variables for what we send to dan chip
int16_t motor_1_effort = 0;
int16_t motor_2_effort = 0;

//AMP DBC get the data from the register so we can read it
int16_t ADC1val = 0;
int16_t ADC2val = 0;

//amp dbc joystick vars
int16_t ADCIND0_raw = 0; // 0-4095
int16_t ADCIND1_raw = 0;
float ADCIND0_volt = 0; //voltage

float IR_2 = 0; //amp dbc global variables to print the left and right IR sensor Data
float IR_1 = 0;

int16_t countUp = 1;

//amp dbc variables for IMU data
int16_t accel_x_raw = 0;
int16_t accel_y_raw = 0;
int16_t accel_z_raw = 0;

int16_t gyro_x_raw = 0;
int16_t gyro_y_raw = 0;
int16_t gyro_z_raw = 0;

float accel_x = 0.0;
float accel_y = 0.0;
float accel_z = 0.0;

float gyro_x = 0.0;
float gyro_y = 0.0;
float gyro_z = 0.0;

float leftWheel = 0; //radians
float rightWheel = 0;

float leftDistance = 0; //feet
float rightDistance = 0;
float prev_leftDistance = 0;
float prev_rightDistance = 0;
float leftVel = 0;
float rightVel = 0;

float uLeft = 5;
float uRight = 5;
float WhlDiff = 0;

//for PI controller
float ekLeft = 0;
float ekRight = 0;
float ekLeft_1 = 0; //previous (1 ago) amp dbc
float ekRight_1 = 0; //previoius

float IkLeft = 0;
float IkRight = 0;

float IkLeft_1 = 0; //previous (1 ago) amp dbc
float IkRight_1 = 0; //previous (1 ago)

float ukLeft = 0;
float ukRight = 0;

float Kp = 3;
float Ki = 25;

float Vref = 0; //ft/sec

//turn stuff
float KPturn = 3;
float turn = 0; //positive Pturn is turning left amp dbc
float eturn = 0;

//dead reckoning variables amp dbc
float W = 0.56759;
float Rwh = 0.19460;
float omegaL = 0;
float omegaR = 0;
float thetaAvg = 0;
float omegaAvg = 0;
float x_dot = 0;
float y_dot = 0;
float x_dotPrev = 0;
float y_dotPrev = 0;

//KALMAN FILTER
// Needed global Variables
float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;
float gyrox_offset = 0;
float gyroy_offset = 0;
float gyroz_offset = 0;
float accelzBalancePoint = -.60; // .62
int16 IMU_data[9];
uint16_t temp = 0;
int16_t doneCal = 0;
float tilt_value = 0;
float tilt_array[4] = { 0, 0, 0, 0 };
float gyro_value = 0;
float gyro_array[4] = { 0, 0, 0, 0 };
float LeftWheel = 0;
float RightWheel = 0;
float LeftWheelArray[4] = { 0, 0, 0, 0 };
float RightWheelArray[4] = { 0, 0, 0, 0 };
// Kalman Filter vars
float T = 0.001; //sample rate, 1ms
float Q = 0.01; // made global to enable changing in runtime
float R = 25000; //50000;
float kalman_tilt = 0;
float kalman_P = 22.365;
int16_t SpibNumCalls = -1;
float pred_P = 0;
float kalman_K = 0;
int32_t timecount = 0;
int16_t calibration_state = 0;
int32_t calibration_count = 0;

float linearPos = 0;
float speedRef = 0;
float linear_move_command_vel = 0;

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
    EALLOW;
    // This is needed to write to EALLOW protected registers
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
    PieVectTable.SPIB_RX_INT = &SPIB_isr;
    PieVectTable.ADCA1_INT = &ADCA_ISR;
    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;
    // This is needed to disable write to EALLOW protected registers

    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every given period:
    // 200MHz CPU Freq,                       Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, LAUNCHPAD_CPU_FREQUENCY, 1000);
    ConfigCpuTimer(&CpuTimer1, LAUNCHPAD_CPU_FREQUENCY, 4000);
    ConfigCpuTimer(&CpuTimer2, LAUNCHPAD_CPU_FREQUENCY, 10000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    init_serialSCIA(&SerialA, 115200);

    EALLOW;
    EPwm5Regs.ETSEL.bit.SOCAEN = 0; // Disable SOC on A group
    EPwm5Regs.TBCTL.bit.CTRMODE = 3; // freeze counter
    EPwm5Regs.ETSEL.bit.SOCASEL = 2; // Select Event when counter equal to PRD
    EPwm5Regs.ETPS.bit.SOCAPRD = 1; // Generate pulse on 1st event (“pulse” is the same as “trigger”)
    EPwm5Regs.TBCTR = 0x0; // Clear counter
    EPwm5Regs.TBPHS.bit.TBPHS = 0x0000; // Phase is 0
    EPwm5Regs.TBCTL.bit.PHSEN = 0; // Disable phase loading
    EPwm5Regs.TBCTL.bit.CLKDIV = 0; // divide by 1 50Mhz Clock
    EPwm5Regs.TBPRD = 50000; // Set Period to 1ms sample. mimimimimmi Input clock is 50MHz.
    // Notice here that we are not setting CMPA or CMPB because we are not using the PWM signal
    EPwm5Regs.ETSEL.bit.SOCAEN = 1; //enable SOCA
    EPwm5Regs.TBCTL.bit.CTRMODE = 0; //unfreeze, and enter up count mode
    EDIS;

    EALLOW;
    //write configurations for all ADCs ADCA, ADCB, ADCC, ADCD
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE); //read calibration settings
    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    //power up the ADCs
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    //Select the channels to convert and end of conversion flag
    //Many statements commented out, To be used when using ADCA or ADCB
    //amp dbc ADCA
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; // set SOC0 to convert pin A2
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 13; //0Dh // EPWM5 ADCSOCA will trigger SOC0
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; //set SOC1 to convert pin A3
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 13; // EPWM5 ADCSOCA will trigger SOC1
    //AdcaRegs.ADCSOC2CTL.bit.CHSEL = ???; //set SOC2 to convert pin D2
    //AdcaRegs.ADCSOC2CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC2
    //AdcaRegs.ADCSOC3CTL.bit.CHSEL = ???; //set SOC3 to convert pin D3
    //AdcaRegs.ADCSOC3CTL.bit.ACQPS = 99; //sample window is acqps + 1 SYSCLK cycles = 500ns
    //AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = ???; // EPWM5 ADCSOCA will trigger SOC3
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 1; //set to SOC1, the last converted, and it will set INT1  flag ADCD1
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared
    EDIS;

    setupSpib();
    init_eQEPs(); //amp dbc lab 6 call inti eQEPS

    // amp dbc PWM2A setup
    EPwm2Regs.TBCTL.bit.FREE_SOFT = 0x3; // 11 in bit
    EPwm2Regs.TBCTL.bit.CTRMODE = 0x0; //00 in bit
    EPwm2Regs.TBCTL.bit.PHSEN = 0x0; //0 in bit
    EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0x0; //000 in bit

    EPwm2Regs.TBCTR = 0x0; //000 in bit
    EPwm2Regs.TBPRD = 2500; //(1/20000)/(1/50000000)
    EPwm2Regs.CMPA.bit.CMPA = 0;
    EPwm2Regs.CMPB.bit.CMPB = 0;

    EPwm2Regs.AQCTLA.bit.CAU = 1; //clearwhen TBCTR = CMPA on up count
    EPwm2Regs.AQCTLB.bit.CBU = 1; //clearwhen TBCTR = CMPB on up count

    EPwm2Regs.AQCTLA.bit.ZRO = 2; //set when TBCTR = 0
    EPwm2Regs.AQCTLB.bit.ZRO = 2; //set when TBCTR = 0

    EPwm2Regs.TBPHS.bit.TBPHS = 0; //set phase to 0

    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 1); //GPIO PinName, CPU, Mux Index
    GPIO_SetupPinMux(3, GPIO_MUX_CPU1, 1); //GPIO PinName, CPU, Mux Index
    //set gpio 2 to EPWM2A
    //set gpio 3 to EPWM2B

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    //amp dbc for the SPI group 6
    IER |= M_INT6;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    //AMP DBC for SPI enable Group 6 interrupt 3
    PieCtrlRegs.PIEIER6.bit.INTx3 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1; //for ADCA

    //init_serialSCIB(&SerialB,115200);
    init_serialSCIC(&SerialC, 115200);
    init_serialSCID(&SerialD, 115200);
    // Enable global Interrupts and higher priority real-time debug events
    EINT;
    // Enable Global interrupt INTM
    ERTM;
    // Enable Global realtime interrupt DBGM


    while (GpioDataRegs.GPADAT.bit.GPIO4){ //WAIT for stuff to happen

    }

    // IDLE loop. Just sit and loop forever (optional):
    float initial_position = linearPos;
    while (1)
    {
        move_linear( initial_position + 2 - linearPos );
        speedRef = linear_move_command_vel;

        if (UARTPrint == 1)
        {
//            serial_printf(&SerialA, "accel_x:%.3f accel_y: %.3f accel_z:%.3f gyro_x:%.3f gyro_y:%.3f gyro_z:%.3f \r \n",
//                          accel_x, accel_y,accel_z,gyro_x,gyro_y,gyro_z);
            serial_printf(&SerialA,
                          "POSITION: LeftWheel: %.3f  RightWheel: %.3f\r\n",
                          leftWheel, rightWheel);
            //serial_printf(&SerialA, "VELOCITY: LeftWheel: %.3f  RightWheel: %.3f\r\n", leftVel, rightVel);

//            serial_printf(&SerialA,
//                          "bearing: %.3f  x_pos: %.3f y_pos: %.3f\r\n", bearing,
//                          x, y);

            serial_printf(&SerialA,
                          "IR_2_VOLTAGE:%f IR_1_VOLTAGE:%f ACCEL_Z:%f GYRO_X:%f\r\n",
                          IR_2, IR_1, accel_z, gyro_x);

            UARTPrint = 0;
        }
    }
}

// Lab 7: AMP DBC TN MZ: intiialize PID values for balancing
float vel_Right = 0;
float vel_Left = 0;
float vel_Left_prev = 0;
float leftWheel_prev = 0;
float vel_Right_prev = 0;
float rightWheel_prev = 0;

//float gyrorate = 0;
float gyrorate_prev = 0;
float gyrorate_dot = 0;
float gyrorate_dot_prev = 0;

float ubal = 0;
float K1 = -60.0;
float K2 = -4.5;
float K3 = -1.1;
float K4 = -0.1;

float PrevWhlDiff = 0;
float PrevVelWhlDiff = 0;
float turnref = 0;
float errorDiff = 0;
float SumErrorDiff = 0;
float uTurn = 0;
float PrevSumErrorDiff = 0;
float PrevErrorDiff = 0;

// Lab 7: AMP DBC TN MZ: declare turn PID values
float turnKp = 6;
float turnKi = 20;
float turnKd = 0.08;
float VelWhlDiff = 0;
float turnRate = 0;
float prevTurnRate = 0;
float prevTurnref = 0;

// Lab 7: AMP DBC TN MZ: declare speed PID Values
float speedKp = 0.35;
float speedKi = 1.5;

float IK_eSpeed = 0;
float eSpeedPrev = 0;
float eSpeed = 0;
float ForwardBackwardCommand = 0;
float PrevIK_eSpeed = 0;

void labview_comms(){
    if (NewLVData == 1)
       {
           NewLVData = 0;
           speedRef = fromLVvalues[0]; // Lab 7: AMP DBC TN MZ: receiving speed value from labview
           turnRate = fromLVvalues[1]; // Lab 7: AMP DBC TN MZ: receiving turn speed value from labview
           printLV3 = fromLVvalues[2];
           printLV4 = fromLVvalues[3];
           printLV5 = fromLVvalues[4];
           printLV6 = fromLVvalues[5];
           printLV7 = fromLVvalues[6];
           printLV8 = fromLVvalues[7];
       }
       if ((numSWIcalls % 62) == 0) // Lab 7: AMP DBC TN MZ: sending robot position and bearing to labview
       { // change to the counter variable of you selected 4ms. timer
           DataToLabView.floatData[0] = x;
           DataToLabView.floatData[1] = y;
           DataToLabView.floatData[2] = bearing;
           DataToLabView.floatData[3] = 2.0 * ((float) numSWIcalls) * .001;
           DataToLabView.floatData[4] = 3.0 * ((float) numSWIcalls) * .001;
           DataToLabView.floatData[5] = (float) numSWIcalls;
           DataToLabView.floatData[6] = (float) numSWIcalls * 4.0;
           DataToLabView.floatData[7] = (float) numSWIcalls * 5.0;
           LVsenddata[0] = '*'; // header for LVdata
           LVsenddata[1] = '$';
           for (int i = 0; i < LVNUM_TOFROM_FLOATS * 4; i++)
           {
               if (i % 2 == 0)
               {
                   LVsenddata[i + 2] = DataToLabView.rawData[i / 2] & 0xFF;
               }
               else
               {
                   LVsenddata[i + 2] = (DataToLabView.rawData[i / 2] >> 8) & 0xFF;
               }
           }
           serial_sendSCID(&SerialD, LVsenddata, 4 * LVNUM_TOFROM_FLOATS + 2);
       }

}

float ubal_prev = 0;
float ubal_next = 0;
// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void)
{
    // Lab 7: AMP DBC TN MZ: Labview communication
    labview_comms();
    leftWheel = readEncLeft(); // Lab 7: AMP DBC TN MZ: get wheel radians per second
    rightWheel = readEncRight();

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");
    // Wait one cycle
    EINT;
    // Clear INTM to enable interrupts

    // Insert SWI ISR Code here.......

    vel_Left = 0.6 * vel_Left_prev + 100.0 * (LeftWheel - leftWheel_prev);// Lab 7: AMP DBC TN MZ:continous transfer function to estimate velocities and gyro with less noise
    vel_Right = 0.6 * vel_Right_prev + 100.0 * (RightWheel - rightWheel_prev);

    gyrorate_dot = 0.6 * gyrorate_dot_prev + 100.0 * (gyro_value - gyrorate_prev);

    WhlDiff = leftWheel - rightWheel; // Lab 7: AMP DBC TN MZ:find wheel difference

    VelWhlDiff = 0.333 * PrevVelWhlDiff + 166.667 * (WhlDiff - PrevWhlDiff);// Lab 7: AMP DBC TN MZ:continous transfer function testimate wheel diff velocity

    turnref += .004 * (turnRate + prevTurnRate) / 2.0; // Lab 7: AMP DBC TN MZ:update turn reference based off turn speed

    errorDiff = turnref - WhlDiff; // Lab 7: AMP DBC TN MZ: find error in turning
    SumErrorDiff += (errorDiff + PrevErrorDiff) / 2.0 * .004;
    uTurn = turnKp * errorDiff + turnKi * SumErrorDiff - turnKd * VelWhlDiff; // Lab 7: AMP DBC TN MZ: PID for turn effort

    if (fabs(uTurn) > 3)
    { //prevent integral windup
        SumErrorDiff = PrevSumErrorDiff;
    }

    //saturate integral windup
    if (uTurn > 4)
    {
        uTurn = 4;
    }
    else if (uTurn < -4)
    {
        uTurn = -4;
    }

    //forward backward pid

    eSpeed = speedRef - ((vel_Left + vel_Right) / 2.0);// Lab 7: AMP DBC TN MZ: calculate speed error

    IK_eSpeed += (eSpeed + eSpeedPrev) / 2.0 * .004; // Lab 7: AMP DBC TN MZ: caclulate intergral error

    ForwardBackwardCommand = speedKp * eSpeed + speedKi * IK_eSpeed; // Lab 7: AMP DBC TN MZ: calculate forward backward PID

    if (fabs(IK_eSpeed) > 3)
    { //prevent integral windup
        IK_eSpeed = PrevIK_eSpeed;
    }

    //saturate integral windup of forward backward command
    if (ForwardBackwardCommand > 4)
    {
        ForwardBackwardCommand = 4;
    }
    else if (ForwardBackwardCommand < -4)
    {
        ForwardBackwardCommand = -4;
    }

    //**
    ubal_prev = ubal_next;

    ubal_next = -K1 * 1.75 * tilt_value - K2 * 1.75 * gyro_value
            - K3 * 1.8 * (vel_Left + vel_Right) / 2.0 - K4 * 1.8 * gyrorate_dot; // Lab 7: AMP DBC TN MZ: balancing effort

    float alpha = (2*PI*1*.004)/(1+1*2*PI*.004);

    ubal = (alpha)*ubal_next + (1-alpha)*ubal_prev;

    //ubal *= 2;


    uRight = ubal / 2.0 - uTurn - ForwardBackwardCommand; // Lab 7: AMP DBC TN MZ: effort to each wheel
    uLeft = ubal / 2.0 + uTurn - ForwardBackwardCommand;

    setEPWM2A(uRight); // RIGHT MOTOR
    setEPWM2B(-uLeft); //LEFT MOTOR

    rightWheel_prev = RightWheel; // Lab 7: AMP DBC TN MZ: set previous values
    leftWheel_prev = LeftWheel;
    vel_Right_prev = vel_Right;
    vel_Left_prev = vel_Left;

    gyrorate_dot_prev = gyrorate_dot;
    gyrorate_prev = gyro_value;

    PrevWhlDiff = WhlDiff;
    PrevVelWhlDiff = VelWhlDiff;
    PrevErrorDiff = errorDiff;
    PrevSumErrorDiff = SumErrorDiff;
    prevTurnref = turnref;
    prevTurnRate = turnRate;

    eSpeedPrev = eSpeed;
    PrevIK_eSpeed = IK_eSpeed;

    numSWIcalls++;
    DINT;

}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    //    numTimer0calls++;
    //    if (motor_1_effort >= 3000) {
    //        countUp = 0;
    //    }
    //    if (motor_1_effort <= 0) {
    //        countUp = 1;
    //    }
    //    if (countUp) {
    //        motor_1_effort += 10;
    //        motor_2_effort += 10;
    //    } else {
    //        motor_1_effort -= 10;
    //        motor_2_effort -= 10;
    //    }
    //
    //    //AMP DBC send the data over SPI
    //    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
    //    SpibRegs.SPIFFRX.bit.RXFFIL = 3; // Issue the SPIB_RX_INT when two values are in the RX FIFO
    //    SpibRegs.SPITXBUF = 0x00DA; //amp dbc start command for Dan Chip
    //    SpibRegs.SPITXBUF = motor_1_effort; //amp dbc now, queue up the values we want to send
    //    SpibRegs.SPITXBUF = motor_2_effort;

//    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // slave select low
//    SpibRegs.SPIFFRX.bit.RXFFIL = 8; // Issue the SPIB_RX_INT when 8 values are in the RX FIFO
//    SpibRegs.SPITXBUF = 0xBA00; // AMP dbc starting with init_status
//    SpibRegs.SPITXBUF = 0x0000; //sending garbage to receive next 7 16-bit vals
//    SpibRegs.SPITXBUF = 0x0000;
//    SpibRegs.SPITXBUF = 0x0000;
//    SpibRegs.SPITXBUF = 0x0000;
//    SpibRegs.SPITXBUF = 0x0000;
//    SpibRegs.SPITXBUF = 0x0000;
//    SpibRegs.SPITXBUF = 0x0000;

    if ((numTimer0calls % 25) == 0)
    {
        displayLEDletter(LEDdisplaynum);
        LEDdisplaynum++;
        if (LEDdisplaynum == 0xFFFF)
        {  // prevent roll over exception
            LEDdisplaynum = 0;
        }
    }

    if ((numTimer0calls % 50) == 0)
    {
        // Blink LaunchPad Red LED
//        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    numTimer1calls++;

    //WHEEL MOTORS::
    leftWheel = readEncLeft();
    rightWheel = readEncRight();

    leftDistance = leftWheel / 5.061; //feet traveled
    rightDistance = rightWheel / 5.061;

    linearPos = (leftDistance + rightDistance)/2.0;

    //velocity in ft/sec
    leftVel = (leftDistance - prev_leftDistance) / 0.004;
    rightVel = (rightDistance - prev_rightDistance) / 0.004;

    //amp dbc radians per second for calculating bearing and X and Y pos
    omegaL = leftVel * 5.061; //angular velocity of wheels in radians/sec
    omegaR = rightVel * 5.061;
    thetaAvg = (leftWheel + rightWheel) / 2.0;
    omegaAvg = (omegaL + omegaR) / 2.0;
    calculateRobotPose();

    //amp dbc sets the old distance in ft for velocity calculation
    prev_leftDistance = leftDistance;
    prev_rightDistance = rightDistance;

    //set EPWM to command motor velocity amp dbc
//    setEPWM2A(uRight); // RIGHT MOTOR
//    setEPWM2B(-uLeft); //LEFT MOTOR

    //amp dbc PI CONTROLLER
    eturn = turn + (leftVel - rightVel); // turning error

    ekLeft = Vref - leftVel - KPturn * eturn;
    ekRight = Vref - rightVel + KPturn * eturn; //note the plus sign in this line over the previous, means that they trun in opposite directions amp dbc

    //Ik calcs amp dbc

    IkLeft = IkLeft_1 + 0.004 * ((ekLeft + ekLeft_1) / 2);
    IkRight = IkRight_1 + 0.004 * ((ekRight + ekRight_1) / 2);

    if (uLeft > 10 || uLeft < -10)
    { // stops integral wind up
        IkLeft = IkLeft_1;
    }
    if (uRight > 10 || uRight < -10)
    { // stops integral wind up
        IkRight = IkRight_1;
    }

//    uLeft = Kp * ekLeft + Ki * IkLeft; //control effort calc
//    uRight = Kp * ekRight + Ki * IkRight;

    //amp dbc set the previous values
    ekLeft_1 = ekLeft;
    ekRight_1 = ekRight;
    IkLeft_1 = IkLeft;
    IkRight_1 = IkRight;

    CpuTimer1.InterruptCount++;
}

// cpu_timer2_isr CPU Timer2 ISR


__interrupt void cpu_timer2_isr(void)
{
    /*
    speedRef = -2;
    //DB implementing wall distance PI control for maze corridor movement
    float Kp_wall = 1.5;
    float dt = 0.002;
    float Ki_wall = .03;
    float wall_ref= 1;
    float e_wall = 0;

    if (IR_left  > 0.4 && IR_right > 0.4) { //if both sides see a wall
        e_wall = IR_left - IR_right;
        if (fabs(turnRate) < 3) {
            i_wall += (e_wall + e_wall_1)*dt/2;
        }
        turnRate = Kp_wall*e_wall + Ki_wall*i_wall;
    } else if (IR_left > 0.4) { //if only left side sees a wall
        e_wall = IR_left - wall_ref;
        if (fabs(turnRate) < 3) {
            i_wall += (e_wall + e_wall_1)*dt/2;
        }
        turnRate = Kp_wall*e_wall + Ki_wall*i_wall;

    } else if (IR_right > 0.4) { // if only right side sees a wall
        e_wall = wall_ref - IR_right;
        if (fabs(turnRate) < 3) {
            i_wall += (e_wall + e_wall_1)*dt/2;
        }
        turnRate = Kp_wall*e_wall + Ki_wall*i_wall;
    } else { //nobody sees a wall

    }

    if (turnRate > 3)
        turnRate = 3;
    else if (turnRate < -3)
        turnRate = -3;
    e_wall_1 = e_wall;


    CpuTimer2.InterruptCount++;

    if ((CpuTimer2.InterruptCount % 10) == 0)
    {



    }*/
}
int16_t spibCount = 0;

__interrupt void SPIB_isr(void)
{
    //    amp dbc Joystick motor control
    //    int16_t temp = SpibRegs.SPIRXBUF;
    //    ADC1val = SpibRegs.SPIRXBUF; // Read first 16-bit value off RX FIFO should be ADC1
    //    ADC2val = SpibRegs.SPIRXBUF;// Read second 16-bit value off RX FIFO
    //    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Set GPIO9 high to end Slave Select. Now Scope. Later to deselect DAN28027

    //recieving from RX buf
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // slave select High

    int16_t temp = SpibRegs.SPIRXBUF;
    accel_x_raw = SpibRegs.SPIRXBUF;
    accel_y_raw = SpibRegs.SPIRXBUF;
    accel_z_raw = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    gyro_x_raw = SpibRegs.SPIRXBUF;
    gyro_y_raw = SpibRegs.SPIRXBUF;
    gyro_z_raw = SpibRegs.SPIRXBUF;

    //amp dbc perform integer mapping
    accel_x = accel_x_raw / 32768.0 * 4.0;
    accel_y = accel_y_raw / 32768.0 * 4.0;
    accel_z = accel_z_raw / 32768.0 * 4.0;
    gyro_x = gyro_x_raw / 32768.0 * 250;
    gyro_y = gyro_y_raw / 32768.0 * 250;
    gyro_z = gyro_z_raw / 32768.0 * 250;

    //Code to be copied into SPIB_ISR interrupt function after the IMU measurements have been collected.
    if (calibration_state == 0)
    {
        calibration_count++;
        if (calibration_count == 2000)
        {
            calibration_state = 1;
            calibration_count = 0;
        }
    }
    else if (calibration_state == 1)
    {
        accelx_offset += accel_x;
        accely_offset += accel_y;
        accelz_offset += accel_z;
        gyrox_offset += gyro_x;
        gyroy_offset += gyro_y;
        gyroz_offset += gyro_z;
        calibration_count++;
        if (calibration_count == 2000)
        {
            calibration_state = 2;
            accelx_offset /= 2000.0;
            accely_offset /= 2000.0;
            accelz_offset /= 2000.0;
            gyrox_offset /= 2000.0;
            gyroy_offset /= 2000.0;
            gyroz_offset /= 2000.0;
            calibration_count = 0;
            doneCal = 1;
        }
    }
    else if (calibration_state == 2)
    {
        accel_x -= (accelx_offset);
        accel_y -= (accely_offset);
        accel_z -= (accelz_offset - accelzBalancePoint);
        gyro_x -= gyrox_offset;
        gyro_y -= gyroy_offset;
        gyro_z -= gyroz_offset;
        /*--------------Kalman Filtering code start---------------------------------------------------------------------*/
        float tiltrate = (gyro_x * PI) / 180.0; // rad/s
        float pred_tilt, z, y, S;
        // Prediction Step
        pred_tilt = kalman_tilt + T * tiltrate;
        pred_P = kalman_P + Q;
        // Update Step
        z = -accel_z; // Note the negative here due to the polarity of AccelZ
        y = z - pred_tilt;
        S = pred_P + R;
        kalman_K = pred_P / S;
        kalman_tilt = pred_tilt + kalman_K * y;
        kalman_P = (1 - kalman_K) * pred_P;
        SpibNumCalls++;
        // Kalman Filter used
        tilt_array[SpibNumCalls] = kalman_tilt;
        gyro_array[SpibNumCalls] = tiltrate;
        LeftWheelArray[SpibNumCalls] = readEncLeft();
        RightWheelArray[SpibNumCalls] = readEncRight();


        if (SpibNumCalls >= 3)
        { // should never be greater than 3
            tilt_value = (tilt_array[0] + tilt_array[1] + tilt_array[2]
                    + tilt_array[3]) / 4.0;
            gyro_value = (gyro_array[0] + gyro_array[1] + gyro_array[2]
                    + gyro_array[3]) / 4.0;
            LeftWheel = (LeftWheelArray[0] + LeftWheelArray[1]
                    + LeftWheelArray[2] + LeftWheelArray[3]) / 4.0;
            RightWheel = (RightWheelArray[0] + RightWheelArray[1]
                    + RightWheelArray[2] + RightWheelArray[3]) / 4.0;
            SpibNumCalls = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }


        /*
        if (SpibNumCalls >= 1)
        { // should never be greater than 3
            tilt_value = (tilt_array[0] + tilt_array[1]) / 2.0;
            gyro_value = (gyro_array[0] + gyro_array[1]) / 2.0;
            LeftWheel = (LeftWheelArray[0] + LeftWheelArray[1]) / 2.0;
            RightWheel = (RightWheelArray[0] + RightWheelArray[1]) / 2.0;
            SpibNumCalls = -1;
            PieCtrlRegs.PIEIFR12.bit.INTx9 = 1; // Manually cause the interrupt for the SWI
        }*/
    }
    timecount++;
    if ((timecount % 200) == 0)
    {
        if (doneCal == 0)
        {
            GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1; // Blink Blue LED while calibrating
        }
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Always Block Red LED
        UARTPrint = 1; // Tell While loop to print
    }
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

    //WHEEL MOTORS::
//    leftWheel = readEncLeft();
//    rightWheel = readEncRight();

    // Later when actually communicating with the DAN28027 do something with the data
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag just in case of an overflow
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear RX FIFO Interrupt flag so next interrupt will happen
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6; // Acknowledge INT6 PIE interrupt

    if ((spibCount % 200) == 0)
    {
        UARTPrint = 1;
    }
    spibCount++;
}

void setupSpib(void) //Call this function in main() somewhere after the DINT; line of code.
{
    // ---------------------------------------------------------------------------
    SpibRegs.SPICCR.bit.SPISWRESET = 0; // Put SPI in Reset
    SpibRegs.SPICTL.bit.CLK_PHASE = 1; //This happens to be the mode for both the DAN28027 and
    SpibRegs.SPICCR.bit.CLKPOLARITY = 0; //The MPU-9250, Mode 01.
    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1; // Set to SPI Master
    SpibRegs.SPICCR.bit.SPICHAR = 15; // Set to transmit and receive 16-bits each write to SPITXBUF
    SpibRegs.SPICTL.bit.TALK = 1; // Enable transmission
    SpibRegs.SPIPRI.bit.FREE = 1; // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA = 0; // Disables the SPI interrupt
    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49; // Set SCLK bit rate to 1 MHz so 1us period. SPI base clock is
    // 50MHZ. And this setting divides that base clock to create SCLK’s period
    SpibRegs.SPISTS.all = 0x0000; // Clear status flags just in case they are set for some reason
    SpibRegs.SPIFFTX.bit.SPIRST = 1; // Pull SPI FIFO out of reset, SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA = 1; // Enable SPI FIFO enhancements
    SpibRegs.SPIFFTX.bit.TXFIFO = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR = 1; // Write 1 to clear SPIFFTX[TXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0; // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Write 1 to clear SPIFFRX[RXFFOVF] just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Write 1 to clear SPIFFRX[RXFFINT] flag just in case it is set
    SpibRegs.SPIFFRX.bit.RXFFIENA = 1; // Enable the RX FIFO Interrupt. RXFFST >= RXFFIL
    SpibRegs.SPIFFCT.bit.TXDLY = 16; //Set delay between transmits to 16 spi clocks. Needed by DAN28027 chip
    SpibRegs.SPICCR.bit.SPISWRESET = 1; // Pull the SPI out of reset
    SpibRegs.SPIFFTX.bit.TXFIFO = 1; // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1; // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA = 1; // Enables SPI interrupt. !! I don’t think this is needed. Need to Test
    SpibRegs.SPIFFRX.bit.RXFFIL = 16; //Interrupt Level to 16 words or more received into FIFO causes interrupt. This is just the initial setting for the register. Will be changed below

    int16_t temp = 0;
    //Step 1.
    // cut and paste here all the SpibRegs initializations you found for part 3. Make sure the TXdelay in
    // between each transfer to 0. Also don’t forget to cut and paste the GPIO settings for GPIO9, 63, 64, 65,
    // 66 which are also a part of the SPIB setup.

    //amp dbc Init the SPI data communication
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0); // Set as GPIO9 and used as DAN28027 SS
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO9 an Output Pin
    GpioDataRegs.GPASET.bit.GPIO9 = 1; //Initially Set GPIO9/SS High so DAN28027 is not selected
    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0); // Set as GPIO66 and used as MPU-9250 SS
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL); // Make GPIO66 an Output Pin
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; //Initially Set GPIO66/SS High so MPU-9250 is not selected
    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15); //Set GPIO63 pin to SPISIMOB
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15); //Set GPIO64 pin to SPISOMIB
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15); //Set GPIO65 pin to SPICLKB
    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0; // Enable Pull-ups on SPI PINs Recommended by TI for SPI Pins
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3; // Set I/O pin to asynchronous mode recommended for SPI
    EDIS;
    //DAN28027  CS  Chip Select
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO9 = 1;

    SpibRegs.SPICCR.bit.SPICHAR = 0xF;
    SpibRegs.SPIFFCT.bit.TXDLY = 0x00;

    //---------------------------------------------------------------------------------------------------

    //Step 2.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x13,0x14,0x15,0x16
    // 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C 0x1D, 0x1E, 0x1F. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    SpibRegs.SPIFFRX.bit.RXFFIL = 7; // Perform the number of needed writes to SPITXBUF to write to all 13 registers. Remember we are sending 16-bit transfers, so two registers at a time after the first 16-bit transfer.
    SpibRegs.SPITXBUF = 0x1300; // To address 00x13 write 0x00
    SpibRegs.SPITXBUF = 0x0000; // To address 00x14 write 0x00
    SpibRegs.SPITXBUF = 0x0000; // To address 00x15 write 0x00
    SpibRegs.SPITXBUF = 0x0013; // To address 00x16 write 0x00
    SpibRegs.SPITXBUF = 0x0200; // To address 00x17 write 0x00
    SpibRegs.SPITXBUF = 0x0806; // To address 00x18 write 0x00
    SpibRegs.SPITXBUF = 0x0000; // To address 00x19 write 0x13
    // To address 00x1A write 0x02
    // To address 00x1B write 0x00
    // To address 00x1C write 0x08
    // To address 00x1D write 0x06
    // To address 00x1E write 0x00
    // To address 00x1F write 0x00
    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while (SpibRegs.SPIFFRX.bit.RXFFST != 7)
        ; //amp dbc wait for all 13 items to be sent
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF; // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.
    //Step 3.
    // perform a multiple 16-bit transfer to initialize MPU-9250 registers 0x23,0x24,0x25,0x26
    // 0x27, 0x28, 0x29. Use only one SS low to high for all these writes
    // some code is given, most you have to fill you yourself.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // Slave Select Low
    SpibRegs.SPIFFRX.bit.RXFFIL = 4; // Perform the number of needed writes to SPITXBUF to write to all 7 registers
    SpibRegs.SPITXBUF = 0x2300; // To address 00x23 write 0x00
    SpibRegs.SPITXBUF = 0x408C; // To address 00x24 write 0x40
    SpibRegs.SPITXBUF = 0x0288; // To address 00x25 write 0x8C
    SpibRegs.SPITXBUF = 0x0C0A; // To address 00x26 write 0x02
    // To address 00x27 write 0x88
    // To address 00x28 write 0x0C
    // To address 00x29 write 0x0A
    // wait for the correct number of 16-bit values to be received into the RX FIFO
    while (SpibRegs.SPIFFRX.bit.RXFFST != 4)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1; // Slave Select High
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    // ???? read the additional number of garbage receive values off the RX FIFO to clear out the RX FIFO
    DELAY_US(10); // Delay 10us to allow time for the MPU-2950 to get ready for next transfer.
    //Step 4.
    // perform a single 16-bit transfer to initialize MPU-9250 register 0x2A
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = 0x2A81;    // Write to address 0x2A the value 0x81
    // wait for one byte to be received
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    // The Remainder of this code is given to you and you do not need to make any changes.
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3800 | 0x0001); // 0x3800
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x3A00 | 0x0001); // 0x3A00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6400 | 0x0001); // 0x6400
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6700 | 0x0003); // 0x6700
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6A00 | 0x0020); // 0x6A00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x6B00 | 0x0001); // 0x6B00
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7500 | 0x0071); // 0x7500
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7700 | 0x00EC); // 0x7700 //amp dbc x accel OFFSETS
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7800 | 0x0020); // 0x7800  //amp dbc xaccel OFFSETS
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7A00 | 0x0004); // 0x7A00 //amp dbc Yaccel OFFSETS
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7B00 | 0x00B0); // 0x7B00 //amp dbc Yaccel OFFSETS
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7D00 | 0x0023); // 0x7D00 //amp dbc Zaccel OFFSETS
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x7E00 | 0x0028); // 0x7E00 //amp dbc Zaccel OFFSETS
    while (SpibRegs.SPIFFRX.bit.RXFFST != 1)
        ;
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    DELAY_US(50);
    // Clear SPIB interrupt source just in case it was issued due to any of the above initializations.
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR = 1; // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR = 1; // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;
}

void init_eQEPs(void)
{
    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QPOSCNT = 0;
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep

    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pinsfor reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend
    EQep2Regs.QPOSCNT = 0;
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
}
float readEncLeft(void)
{
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U
    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue / 2)
        raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw * (-1 * (2 * PI) / 12000)); //2pi/12000 //12000 counts/ rev of wheel
}
float readEncRight(void)
{
    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU; //4294967295U -1 32bit signed int
    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue / 2)
        raw -= QEP_maxvalue; // I don't think this is needed and never true
    // 100 slits in the encoder disk so 100 square waves per one revolution of the
    // DC motor's back shaft. Then Quadrature Decoder mode multiplies this by 4 so 400 counts per one rev
    // of the DC motor's back shaft. Then the gear motor's gear ratio is 30:1.
    return (raw * ((2 * PI) / 12000));
}
//amp dbc saturated the motor input and sets the motor velocity
void setEPWM2A(float controleffort)
{
    //amp dbc saturate controleffort if too big or too small
    if (controleffort > 10)
    {
        controleffort = 10;
    }
    if (controleffort < -10)
    {
        controleffort = -10;
    }
    // this is a linear mapping
    EPwm2Regs.CMPA.bit.CMPA = ((controleffort / 20.0) + 0.5) * EPwm2Regs.TBPRD; //note the float division
}

//amp dbc saturated the motor input and sets the motor velocity
void setEPWM2B(float controleffort)
{
    //amp dbc saturate controleffort if too big or too small
    if (controleffort > 10)
    {
        controleffort = 10;
    }
    if (controleffort < -10)
    {
        controleffort = -10;
    }

    EPwm2Regs.CMPB.bit.CMPB = ((controleffort / 20.0) + 0.5) * EPwm2Regs.TBPRD;
}
//amp dbc calculates the robot X Y position and the bearing
void calculateRobotPose(void)
{
    bearing = Rwh / W * (rightWheel - leftWheel);
    x_dot = Rwh * omegaAvg * cos(bearing);
    y_dot = Rwh * omegaAvg * sin(bearing);
    x += (0.004 * (x_dot + x_dotPrev) / 2.0);
    y += (0.004 * (y_dot + y_dotPrev) / 2.0);
    x_dotPrev = x_dot;
    y_dotPrev = y_dot;

}

float linear_move_kp = 4;
float linear_move_tolerance = .01;

//units of error in ft
int move_linear(float error){
    linear_move_command_vel = linear_move_kp * error;

   if(error < linear_move_tolerance){
       return 1;
   }
   else{
       return 0;
   }

}
/*
void move(float targetX, float targetY){
    float posError = targetPos - ;


}*/

//xk is the current ADC reading, xk_1 is the ADC reading one millisecond ago, xk_2 two milliseconds ago, etc
float xk = 0;
int arr_size = 22;
float xk_arr[22];
//yk is the filtered value
float yk = 0;
//b is the filter coefficients from matlab
float b[22] = { -2.3890045153263611e-03, -3.3150057635348224e-03,
                -4.6136191242627002e-03, -4.1659855521681268e-03,
                1.4477422497795286e-03, 1.5489414225159667e-02,
                3.9247886844071371e-02, 7.0723964095458614e-02,
                1.0453473887246176e-01, 1.3325672639406205e-01,
                1.4978314227429904e-01, 1.4978314227429904e-01,
                1.3325672639406205e-01, 1.0453473887246176e-01,
                7.0723964095458614e-02, 3.9247886844071371e-02,
                1.5489414225159667e-02, 1.4477422497795286e-03,
                -4.1659855521681268e-03, -4.6136191242627002e-03,
                -3.3150057635348224e-03, -2.3890045153263611e-03 }; // 0.2 is 1/5th therefore a 5 point average

float x_raw = 0;
float y_raw = 0;
float x_volt = 0;
float y_volt = 0;

float x_arr[22];
float y_arr[22];

float x_filtered = 0;
float y_filtered = 0;
__interrupt void ADCA_ISR(void)
{

    //enable SPI
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1; // slave select low
    SpibRegs.SPIFFRX.bit.RXFFIL = 8; // Issue the SPIB_RX_INT when 8 values are in the RX FIFO
    SpibRegs.SPITXBUF = 0xBA00; // AMP dbc starting with init_status
    SpibRegs.SPITXBUF = 0x0000; //sending garbage to receive next 7 16-bit vals
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;
    SpibRegs.SPITXBUF = 0x0000;

    x_raw = AdcaResultRegs.ADCRESULT0;
    y_raw = AdcaResultRegs.ADCRESULT1;

    // Here covert ADCIND0, ADCIND1 to volts
    x_volt = x_raw / 4096.0 * 3.0;
    y_volt = y_raw / 4096.0 * 3.0;

    //amp dbc clearing the values
    x_filtered = 0; //amp dbc resets the values being outputted every time so that we dont keep counting up
    y_filtered = 0;

    //amp dbc For loops for the filtering so that we can have many order filters
    for (int i = arr_size - 1; i > 0; i--)
    {
        x_arr[i] = x_arr[i - 1];
        y_arr[i] = y_arr[i - 1];

        x_filtered += x_arr[i] * b[i];
        y_filtered += y_arr[i] * b[i];
    }
    x_arr[0] = x_volt;
    x_filtered += x_arr[0] * b[0];

    y_arr[0] = y_volt;
    y_filtered += y_arr[0] * b[0];

    //amp dbc send the voltages to global variable to be printed
    IR_2 = x_filtered;
    IR_1 = y_filtered;

    // Print ADCIND0 and ADCIND1’s voltage value to TeraTerm every 100ms
//    if ((numADCD1calls % 1000) == 0)
//    {
//        UARTPrint = 1;
//    }
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
