//added gain increment snippet for inductive loads.
//removed delay line 812
//made stepfreq global
//changed constant GBWP
//TODO:CHANGE maxSweepFCount
// Standard includes
#include <string.h>
#include <stdlib.h>
#include <math.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_ints.h"
#include "spi.h"
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "prcm.h"
#include "uart.h"
#include "interrupt.h"
#include "common.h"
#include "timer_if.h"
#include "timer.h"
#include "gpio.h"
#include "hw_gprcm.h"
#include "pin.h"
#include "adc.h"
#include "i2c_if.h"

// Common interface includes
#include "uart_if.h"
#include "pin_mux_config.h"





// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode

#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     128
#define adcSamplesNumber 16000   //to trace a whole period @10hz, 6250samples are needed per channel.(twice that for the 2 channels i use per input)
#define GBWP  2700000   //AD8231 GBWP
#define maxSweepFCount     1100//wont use step less than 10.

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
unsigned short controlReg1=0b0010000100000000;    //reset=1,two word write follows
unsigned short freq0Msbs;
unsigned short freq0Lsbs;
unsigned short reset1=0b0000000100000000;
unsigned short reset0=0b0000000000000000;
int stepFreq,startFreq,freq,endFreq,mode;
int mincounter0,mincounter1,minTimestamp1,minTimestamp0;
float stampDiff;
static short D,Dprev;
static unsigned short valAdc1[adcSamplesNumber],valAdc0[adcSamplesNumber],temp[adcSamplesNumber],minUnsmoothed1,minUnsmoothed0,maxUnsmoothed0,maxUnsmoothed1;
static unsigned short minValue0,maxValue0,minValue1,maxValue1;
static unsigned short count;//LET 10000 be the max bumber of frequencies todo change these to single variable later to savce space?
static float pk_pk_phaseDiff[maxSweepFCount],temp1[maxSweepFCount],ohm,gain,impedance[maxSweepFCount],noRolloffFreq;
static float pk_pk1[maxSweepFCount];//save space TODO
unsigned long x,x1;
float ms,y,periodus,periodsToScan;
unsigned char i2cBuf[TR_BUFF_SIZE];
long spiRet;
static tBoolean TA1running;
static tBoolean clipped;
static int adcIndex1,adcIndex0; //TODO:CHECK MAX NUMBER THIS CAN REACH AND DECREASE DATA TYPE TO SAVE MEMORY.

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static void BoardInit();
int getStartFreq(void);
int getEndFreq(void);
int getStepFreq(void);
int selectMode(void);
void changeGain(unsigned short);
void waitForEnter(void);
float tdif(unsigned long int m);
void adcSetup(void);
void enableADCints(void);
void disableADCints(void);
void inAmpSetup(void);
static void adint3(void);
static void adint1(void);
static void adint0(void);
static void adint2(void);
static void countdownTimerInt(void);
void configTA1(void);
void startatFreq(float);
void int31(void);
void findInitialGain(void);
void finalSmoothingImpedance(void);
void Evaluate(void);
void finalSmoothingMedian(void);
void finalSmoothingAverage(void);
void clearAdc(void);

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************



void changeGain(unsigned short d){//d=0-7
    gain=1<<d;
    tBoolean gpio9,gpio10,gpio11;//values for the A2,A1,A0
    gpio9=(d&0b100)>>2;//msb
    gpio10=(d&0b10)>>1;
    gpio11=d&0b1;//lsb
    GPIOPinWrite(GPIOA1_BASE, 0x2,gpio9<<1);
    GPIOPinWrite(GPIOA1_BASE, 0x4,gpio10<<2);
    GPIOPinWrite(GPIOA1_BASE, 0x8,gpio11<<3);
    MAP_UtilsDelay(1000);               // TODO:wait for inamp?
}

void waitForEnter(){
    char acCmdStore[50];
    int lRetVal;
    UART_PRINT("Press enter for next frequency...");
    lRetVal = GetCmd(acCmdStore, sizeof(acCmdStore));
}

int selectMode(){
    int iInput = 0;
    char acCmdStore[50];
    int lRetVal;
    UART_PRINT("Give mode: 1-sinusoidal  2-triangle  3-square wave");
    do
    {
        lRetVal = GetCmd(acCmdStore, sizeof(acCmdStore));
        if (lRetVal==0){UART_PRINT("Wrong input,try again");}
        else{
            iInput  = (int)strtoul(acCmdStore,0,10);
            if(iInput<=0 || iInput>3){
                  UART_PRINT("Wrong input,try again");
                }
            else return iInput;
        }
    }while(true);
}

int getStartFreq(){
    int iInput = 0;
    char acCmdStore[50];
    int lRetVal;
    UART_PRINT("Give start frequency in Hz(integer):");
    do
    {
        lRetVal = GetCmd(acCmdStore, sizeof(acCmdStore));
        if (lRetVal==0){UART_PRINT("Wrong input,try again");}
        else{
            iInput  = (int)strtoul(acCmdStore,0,10);
            if(iInput<=0 || iInput>10000000){
                  UART_PRINT("Wrong input,try again");
                }
            else return iInput;
        }
    }while(true);
}

int getEndFreq(){
    int iInput = 0;
    char acCmdStore[50];
    int lRetVal;
    UART_PRINT("Give end frequency in Hz(integer):");
    do
    {
        lRetVal = GetCmd(acCmdStore, sizeof(acCmdStore));
        if (lRetVal==0){UART_PRINT("Wrong input,try again");}
        else{
            iInput  = (int)strtoul(acCmdStore,0,10);
            if(iInput<=startFreq || iInput>10000000){
                  UART_PRINT("Wrong input,try again");
                }
            else return iInput;
        }
    }while(true);
}
int getStepFreq(){
    int iInput = 0;
    char acCmdStore[50];
    int lRetVal;
    UART_PRINT("Give frequency step in Hz(integer):");
    do
    {
        lRetVal = GetCmd(acCmdStore, sizeof(acCmdStore));
        if (lRetVal==0){UART_PRINT("Wrong input,try again");}
        else{
            iInput  = (int)strtoul(acCmdStore,0,10);
            if(iInput<1 || iInput>10000000){
                  UART_PRINT("Wrong input,try again");
                }
            else return iInput;
        }
    }while(true);
}

void startatFreq(float Freq ){

    unsigned int frequencyReg=(Freq*0x10000000)/25000000.0;    //get frequency via UART, and make calculation as per datasheet.(mclk=25Mhz)
    freq0Lsbs= (frequencyReg&0x3fff) | 0b0100000000000000 ;//keep the 14 LSBs and set the 2 msbs according to datasheet
    freq0Msbs=  (frequencyReg>>14) | 0b0100000000000000 ;  //keep 14 msbs,set 2 msbs

    spiRet=MAP_SPITransfer(GSPI_BASE,&reset1,0,2,
            SPI_CS_ENABLE|SPI_CS_DISABLE);
    spiRet=MAP_SPITransfer(GSPI_BASE,&controlReg1,0,2,
            SPI_CS_ENABLE|SPI_CS_DISABLE);
    spiRet=MAP_SPITransfer(GSPI_BASE,&freq0Lsbs,0,2,
            SPI_CS_ENABLE|SPI_CS_DISABLE);
    spiRet=MAP_SPITransfer(GSPI_BASE,&freq0Msbs,0,2,
            SPI_CS_ENABLE|SPI_CS_DISABLE);
    spiRet=MAP_SPITransfer(GSPI_BASE,&reset0,0,2,
            SPI_CS_ENABLE|SPI_CS_DISABLE);
    MAP_UtilsDelay(1000);               //wait for DDS. datasheet says wait 8 MCLK cycles. Worst case if 1Mhz,wait 640 cc3200 cycles.

}

void findInitialGain(){
    D=0;
    Dprev=D;
    while(1){
        changeGain(D);
        minUnsmoothed1=4095;//start maximized,finish with acquired min value after each measurement.
        minUnsmoothed0=4095;
        clipped=false;
        adcIndex1=0;
        adcIndex0=0;
        TA1running=true;
        if(freq<800){//found this to be ok
            periodsToScan=1.2;
            TimerLoadSet(TIMERA1_BASE,TIMER_A,MILLISECONDS_TO_TICKS(periodsToScan*1000.0/freq)); //  1/freq = 1 period.
        }
        else{
            periodsToScan=6;
            TimerLoadSet(TIMERA1_BASE,TIMER_A,MILLISECONDS_TO_TICKS(periodsToScan*1000.0/freq)); //  1/freq = 1 period.
        }
        TimerEnable(TIMERA1_BASE,TIMER_A);
        clearAdc();// clear data
        enableADCints();
        ADCEnable(ADC_BASE);    //START TO MEASURE
        while(TA1running==true);
        disableADCints();

        if((minUnsmoothed1<1400)||(clipped==true)){
            D=Dprev;
            break;
        }
        else{
            Dprev=D;
            D=D+1;//BE INCREASING GAIN UNTIL CLIPPED.
            if(D>7){
                D=7;
                UART_PRINT("Sample very conductive\n\r");
                break;
            }
        }
    }
    changeGain(D);
}
//find edges & calculate phase difference and amplitude
void Evaluate(){
    char i3,G=3;
    int i1,i2;
    float i7,i8,i9;


    //find edges in array:
    minValue0=valAdc0[0];
    maxValue0=valAdc0[0];
    for(i1=1;i1<adcIndex0;i1++){
        if(valAdc0[i1]<minValue0){
            minValue0=valAdc0[i1];
            if(8.0*i1<periodus){       //use only the first sampled period of the wave for finding phase diff
                minTimestamp0=i1;
            }

            mincounter0++;
        }
        if(valAdc0[i1]>maxValue0){
            maxValue0=valAdc0[i1];
        }
    }


    //find edges in array:
    minValue1=valAdc1[0];
    maxValue1=valAdc1[0];
    for(i1=1;i1<adcIndex1;i1++){
        if(valAdc1[i1]<minValue1){
            minValue1=valAdc1[i1];
            if(8.0*i1<periodus){       //use only the first sampled period of the wave for finding phase diff
                minTimestamp1=i1;
            }
            mincounter1++;
        }
        if(valAdc1[i1]>maxValue1){
            maxValue1=valAdc1[i1];
        }
    }

    if(mode!=3){//no use detecting input channel edges for square waveform
        i7=minTimestamp0-minTimestamp1;
        i8=minTimestamp0-(minTimestamp1+((float)adcIndex1/periodsToScan));//(from notes)add 1 period in case the 1st edge didn't get picked at the dds source signal,causing it to show 1 period later.
        i9=minTimestamp1-(minTimestamp0+((float)adcIndex1/periodsToScan));
        if((abs(i7)<abs(i8))&&(abs(i7)<abs(i9))){//from notes:go with the smallest abs value of phase difference.
            stampDiff=i7;
        }
        else if ((abs(i8)<abs(i7))&&(abs(i8)<abs(i9))){
            stampDiff=i8;
        }
        else{
            stampDiff=i9;
        }
        pk_pk_phaseDiff[count]=360.0*freq*stampDiff*8/1e6;//adc timer cycle is 16us,sample for each channel every 8us

    }
    else{
        pk_pk_phaseDiff[count]=0;
    }

    pk_pk1[count]= (maxValue1-minValue1)*1.467/4096.0;//in volts TODO:try the other formula i have written down too.
    impedance[count]=(maxValue1-minValue1)*1.467/4096.0*1e5/1.87/gain;//p-p current is 18.7uA with 33k Radj
}

//Median filter according to wikipedia. Change each value according to its neigborhood's median. smoothen out response waveforms using rolling median
void finalSmoothingImpedance(){
    //smoothen out impedance
        unsigned char smoothingInterval1=5;
        char i3;
        int i1,i2;
        float smoothWindow[300],temp2;//!!let smoothingInterval<300
        for(i1=smoothingInterval1/2;i1<(count-smoothingInterval1/2);i1++){
            i3=0;
            //copy values to buffer to sort them:
            for(i2=0;i2<smoothingInterval1;i2++){
                smoothWindow[i2]=impedance[i1-smoothingInterval1/2+i2];
            }
            while(i3<smoothingInterval1-1){
            i2=0;
                while(i2<smoothingInterval1-1){
                    if (smoothWindow[i2]>smoothWindow[i2+1]){
                        temp2=smoothWindow[i2];
                        smoothWindow[i2]=smoothWindow[i2+1];
                        smoothWindow[i2+1]=temp2;
                    }
                i2++;
                }
            i3++;
            }
            temp1[i1]=smoothWindow[smoothingInterval1/2];//save median

        }
        //fill array borders with first&last filtered values:
        for(i1=0;i1<count;i1++){
            if (i1<=smoothingInterval1/2){
                temp1[i1]=temp1[smoothingInterval1/2+1];
            }
            else if (i1>=count-smoothingInterval1/2){
                temp1[i1]=temp1[count-smoothingInterval1/2-1];
            }
        }

        //restore filtered array to original array
        for(i1=0;i1<count;i1++){
            impedance[i1]=temp1[i1];
        }
}

//smoothen out response waveforms using rolling median
void finalSmoothingMedian(){
    //smoothen out phase
    unsigned char smoothingInterval1=13;
    char i3;
    int i1,i2;
    float smoothWindow[300],temp2;//!!let smoothingInterval<300
    for(i1=smoothingInterval1/2;i1<(count-smoothingInterval1/2);i1++){
        i3=0;
        //copy values to buffer to sort them:
        for(i2=0;i2<smoothingInterval1;i2++){
            smoothWindow[i2]=pk_pk_phaseDiff[i1-smoothingInterval1/2+i2];
        }
        while(i3<smoothingInterval1-1){
        i2=0;
            while(i2<smoothingInterval1-1){
                if (smoothWindow[i2]>smoothWindow[i2+1]){
                    temp2=smoothWindow[i2];
                    smoothWindow[i2]=smoothWindow[i2+1];
                    smoothWindow[i2+1]=temp2;
                }
            i2++;
            }
        i3++;
        }
        temp1[i1]=smoothWindow[smoothingInterval1/2];//save median

    }
    //fill array borders with first&last filtered values:
    for(i1=0;i1<count;i1++){
        if (i1<=smoothingInterval1/2){
            temp1[i1]=temp1[smoothingInterval1/2+1];
        }
        else if (i1>=count-smoothingInterval1/2){
            temp1[i1]=temp1[count-smoothingInterval1/2-1];
        }
    }

    //restore filtered array to original array
    for(i1=0;i1<count;i1++){
        pk_pk_phaseDiff[i1]=temp1[i1];
    }

}

//smoothen out response waveforms using rolling average
void finalSmoothingAverage(){
    //smoothen out phase
    unsigned char smoothingInterval1=13;
    char i3;
    int i1,i2;
    float smoothWindow[300],temp2;//!!let smoothingInterval<300
    for(i1=smoothingInterval1/2;i1<(count-smoothingInterval1/2);i1++){
        i3=0;
        //copy values to buffer to sort them:
        for(i2=0;i2<smoothingInterval1;i2++){
            smoothWindow[i2]=pk_pk_phaseDiff[i1-smoothingInterval1/2+i2];
        }
        while(i3<smoothingInterval1-1){
        i2=0;
        temp2=0;
            while(i2<smoothingInterval1-1){
                    temp2=smoothWindow[i2]+temp2;
            i2++;
            }
        i3++;
        }
        temp1[i1]=temp2/smoothingInterval1;//save median
    }

    //fill array borders with first&last filtered values:
    for(i1=0;i1<count;i1++){
        if (i1<=smoothingInterval1/2){
            temp1[i1]=temp1[smoothingInterval1/2+1];
        }
        else if (i1>=count-smoothingInterval1/2){
            temp1[i1]=temp1[count-smoothingInterval1/2-1];
        }
    }

    //restore filtered array to original array
    for(i1=0;i1<count;i1++){
        pk_pk_phaseDiff[i1]=temp1[i1];
    }

}

float tdif(unsigned long int m){
    unsigned long h=0;
    float k;
    h=TimerValueGet(TIMERA0_BASE, TIMER_A);
    if(h<m){
        k=(m-h)*12.5/1000;
    }
    else{

    }
    return k;
}
//*****************************************************************************
//
//! SPI Master mode main loop
//!
//! This function configures SPI modelue as master and enables the channel for
//! communication
//!
//! \return None.
//
//*****************************************************************************


void configTA1(){   //countdown timer
    Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_ONE_SHOT, TIMER_A, 0);  //countdown timer
    TimerControlStall(TIMERA1_BASE, TIMER_A,true);  //enable timer stall on debug breakpoint
    Timer_IF_IntSetup(TIMERA1_BASE, TIMER_A, countdownTimerInt);
}



void inAmpSetup(void){
 //smallest GAIN=1
changeGain(0);

}

void adcSetup(){
    // clear data
    while(ADCFIFOLvlGet(ADC_BASE, ADC_CH_3)) {
        ADCFIFORead(ADC_BASE, ADC_CH_3);}
    while(ADCFIFOLvlGet(ADC_BASE, ADC_CH_1)) {
        ADCFIFORead(ADC_BASE, ADC_CH_1);}
    while(ADCFIFOLvlGet(ADC_BASE, ADC_CH_0)) {
        ADCFIFORead(ADC_BASE, ADC_CH_0);}
    while(ADCFIFOLvlGet(ADC_BASE, ADC_CH_2)) {
        ADCFIFORead(ADC_BASE, ADC_CH_2);}

// initialize ADC
    ADCDisable(ADC_BASE);
    ADCTimerConfig(ADC_BASE,0x1ffff);  //or 20000?
    ADCTimerEnable(ADC_BASE);
    ADCIntClear(ADC_BASE, ADC_CH_3,0xf);
    //ADCIntEnable(ADC_BASE, ADC_CH_3,ADC_FIFO_FULL);
    ADCIntRegister(ADC_BASE, ADC_CH_3,adint3);
    ADCIntClear(ADC_BASE, ADC_CH_0,0xf);
    //ADCIntEnable(ADC_BASE, ADC_CH_0,ADC_FIFO_FULL);
    ADCIntRegister(ADC_BASE, ADC_CH_0,adint0);
    ADCIntClear(ADC_BASE, ADC_CH_1,0xf);
    //ADCIntEnable(ADC_BASE, ADC_CH_1,ADC_FIFO_FULL);
    ADCIntRegister(ADC_BASE, ADC_CH_1,adint1);
    ADCIntClear(ADC_BASE, ADC_CH_2,0xf);
    //ADCIntEnable(ADC_BASE, ADC_CH_2,ADC_FIFO_FULL);
    ADCIntRegister(ADC_BASE, ADC_CH_2,adint2);
    ADCChannelEnable(ADC_BASE, ADC_CH_3);
    ADCChannelEnable(ADC_BASE, ADC_CH_1);
    ADCChannelEnable(ADC_BASE, ADC_CH_0);
    ADCChannelEnable(ADC_BASE, ADC_CH_2);
}
void enableADCints(){
    ADCIntEnable(ADC_BASE, ADC_CH_0,ADC_FIFO_FULL);
    ADCIntEnable(ADC_BASE, ADC_CH_1,ADC_FIFO_FULL);
    ADCIntEnable(ADC_BASE, ADC_CH_2,ADC_FIFO_FULL);
    ADCIntEnable(ADC_BASE, ADC_CH_3,ADC_FIFO_FULL);
}
void disableADCints(){
    ADCIntDisable(ADC_BASE, ADC_CH_0,ADC_FIFO_FULL);
    ADCIntDisable(ADC_BASE, ADC_CH_1,ADC_FIFO_FULL);
    ADCIntDisable(ADC_BASE, ADC_CH_2,ADC_FIFO_FULL);
    ADCIntDisable(ADC_BASE, ADC_CH_3,ADC_FIFO_FULL);
}
void clearAdc(){
    while(ADCFIFOLvlGet(ADC_BASE, ADC_CH_3)) {
        ADCFIFORead(ADC_BASE, ADC_CH_3);}
    while(ADCFIFOLvlGet(ADC_BASE, ADC_CH_1)) {
        ADCFIFORead(ADC_BASE, ADC_CH_1);}
    while(ADCFIFOLvlGet(ADC_BASE, ADC_CH_0)) {
        ADCFIFORead(ADC_BASE, ADC_CH_0);}
    while(ADCFIFOLvlGet(ADC_BASE, ADC_CH_2)) {
        ADCFIFORead(ADC_BASE, ADC_CH_2);}
}

static void adint1()
{   //unsigned long Status = ADCIntStatus(ADC_BASE, ADC_CH_1);
    ADCIntClear(ADC_BASE, ADC_CH_1,0x1f);
        if (TA1running==true){    //this doesnt cause issues.
            valAdc1[adcIndex1] = ADCFIFORead(ADC_BASE, ADC_CH_1) & 0x3FFF;
            valAdc1[adcIndex1] = valAdc1[adcIndex1] >> 2;


            if(minUnsmoothed1>valAdc1[adcIndex1]){
                minUnsmoothed1=valAdc1[adcIndex1];
            }
            adcIndex1++;

    }
}

static void adint3()
{   //unsigned long Status = ADCIntStatus(ADC_BASE, ADC_CH_3);
    ADCIntClear(ADC_BASE, ADC_CH_3,0x1f);
    if (TA1running==true){    //this doesnt cause issues.
        valAdc1[adcIndex1] = ADCFIFORead(ADC_BASE, ADC_CH_3) & 0x3FFF;
        valAdc1[adcIndex1] = valAdc1[adcIndex1] >> 2;

        adcIndex1++;
    }
}
static void adint0()
{   //unsigned long Status = ADCIntStatus(ADC_BASE, ADC_CH_3);
    ADCIntClear(ADC_BASE, ADC_CH_0,0x1f);
    if (TA1running==true){    //this doesnt cause issues.
        valAdc0[adcIndex0] = ADCFIFORead(ADC_BASE, ADC_CH_0) & 0x3FFF;
        valAdc0[adcIndex0] = valAdc0[adcIndex0] >> 2;

        if(minUnsmoothed0>valAdc0[adcIndex0]){
            minUnsmoothed0=valAdc0[adcIndex0];
        }
        adcIndex0++;
    }
}

static void adint2()
{   //unsigned long Status = ADCIntStatus(ADC_BASE, ADC_CH_3);
    ADCIntClear(ADC_BASE, ADC_CH_2,0x1f);
    if (TA1running==true){    //this doesnt cause issues.
        valAdc0[adcIndex0] = ADCFIFORead(ADC_BASE, ADC_CH_2) & 0x3FFF;
        valAdc0[adcIndex0] = valAdc0[adcIndex0] >> 2;



        adcIndex0++;
    }
}

static void countdownTimerInt()
{
    if (TimerIntStatus(TIMERA1_BASE,true)==0x1){   // timeout int has happened
        //unsigned int x5=TimerValueGet(TIMERA1_BASE, TIMER_A);
        TimerIntClear(TIMERA1_BASE,TIMER_TIMA_TIMEOUT);
        ADCDisable(ADC_BASE);    //STop in case it's measuring
        TA1running=false;
    }
}

void int31(void){   //clipper activated
    GPIOIntClear(GPIOA3_BASE, 0x80);
    clipped=true;
}
//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
/* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
  //
  // Set vector table base
  //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//! Main function for spi demo application
//!
//! \param none
//!
//! \return None.
//
//*****************************************************************************
void main()
{
    //
    // Initialize Board configurations
    //
    BoardInit();

    //
    // Muxing UART and SPI lines.
    //
    PinMuxConfig();

    //
    // Initialising the Terminal.
    //
    InitTerm();

    //
    // Clearing the Terminal.
    //
    ClearTerm();

    //
    // Display the Banner
    //
    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t   ********************************************\n\r");
    UART_PRINT("\t\t        CC3200 SPI EIS  \n\r");
    UART_PRINT("\t\t   ********************************************\n\r");
    UART_PRINT("\n\n\n\r");

    //
    // Reset the peripheral
    //
    MAP_PRCMPeripheralReset(PRCM_GSPI);


    //
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    //Configure gpio31 for clipper interrupt:
    GPIOIntTypeSet(GPIOA3_BASE, 0x80,GPIO_RISING_EDGE);
    GPIOIntRegister(GPIOA3_BASE,int31);
    GPIOIntClear(GPIOA3_BASE, 0x80);
    GPIOIntEnable(GPIOA3_BASE, 0x80);


    adcSetup();
    configTA1();  //countdown timer
    inAmpSetup();


    //
    // Reset SPI
    //
    MAP_SPIReset(GSPI_BASE);

    //
    // Configure SPI interface
    //
    MAP_SPIConfigSetExpClk(GSPI_BASE,MAP_PRCMPeripheralClockGet(PRCM_GSPI),
                     SPI_IF_BIT_RATE,SPI_MODE_MASTER,SPI_SUB_MODE_2,
                     (SPI_SW_CTRL_CS |
                     SPI_4PIN_MODE |
                     SPI_TURBO_OFF |
                     SPI_CS_ACTIVELOW |
                     SPI_WL_16));

    //
    // Enable SPI for communication
    //
    MAP_SPIEnable(GSPI_BASE);

    UART_PRINT("Enabled SPI Interface in Master Mode\n\r");
    //
    // Send the strings to slave. Chip Select(CS) needs to be
    // asserted at start of transfer and deasserted at the end.
    //

    mode=1;//selectMode();
    while(true){
        count=0;//counter for each fequency change
        startFreq=10;//getStartFreq();
        endFreq=10000;//getEndFreq();
        stepFreq=10;//getStepFreq();
        freq=endFreq;//START FROM FINAL FREQUENCY TO BE ABLE TO ADJUST GAIN FASTER
        periodus=(1e6)/freq;
        switch(mode){
        case 1://first change sinusoid,THEN change input to ADC to avoid overvoltage
            reset0=0b0000000000000000;  //set the mode at this reset de-assertion command
            startatFreq(freq);
            GPIOPinWrite(GPIOA3_BASE, 0x40,0);   //MOSFET OFF
            // FOR WHEN WE WANT TO CHANGE TO ADC FOR TRIANGLE & SINUSOIDAL:
            // Configure PIN_57 for ADC0 ADC_CH0
            //
            PinTypeADC(PIN_57, PIN_MODE_255);
            //
            // Configure PIN_59 for ADC0 ADC_CH2
            //
            PinTypeADC(PIN_59, PIN_MODE_255);
            ADCChannelEnable(ADC_BASE, ADC_CH_0);//open adc as per https://e2e.ti.com/support/wireless-connectivity/wifi/f/968/t/814808?CC3220S-CC3220S-ADC
            ADCChannelEnable(ADC_BASE, ADC_CH_2);
            break;
        case 2:
            reset0=0b0000000000000010;
            startatFreq(freq);
            GPIOPinWrite(GPIOA3_BASE, 0x40,0);
            PinTypeADC(PIN_57, PIN_MODE_255);
            PinTypeADC(PIN_59, PIN_MODE_255);
            ADCChannelEnable(ADC_BASE, ADC_CH_0);//open adc as per https://e2e.ti.com/support/wireless-connectivity/wifi/f/968/t/814808?CC3220S-CC3220S-ADC
            ADCChannelEnable(ADC_BASE, ADC_CH_2);
            break;
        case 3:
            //first change to GPIO,THEN change to square to avoid overvoltage in input pins
            ADCChannelDisable(ADC_BASE, ADC_CH_0);//close adc as per https://e2e.ti.com/support/wireless-connectivity/wifi/f/968/t/814808?CC3220S-CC3220S-ADC
            ADCChannelDisable(ADC_BASE, ADC_CH_2);
            // Configure PIN_57 for GPIO Input
            //
            PinTypeGPIO(PIN_57, PIN_MODE_0, false);
            GPIODirModeSet(GPIOA0_BASE, 0x4, GPIO_DIR_MODE_IN);

            //
            // Configure PIN_59 for GPIO Input
            //
            PinTypeGPIO(PIN_59, PIN_MODE_0, false);
            GPIODirModeSet(GPIOA0_BASE, 0x10, GPIO_DIR_MODE_IN);

            reset0=0b0000000000101000;
            startatFreq(freq);
            GPIOPinWrite(GPIOA3_BASE, 0x40,1<<6);    //MOSFET ON. insert to the position of bit6
            break;
        default:
            Report("Error reading mode,defaulting to sinusoidal\n\r");
            reset0=0b0000000000000000;  //set the mode at this reset de-assertion command
            startatFreq(freq);
            GPIOPinWrite(GPIOA3_BASE, 0x40,0);   //MOSFET OFF
            PinTypeADC(PIN_57, PIN_MODE_255);
            PinTypeADC(PIN_59, PIN_MODE_255);
        }

        findInitialGain();//find the starting gain in the max frequency.
        while(freq>=startFreq){   //CLK FREQ is 20MHZ
            periodus=(1e6)/freq;
            startatFreq(freq);
            //
            // Report to the user
            //
            UART_PRINT("Sweeping @ freq :%dHz\n\r",freq);

            mincounter1=0;//todo
            mincounter0=0;

            clipped=false;
            minUnsmoothed1=4095;//start maximized,finish with acquired min value after each measurement.
            minUnsmoothed0=4095;
            maxUnsmoothed1=0;//start minimized,finish with acquired max value after each measurement.
            maxUnsmoothed0=0;
            minTimestamp1=0;//timestamps for edge time detection
            minTimestamp0=0;
            adcIndex1=0;
            adcIndex0=0;
            TA1running=true;
            if(freq<800){//todo: tweak this if for higher freqs maybe
                periodsToScan=1.2;
                TimerLoadSet(TIMERA1_BASE,TIMER_A,MILLISECONDS_TO_TICKS(periodsToScan*1000.0/freq)); //  1/freq = 1 period.
            }
            else{
                periodsToScan=7;
                TimerLoadSet(TIMERA1_BASE,TIMER_A,MILLISECONDS_TO_TICKS(periodsToScan*1000.0/freq)); //  1/freq = 1 period.   TODO:REDUCE later 10X??
            }
            TimerEnable(TIMERA1_BASE,TIMER_A);
            clearAdc();// clear data
            enableADCints();
            ADCEnable(ADC_BASE);    //START TO MEASURE
            while(TA1running==true);
            disableADCints();
            //removeFirstValues();//TODO:use??

            if((minUnsmoothed1<800)||(clipped==true)){
                D=D-1;//decrease gain to avoid clipping.
                if(D<0){
                    D=0;
                }
                changeGain(D);
            }
            if(minUnsmoothed1>1650){//for inductive loads. MAYBE NOT NEEDED. 1st order circuits with inductors dont change significanlty in the frequencies we use. need to go to MHZ frequencies.
                D=D+1;
                if(D>7){
                    D=7;
                }
                changeGain(D);
            }

            //32bit timer init for measuring...
            Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
            TimerControlStall(TIMERA0_BASE, TIMER_A,true);  //enable timer stall on debug breakpoint
            TimerLoadSet(TIMERA0_BASE,TIMER_A,0xffffffff);
            TimerEnable(TIMERA0_BASE,TIMER_A);
            x=TimerValueGet(TIMERA0_BASE, TIMER_A); //time reference.warning this method of time measurement can be max ~55s

                Evaluate();

            y=tdif(x);  //us
            ms=y/1000;  //ms
            TimerDisable(TIMERA0_BASE,TIMER_A);


/*            if(freq<endFreq){
                waitForEnter();
            }*/

            //todo:this could be source of error for extremely inductive loads which will increase gain in lower freqs
            if(count==0){
                noRolloffFreq=(float)GBWP/gain/10;    //calculate this from gain used in the max used frequency. I assume zero rolloff happens in BW/10.
            }
            count+=1;
            freq-=stepFreq; //next frequency
        }
        finalSmoothingMedian();//SMOOTHEN PHASE DIFF
        finalSmoothingAverage();//FURTHER SMOOTHEN USING ROLLING AVG FILTER
        finalSmoothingImpedance();//SMOOTHEN imp USING THE ROLLING MEDIAN FILTER
        //tests...
/*        if(mode==1){
            mode=2;
        }
        else if (mode==2){
            mode=3;
        }
        else{
            mode=1;
        }*/

        UART_PRINT("0 Rolloff max frequency for this sample: %.1f\n\r",noRolloffFreq);
        UART_PRINT("Sweep finished\n\r");
    }

    while(1)
    {

    }

}


