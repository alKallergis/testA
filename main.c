//now modified for having stable gain throughout sweep.
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
#define GBWP  1500000   //AD8226 GBWP
#define maxSweepFCount     5000 //TODO:5000 max measurements for now

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
unsigned short controlReg1=0b0010000100000000;    //reset=1,two word write follows
unsigned short freq0Msbs;
unsigned short freq0Lsbs;
unsigned short reset1=0b0000000100000000;
unsigned short reset0=0b0000000000000000;
int startFreq, freq,endFreq;
long long int stampDiff,minsmoothedTimestamp1,minsmoothedTimestamp0,mincounter0,mincounter1;
static unsigned short valAdc1[adcSamplesNumber],valAdc0[adcSamplesNumber],temp[adcSamplesNumber],minUnsmoothed1,minUnsmoothed0,maxUnsmoothed0,maxUnsmoothed1;
static unsigned short minValue0,maxValue0,minValue1,maxValue1;
static unsigned short minValuetimestamp[maxSweepFCount],maxValuetimestamp[maxSweepFCount],count;//LET 5000 be the max bumber of frequencies todo change these to single variable later to savce space
static float pk_pk_phaseDiff[maxSweepFCount],pk_pk1[maxSweepFCount],temp1[maxSweepFCount],ohm,gain,impedance[maxSweepFCount],noRolloffFreq;
unsigned long x,x1;
float ms,y,periodus,periodsToScan;
unsigned char i2cBuf[TR_BUFF_SIZE];
unsigned char smoothingInterval; //goes into smoothenAndEvaluate function
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
void configTA2(void);
void adcSetup(void);
void digResSetup(void);
static void adint3(void);
static void adint1(void);
static void adint0(void);
static void adint2(void);
static void countdownTimerInt(void);
void configTA1(void);
void startatFreq(float);
void int31(void);
void smoothenAndEvaluate(void);
void finalSmoothingMedian(void);
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


void changeGain(unsigned short d){//d=AD5272 digital wiper value
    int i2cret=0;
    unsigned int iohm;
    ohm=d*100000.0/1023;
    iohm=ohm;
    gain=49400.0/ohm+1;
    i2cBuf[1]=d;
    i2cBuf[0]=(d>>8)|0x04;
    i2cret=I2C_IF_Write(0X2E,&i2cBuf,2,1);   //WRITE RDAC
    MAP_UtilsDelay(1000);               //TODO:TWEAK. wait for DIGRES. maybe use delay timer like below,...
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

    unsigned int frequencyReg=(Freq*0x10000000)/20000000.0;    //get frequency via UART, and make calculation as per datasheet.(mclk=20Mhz)
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
    MAP_UtilsDelay(1000);               //TODO:TWEAK. wait for DDS. datasheet says wait 8 MCLK cycles. Worst case if 1Mhz,wait 640 cc3200 cycles.maybe use delay timer like below,...

}



//new. Median filter according to wikipedia. Change each value according to its neigborhood's median. No need to change the timestamps.
void smoothenAndEvaluate(){
    //TODO:testing for smoothingInterval=3,increase if needed. needs to BE ODD.we are taking the MEDIAN.
    char i3,G=3;
    int i1,i2;
    unsigned  short smoothWindow[300],temp2;//!!let smoothingInterval<300

    //smoothingInterval=3;
    smoothingInterval=G*log(2*endFreq/freq);//todo:change endFreq here to 4000 since it works, permanently
    if(smoothingInterval<3) smoothingInterval=3;//limit to above 3
    if((smoothingInterval%2)==0) smoothingInterval++;//needs to be odd






    for(i1=smoothingInterval/2;i1<(adcIndex0-smoothingInterval/2);i1++){
        i3=0;
        //copy values to buffer to sort them:
        for(i2=0;i2<smoothingInterval;i2++){
            smoothWindow[i2]=valAdc0[i1-smoothingInterval/2+i2];
        }
        while(i3<smoothingInterval-1){
        i2=0;
            while(i2<smoothingInterval-1){
                if (temp[i2]>temp[i2+1]){
                    temp2=smoothWindow[i2];   //sort values with their respective timestamps
                    smoothWindow[i2]=smoothWindow[i2+1];
                    temp[i2+1]=temp2;
                }
            i2++;
            }
        i3++;
        }
        temp[i1]=smoothWindow[smoothingInterval/2];//save median

    }
    //fill array borders with first&last filtered values:
    for(i1=0;i1<adcIndex0;i1++){
        if (i1<=smoothingInterval/2){
            temp[i1]=temp[smoothingInterval/2+1];
        }
        else if (i1>=adcIndex0-smoothingInterval/2){
            temp[i1]=temp[adcIndex0-smoothingInterval/2-1];
        }
    }


    //find edges in filtered array:
    minValue0=temp[0];
    maxValue0=temp[0];
    for(i1=1;i1<adcIndex0;i1++){
        if(temp[i1]<minValue0){
            minValue0=temp[i1];
            if(8.0*i1<periodus){       //use only the first sampled period of the wave for finding phase diff todo:change 8.0 to 8 and see what happens?
                minsmoothedTimestamp0=i1;//todo change this type to simple unsigned int
            }

            mincounter0++;
        }
        if(temp[i1]>maxValue0){
            maxValue0=temp[i1];
        }
    }





    for(i1=smoothingInterval/2;i1<(adcIndex1-smoothingInterval/2);i1++){
        i3=0;
        //copy values to buffer to sort them:
        for(i2=0;i2<smoothingInterval;i2++){
            smoothWindow[i2]=valAdc1[i1-smoothingInterval/2+i2];
        }
        while(i3<smoothingInterval-1){
        i2=0;
            while(i2<smoothingInterval-1){
                if (smoothWindow[i2]>smoothWindow[i2+1]){
                    temp2=smoothWindow[i2];   //sort values with their respective timestamps
                    smoothWindow[i2]=smoothWindow[i2+1];
                    smoothWindow[i2+1]=temp2;
                }
            i2++;
            }
        i3++;
        }
        temp[i1]=smoothWindow[smoothingInterval/2];//save median

    }
    //fill array borders with first&last filtered values:
    for(i1=0;i1<adcIndex1;i1++){
        if (i1<=smoothingInterval/2){
            temp[i1]=temp[smoothingInterval/2+1];
        }
        else if (i1>=adcIndex1-smoothingInterval/2){
            temp[i1]=temp[adcIndex1-smoothingInterval/2-1];
        }
    }

    //find edges in filtered array:
    minValue1=temp[0];
    maxValue1=temp[0];
    for(i1=1;i1<adcIndex1;i1++){
        if(temp[i1]<minValue1){
            minValue1=temp[i1];
            if(8.0*i1<periodus){       //use only the first sampled period of the wave for finding phase diff
                minsmoothedTimestamp1=i1;//todo change this type to simple unsigned int
            }
            mincounter1++;
        }
        if(temp[i1]>maxValue1){
            maxValue1=temp[i1];
        }
    }






    pk_pk1[count]= (maxValue1-minValue1)*1.467/4096.0;//in volts TODO:try the other formula i have written down too.

    if(abs(minsmoothedTimestamp1-minsmoothedTimestamp0)<abs(minsmoothedTimestamp1-minsmoothedTimestamp0+(adcIndex1/periodsToScan))){//from notes:go with the smallest abs value of phase difference.
        stampDiff=minsmoothedTimestamp1-minsmoothedTimestamp0;
    }
    else{
        stampDiff=minsmoothedTimestamp1-minsmoothedTimestamp0+(adcIndex1/periodsToScan);//(from notes)add 1 period in case the 1st edge didn't get picked at the dds source signal,causing it to show 1 period later.
    }
    pk_pk_phaseDiff[count]=360.0*freq*stampDiff*8/1e6;//todo adc timer cycle is 16us,sample for each channel every 8us
    impedance[count]=pk_pk1[count]*1e5/2/gain;
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

/*    //restore filtered array to original array
    for(i1=0;i1<count;i1++){
        pk_pk_phaseDiff[i1]=temp1[i1];
    }*/

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

/*    //restore filtered array to original array
    for(i1=0;i1<count;i1++){
        pk_pk_phaseDiff[i1]=temp1[i1];
    }*/

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

void configTA2(){
    TimerConfigure(TIMERA2_BASE,(TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM));
    TimerLoadSet(TIMERA2_BASE,TIMER_B,0x0003); //50ns period
    TimerPrescaleSet(TIMERA2_BASE,TIMER_B,0x00);
    TimerMatchSet(TIMERA2_BASE, TIMER_B,0x0001);
    TimerPrescaleMatchSet(TIMERA2_BASE, TIMER_B,0x00);  //switch to low in 25ns
    //TimerControlStall(TIMERA2_BASE, TIMER_BOTH,true);
    TimerEnable(TIMERA2_BASE,TIMER_BOTH);
}

void digResSetup(void){
    int i2cret=0;
    i2cBuf[1]=0X02;
    i2cBuf[0]=0X1c;
    i2cret=I2C_IF_Write(0X2E,&i2cBuf,2,1);  //WRITE CONTROL REG FOR RDAC ENABLE
    i2cBuf[1]=1023; //=100K,smallest GAIN=~1.494
    i2cBuf[0]=(1023>>8)|0x04;
    i2cret=I2C_IF_Write(0X2E,&i2cBuf,2,1);   //WRITE RDAC FOR smallest GAIN=~1.494

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

// initialize ADC on channel 3 and 1
    ADCDisable(ADC_BASE);
    ADCTimerConfig(ADC_BASE,0x1ffff);  //or 20000?
    ADCTimerEnable(ADC_BASE);
    ADCIntClear(ADC_BASE, ADC_CH_3,0xf);
    ADCIntEnable(ADC_BASE, ADC_CH_3,ADC_FIFO_FULL);
    ADCIntRegister(ADC_BASE, ADC_CH_3,adint3);
    ADCIntClear(ADC_BASE, ADC_CH_0,0xf);
    ADCIntEnable(ADC_BASE, ADC_CH_0,ADC_FIFO_FULL);
    ADCIntRegister(ADC_BASE, ADC_CH_0,adint0);
    ADCIntClear(ADC_BASE, ADC_CH_1,0xf);
    ADCIntEnable(ADC_BASE, ADC_CH_1,ADC_FIFO_FULL);
    ADCIntRegister(ADC_BASE, ADC_CH_1,adint1);
    ADCIntClear(ADC_BASE, ADC_CH_2,0xf);
    ADCIntEnable(ADC_BASE, ADC_CH_2,ADC_FIFO_FULL);
    ADCIntRegister(ADC_BASE, ADC_CH_2,adint2);
    ADCChannelEnable(ADC_BASE, ADC_CH_3);
    ADCChannelEnable(ADC_BASE, ADC_CH_1);
    ADCChannelEnable(ADC_BASE, ADC_CH_0);
    ADCChannelEnable(ADC_BASE, ADC_CH_2);
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
    //Status = ADCIntStatus(ADC_BASE, ADC_CH_1);
    //while(!ADCFIFOLvlGet(ADC_BASE, ADC_CH_1));    //no need

/*    if(adcIndex1>20000){
        x1++; //TODO: DUMMY INSTRUCTION
        ADCDisable(ADC_BASE);    //STop in case it's measuring
        ADCIntDisable(ADC_BASE, ADC_CH_1,ADC_FIFO_FULL);    //I found it needs to be stopped because it's giving erratic interrupts that surpass the priority of countdown timer,hindering it from stopping the ADC..

    }*/
    //else{
        if (TA1running==true){    //this doesnt cause issues.
            valAdc1[adcIndex1] = ADCFIFORead(ADC_BASE, ADC_CH_1) & 0x3FFF;
            valAdc1[adcIndex1] = valAdc1[adcIndex1] >> 2;


            if(minUnsmoothed1>valAdc1[adcIndex1]){
                minUnsmoothed1=valAdc1[adcIndex1];
            }
            adcIndex1++;
            /*        if(adcIndex==(adcSamplesNumber-1)){   //maybe should disable in adint3 only,where the final point is taken
            measure=false;
            ADCDisable(ADC_BASE);
            }*/

    }
}

static void adint3()
{   //unsigned long Status = ADCIntStatus(ADC_BASE, ADC_CH_3);
    ADCIntClear(ADC_BASE, ADC_CH_3,0x1f);
/*    if(adcIndex1>20000){
        x1++;
        ADCDisable(ADC_BASE);    //STop in case it's measuring
        ADCIntDisable(ADC_BASE, ADC_CH_3,ADC_FIFO_FULL);//I found it needs to be stopped because it's giving erratic interrupts that surpass the priority of countdown timer,hindering it from stopping the ADC..
    }*/
    //else{
    if (TA1running==true){    //this doesnt cause issues.
        valAdc1[adcIndex1] = ADCFIFORead(ADC_BASE, ADC_CH_3) & 0x3FFF;
        valAdc1[adcIndex1] = valAdc1[adcIndex1] >> 2;

        adcIndex1++;
    }
}
static void adint0()
{   //unsigned long Status = ADCIntStatus(ADC_BASE, ADC_CH_3);
    ADCIntClear(ADC_BASE, ADC_CH_0,0x1f);
/*    if(adcIndex1>20000){
        x1++;
        ADCDisable(ADC_BASE);    //STop in case it's measuring
        ADCIntDisable(ADC_BASE, ADC_CH_3,ADC_FIFO_FULL);//I found it needs to be stopped because it's giving erratic interrupts that surpass the priority of countdown timer,hindering it from stopping the ADC..
    }*/
    //else{
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
/*    if(adcIndex1>20000){
        x1++;
        ADCDisable(ADC_BASE);    //STop in case it's measuring
        ADCIntDisable(ADC_BASE, ADC_CH_3,ADC_FIFO_FULL);//I found it needs to be stopped because it's giving erratic interrupts that surpass the priority of countdown timer,hindering it from stopping the ADC..
    }*/
    //else{
    if (TA1running==true){    //this doesnt cause issues.
        valAdc0[adcIndex0] = ADCFIFORead(ADC_BASE, ADC_CH_2) & 0x3FFF;
        valAdc0[adcIndex0] = valAdc0[adcIndex0] >> 2;



        adcIndex0++;
    }
}

static void countdownTimerInt()
{
    if (TimerIntStatus(TIMERA1_BASE,true)==0x1){   // timeout int has happened
        unsigned int x5=TimerValueGet(TIMERA1_BASE, TIMER_A);
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
    unsigned short D=0,Dprev=0;
    int stepFreq=0;
    int mode=0;
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


    //OUTPUT 20Mhz clock from p64:
    configTA2();

    adcSetup();
    configTA1();  //countdown timer
    digResSetup();


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

    while(true){
        count=0;//counter for each fequency change
        mode=3;//selectMode();
        startFreq=10;//getStartFreq();
        endFreq=4000;//getEndFreq();
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
            break;
        case 2:
            reset0=0b0000000000000010;
            startatFreq(freq);
            GPIOPinWrite(GPIOA3_BASE, 0x40,0);
            PinTypeADC(PIN_57, PIN_MODE_255);
            PinTypeADC(PIN_59, PIN_MODE_255);
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
            GPIOPinWrite(GPIOA3_BASE, 0x40,1<<6);    //insert to the position of bit6
            break;
        default:
            Report("Error reading mode,defaulting to sinusoidal\n\r");
            reset0=0b0000000000000000;  //set the mode at this reset de-assertion command
            startatFreq(freq);
            GPIOPinWrite(GPIOA3_BASE, 0x40,0);   //MOSFET OFF
            PinTypeADC(PIN_57, PIN_MODE_255);
            PinTypeADC(PIN_59, PIN_MODE_255);
        }

        MAP_UtilsDelay(MILLISECONDS_TO_TICKS(200));//DALAY A BIT FOR THE FIRST WAVE TO SETTLE.
        D=1023;
        Dprev=D;
        while(1){
            //MAP_UtilsDelay(10000);               //wait for DDS: datasheet says wait 8 MCLK cycles. Worst case if 1Mhz,wait 640 cc3200 cycles:
/*            TA1running=true;
            TimerLoadSet(TIMERA1_BASE,TIMER_A,79); //  1us,worst case
            TimerEnable(TIMERA1_BASE,TIMER_A);
            while(TA1running==true);*/
            changeGain(D);
            minUnsmoothed1=4095;//start maximized,finish with acquired min value after each measurement.
            minUnsmoothed0=4095;
            clipped=false;
            adcIndex1=0;
            adcIndex0=0;
            TA1running=true;
            if(freq<100){//todo: tweak this if for higher freqs maybe
                periodsToScan=1.2;
                TimerLoadSet(TIMERA1_BASE,TIMER_A,MILLISECONDS_TO_TICKS(periodsToScan*1000.0/freq)); //  1/freq = 1 period.
            }
            else{
                periodsToScan=10;
                TimerLoadSet(TIMERA1_BASE,TIMER_A,MILLISECONDS_TO_TICKS(periodsToScan*1000.0/freq)); //  1/freq = 1 period.   TODO:REDUCE later 10X??
            }
            TimerEnable(TIMERA1_BASE,TIMER_A);
            clearAdc();
            ADCEnable(ADC_BASE);    //START TO MEASURE
            while(TA1running==true);

            if((minUnsmoothed1<1640)||(clipped==true)){
                D=Dprev;
                break;
            }
            else{
                Dprev=D;
                D=D-1;//BE INCREASING GAIN UNTIL CLIPPED.
                if(D<3){
                    D=3;
                    UART_PRINT("Sample very conductive\n\r");
                    break;
                }
            }
        }
        changeGain(D);
        while(freq>=startFreq){   //CLK FREQ is 20MHZ
            periodus=(1e6)/freq;
            startatFreq(freq);//todo
            //
            // Report to the user
            //
            UART_PRINT("Sweeping @ freq :%dHz\n\r",freq);


            minUnsmoothed1=4095;//start maximized,finish with acquired min value after each measurement.
            minUnsmoothed0=4095;
            clipped=false;
            adcIndex1=0;
            adcIndex0=0;
            TA1running=true;
            if(freq<100){//todo: tweak this if for higher freqs maybe
                periodsToScan=1.2;
                TimerLoadSet(TIMERA1_BASE,TIMER_A,MILLISECONDS_TO_TICKS(periodsToScan*1000.0/freq)); //  1/freq = 1 period.
            }
            else{
                periodsToScan=10;
                TimerLoadSet(TIMERA1_BASE,TIMER_A,MILLISECONDS_TO_TICKS(periodsToScan*1000.0/freq)); //  1/freq = 1 period.   TODO:REDUCE later 10X??
            }
            TimerEnable(TIMERA1_BASE,TIMER_A);
            clearAdc();
            ADCEnable(ADC_BASE);    //START TO MEASURE
            while(TA1running==true);

            if((minUnsmoothed1<800)||(clipped==true)){
                D=D*2;
                if(D>1023){
                    D=1023;
                }
            }
            changeGain(D);

            mincounter1=0;//todo
            mincounter0=0;


            minUnsmoothed1=4095;//start maximized,finish with acquired min value after each measurement.
            minUnsmoothed0=4095;
            maxUnsmoothed1=0;//start minimized,finish with acquired max value after each measurement.
            maxUnsmoothed0=0;
            minsmoothedTimestamp1=0;//timestamps for edge time detection
            minsmoothedTimestamp0=0;
            adcIndex1=0;
            adcIndex0=0;
            TA1running=true;
            if(freq<100){//todo: tweak this if for higher freqs maybe
                periodsToScan=1.2;
                TimerLoadSet(TIMERA1_BASE,TIMER_A,MILLISECONDS_TO_TICKS(periodsToScan*1000.0/freq)); //  1/freq = 1 period.
            }
            else{
                periodsToScan=10;
                TimerLoadSet(TIMERA1_BASE,TIMER_A,MILLISECONDS_TO_TICKS(periodsToScan*1000.0/freq)); //  1/freq = 1 period.   TODO:REDUCE later 10X??
            }
            TimerEnable(TIMERA1_BASE,TIMER_A);
            clearAdc();// clear data
            ADCEnable(ADC_BASE);    //START TO MEASURE
            while(TA1running==true);
            //removeFirstValues();//TODO.use??

            //32bit timer init for measuring...
            Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
            TimerControlStall(TIMERA0_BASE, TIMER_A,true);  //enable timer stall on debug breakpoint
            TimerLoadSet(TIMERA0_BASE,TIMER_A,0xffffffff);
            TimerEnable(TIMERA0_BASE,TIMER_A);
            x=TimerValueGet(TIMERA0_BASE, TIMER_A); //time reference.warning this method of time measurement can be max ~55s

                        smoothenAndEvaluate();

            y=tdif(x);  //us
            ms=y/1000;  //ms
            TimerDisable(TIMERA0_BASE,TIMER_A);


/*            if(freq<endFreq){
                waitForEnter();
            }*/
            if(count==0){
                noRolloffFreq=(float)GBWP/gain/10;    //calculate this from gain used in the max used frequency. I assume zero rolloff happens in BW/10.
            }
            count+=1;
            freq-=stepFreq; //next frequency
        }
        finalSmoothingMedian();
        finalSmoothingAverage();
        UART_PRINT("0 Rolloff max frequency for this sample: %.1f\n\r",noRolloffFreq);
        UART_PRINT("Sweep finished\n\r");
    }

    while(1)
    {

    }

}

