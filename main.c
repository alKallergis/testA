
// Standard includes
#include <string.h>
#include <stdlib.h>

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
#define adcSamplesNumber 20000   //to trace a whole period @10hz, 6250samples are needed.


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
unsigned short controlReg1=0b0010000100000000;    //reset=1,two word write follows
unsigned short freq0Msbs;
unsigned short freq0Lsbs;
unsigned short reset1=0b0000000100000000;
unsigned short reset0=0b0000000000000000;
unsigned short valAdc1[adcSamplesNumber][2],valAdc2[adcSamplesNumber][2],test1;
unsigned long x;
long spiRet;
float ms,y;
unsigned char i2cBuf[TR_BUFF_SIZE];

//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static void BoardInit();
int getStartFreq(void);
int getEndFreq(void);
int getStepFreq(void);
int selectMode(void);
void selectGain(void);
void waitForEnter(void);
float tdif(unsigned long int m);
void configTA2(void);
void adcSetup(void);
void digResSetup(void);
static void adint3(void);
static void adint1(void);
static void countdownTimerInt(void);
static tBoolean TA1running;
static int adcIndex1,adcIndex3;
void configTA1(void);
void start12_5hz(void);
void startatFreq(float freq );

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

void selectGain(){
    int i2cret=0;
    int iInput = 0;
    char acCmdStore[50];
    int lRetVal;
    unsigned short D;//AD5272 digital wiper value
    float ohm;
    unsigned int iohm;
    UART_PRINT("Select gain: \n\r1- g=1.494 \n\r2- g=2.997 \n\r3- g=10,02 \n\r4- g=102.07 \n\r5- g=506.262");
    //      D=1023: G=1.494   D=253: G=~2.997,  D=56: ~5474ohm,g=10,02 ,  D=5:~489ohm,g=~102.07,  D=1:~97ohm, g=506.262,
    do
    {
        lRetVal = GetCmd(acCmdStore, sizeof(acCmdStore));
        if (lRetVal==0){UART_PRINT("Wrong input,try again");}
        else{
            iInput  = (int)strtoul(acCmdStore,0,10);
            if(iInput<=0 || iInput>5){
                  UART_PRINT("Wrong input,try again");
                }
            else break;
        }
    }while(true);
    switch(iInput){
    case 1:
        D=1023;
        break;
    case 2:
        D=253;
        break;
    case 3:
        D=56;
        break;
    case 4:
        D=5;
        break;
    case 5:
        D=1;
        break;
    default:
        Report("Error reading gain,defaulting to 102\n\r");
        D=5;
    }
    ohm=D*100000.0/1023;
    iohm=ohm;
    i2cBuf[1]=D;
    i2cBuf[0]=(D>>8)|0x04;
    i2cret=I2C_IF_Write(0X2E,&i2cBuf,2,1);   //WRITE RDAC
}


int selectMode(){
    int iInput = 0;
    char acCmdStore[50];
    int lRetVal;
    UART_PRINT("Give mode and press enter for measurement to start:\n\r1-sinusoidal  2-triangle  3-square wave\n\r");
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


void start12_5hz(){
    float freq=12.5;
    unsigned int frequencyReg=(freq*0x10000000)/20000000.0;    //make calculation as per datasheet.(mclk=20Mhz)
    freq0Lsbs= (frequencyReg&0x3fff) | 0b0100000000000000 ;//keep the 14 LSBs and set the 2 msbs according to datasheet
    freq0Msbs=  (frequencyReg>>14) | 0b0100000000000000 ;  //keep 14 msbs,set 2 msbs
    GPIOPinWrite(GPIOA3_BASE, 0x40,1<<6);    //MOSFET ON. insert to the position of bit6
    reset0=0b0000000000101000;
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
    //
    // Report to the user
    //
    UART_PRINT("Outputting square @ freq :12.5Hz\n\r Default gain=102.07\n\r");
}

void startatFreq(float freq ){

    unsigned int frequencyReg=(freq*0x10000000)/20000000.0;    //get frequency via UART, and make calculation as per datasheet.(mclk=20Mhz)
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
    i2cBuf[1]=5;
    i2cBuf[0]=(5>>8)|0x04;
    i2cret=I2C_IF_Write(0X2E,&i2cBuf,2,1);   //WRITE RDAC FOR GAIN=~102 FOR STARTERS

}

void adcSetup(){
    // clear data
    while(ADCFIFOLvlGet(ADC_BASE, ADC_CH_3)) {
        ADCFIFORead(ADC_BASE, ADC_CH_3);}
    while(ADCFIFOLvlGet(ADC_BASE, ADC_CH_1)) {
        ADCFIFORead(ADC_BASE, ADC_CH_1);}

// initialize ADC on channel 3 and 1
    ADCDisable(ADC_BASE);
    ADCTimerConfig(ADC_BASE,0x1ffff);  //or 20000?
    ADCTimerEnable(ADC_BASE);
    ADCIntClear(ADC_BASE, ADC_CH_3,0xf);
    ADCIntEnable(ADC_BASE, ADC_CH_3,ADC_FIFO_FULL);
    ADCIntRegister(ADC_BASE, ADC_CH_3,adint3);
    ADCIntClear(ADC_BASE, ADC_CH_1,0xf);
    ADCIntEnable(ADC_BASE, ADC_CH_1,ADC_FIFO_FULL);
    ADCIntRegister(ADC_BASE, ADC_CH_1,adint1);
    ADCChannelEnable(ADC_BASE, ADC_CH_3);
    ADCChannelEnable(ADC_BASE, ADC_CH_1);
}

static void adint1()
{   unsigned long Status = ADCIntStatus(ADC_BASE, ADC_CH_1);
    ADCIntClear(ADC_BASE, ADC_CH_1,0x1f);
    Status = ADCIntStatus(ADC_BASE, ADC_CH_1);
    //while(!ADCFIFOLvlGet(ADC_BASE, ADC_CH_1));    //no need
    valAdc1[adcIndex1][0] = ADCFIFORead(ADC_BASE, ADC_CH_1) & 0x3FFF;
    valAdc1[adcIndex1][0] = valAdc1[adcIndex1][0] >> 2;

    valAdc1[adcIndex1][1]=ADCTimerValueGet(ADC_BASE);  //GET TIMESTAMP. 640 timer ticks is a sampling cycle.
    adcIndex1++;
/*        if(adcIndex==(adcSamplesNumber-1)){   //maybe should disable in adint3 only,where the final point is taken
        measure=false;
        ADCDisable(ADC_BASE);
    }*/
}

static void adint3()
{   unsigned long Status = ADCIntStatus(ADC_BASE, ADC_CH_3);
    ADCIntClear(ADC_BASE, ADC_CH_3,0x1f);
    //while(!ADCFIFOLvlGet(ADC_BASE, ADC_CH_3));    //no need
    valAdc1[adcIndex1][0] = ADCFIFORead(ADC_BASE, ADC_CH_3) & 0x3FFF;
    valAdc1[adcIndex1][0] = valAdc1[adcIndex1][0] >> 2;

    valAdc1[adcIndex1][1]=ADCTimerValueGet(ADC_BASE);
    adcIndex1++;
}

static void countdownTimerInt()
{
    if (TimerIntStatus(TIMERA1_BASE,true)==0x1){   // timeout int has happened
        //unsigned int x5=TimerValueGet(TIMERA1_BASE, TIMER_A);
        TimerIntClear(TIMERA1_BASE,TIMER_TIMA_TIMEOUT);
        TA1running=false;
    }
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
    int i;//clock interval counter
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


/*    // FOR WHEN WE WANT TO CHANGE TO ADC FOR TRIANGLE & SINUSOIDAL:
    // Configure PIN_57 for ADC0 ADC_CH0
    //
    PinTypeADC(PIN_57, PIN_MODE_255);
    //
    // Configure PIN_59 for ADC0 ADC_CH2
    //
    PinTypeADC(PIN_59, PIN_MODE_255);*/


    //OUTPUT 20Mhz clock from p64:
    configTA2();

    adcSetup();
    configTA1();  //countdown timer
    digResSetup();


    unsigned int frequencyReg=0;
    float freq=0;
    int endFreq=0;
    int stepFreq=0;
    int mode=0;

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
    start12_5hz();
    while(true){
        selectGain();
        mode=selectMode();

        switch(mode){
        case 1:
            GPIOPinWrite(GPIOA3_BASE, 0x40,0);   //MOSFET OFF
            reset0=0b0000000000000000;  //set the mode at this reset de-assertion command
            break;
        case 2:
            GPIOPinWrite(GPIOA3_BASE, 0x40,0);
            reset0=0b0000000000000010;
            break;
        case 3:                                     //MOSFET ON
            GPIOPinWrite(GPIOA3_BASE, 0x40,1<<6);    //insert to the position of bit6
            reset0=0b0000000000101000;
            break;
        default:
            Report("Error reading mode,defaulting to sinusoidal\n\r");
            GPIOPinWrite(GPIOA3_BASE, 0x40,0);
            reset0=0b0000000000000000;
        }
        freq=12.5;
        startatFreq(freq);
        UART_PRINT("Sweeping @ freq :%fHz\n\r",freq);
        //delay 2,5mins in 1st time...
/*        for (i=1;i<=3;i++){
            TA1running=true;
            TimerLoadSet(TIMERA1_BASE,TIMER_A,MILLISECONDS_TO_TICKS(1000.0*50));
            TimerEnable(TIMERA1_BASE,TIMER_A);
            while(TA1running==true);
        }*/

        while(1){


            //go for 36*50s=30min...
/*            for (i=1;i<=35;i++){
                TA1running=true;
                TimerLoadSet(TIMERA1_BASE,TIMER_A,MILLISECONDS_TO_TICKS(1000.0*50));
                TimerEnable(TIMERA1_BASE,TIMER_A);
                while(TA1running==true);
            }*/
            TA1running=true;
            TimerLoadSet(TIMERA1_BASE,TIMER_A,MILLISECONDS_TO_TICKS(1000.0*4));
            TimerEnable(TIMERA1_BASE,TIMER_A);
            while(TA1running==true);
            //1sec before finighing...
            TA1running=true;
            TimerLoadSet(TIMERA1_BASE,TIMER_A,MILLISECONDS_TO_TICKS(1000));
            TimerEnable(TIMERA1_BASE,TIMER_A);

            if(freq==12.5){
                freq=20;
            }
            else if(freq==20){
                freq=50;
            }
            else if(freq==50){
                freq=100;
            }
            else if(freq==100){
                freq=500;
            }
            else if(freq==500){
                freq=1000;
            }
            else if(freq==1000){
                freq=100000;
            }
            else if(freq==100000){
                freq=12.5;
            }
            startatFreq(freq);
            //
            // Report to the user
            //
            UART_PRINT("Sweeping @ freq :%fHz\n\r",freq);
            while(TA1running==true);

        }
        UART_PRINT("Sweep finished\n\r");
    }

    while(1)
    {

    }

}

