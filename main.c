//changed limits for changing gain,used also maxValue1-minValue1 and maxValue1 to change gain.
//changed findInitialGain function to successively find proper gain.
//removed minUnsmoothed1,minUnsmoothed0,maxUnsmoothed0,maxUnsmoothed1 not needed anymore.
//increased delay in line 1235 for motorola phone.
//TODO:CHANGE maxSweepFCount

// Standard includes
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>

// simplelink includes
#include "simplelink.h"
#include "wlan.h"
#include "network_if.h"

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
#include "pin_mux_config.h"
#include "udma_if.h"
#include "common.h"
#ifndef NOTERM
#include "uart_if.h"
#endif

// JSON Parser
#include "jsmn.h"



// Application Master/Slave mode selector macro
//
// MASTER_MODE = 1 : Application in master mode

#define MASTER_MODE      1

#define SPI_IF_BIT_RATE  100000
#define TR_BUFF_SIZE     128
#define adcSamplesNumber 16000   //to trace a whole period @10hz, 6250samples are needed per channel.(twice that for the 2 channels i use per input)
#define GBWP  2700000   //AD8231 GBWP
#define maxSweepFCount     1100 //TODO:10000 max measurements for now,fix this later to ~1100 after we see what happens with the memory.

#define IP_ADDR             0xc0a80167 // 192.168.1.103  doesn't matter in this project.
#define PORT_NUM            5009    //TODO:CHANGE PORT?
#define BUF_SIZE            100

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    SOCKET_CREATE_ERROR = -0x7D0,
    BIND_ERROR = SOCKET_CREATE_ERROR - 1,
    LISTEN_ERROR = BIND_ERROR -1,
    SOCKET_OPT_ERROR = LISTEN_ERROR -1,
    CONNECT_ERROR = SOCKET_OPT_ERROR -1,
    ACCEPT_ERROR = CONNECT_ERROR - 1,
    SEND_ERROR = ACCEPT_ERROR -1,
    RECV_ERROR = SEND_ERROR -1,
    SOCKET_CLOSE_ERROR = RECV_ERROR -1,
    DEVICE_NOT_IN_STATION_MODE = SOCKET_CLOSE_ERROR - 1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;




//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************


extern volatile unsigned long  g_ulStatus;//SimpleLink Status
extern unsigned long  g_ulGatewayIP; //Network Gateway IP address
extern unsigned char  g_ucConnectionSSID; //Connection SSID
extern unsigned char  g_ucConnectionBSSID; //Connection BSSID
unsigned long  g_ulDestinationIp = IP_ADDR;
unsigned int   g_uiPortNum = PORT_NUM,NewSockID;
unsigned char  g_ucConnectionStatus = 0,ucConfigOpt=0;
unsigned char  g_ucSimplelinkstarted = 0;
unsigned long  g_ulIpAddr = 0;
char g_cBsdBuf[BUF_SIZE];

unsigned short controlReg1=0b0010000100000000;    //reset=1,two word write follows
unsigned short freq0Msbs;
unsigned short freq0Lsbs;
unsigned short reset1=0b0000000100000000;
unsigned short reset0=0b0000000000000000;
static int startFreq,stepFreq,freq,endFreq,mode,freqCount,interval;
int minTimestamp1,minTimestamp0,mincounter0,mincounter1;
float stampDiff;
static unsigned short D;
static unsigned short valAdc1[adcSamplesNumber],valAdc0[adcSamplesNumber],minUnsmoothed1,minUnsmoothed0,maxUnsmoothed0,maxUnsmoothed1;
static unsigned short minValue0,maxValue0,minValue1,maxValue1;
static unsigned short count,firstdata;//LET 10000 be the max bumber of frequencies todo change these to single variable later to save space
static float pk_pk_phaseDiff[maxSweepFCount],temp1[maxSweepFCount],ohm,gain,impedance[maxSweepFCount],noRolloffFreq;
//static float pk_pk1[maxSweepFCount];//save space TODO:
unsigned long x,x1;
float ms,y,periodus,periodsToScan;
long spiRet;
static tBoolean TA1running,TA3running;
static tBoolean clipped;
static int adcIndex1,adcIndex0; //TODO:CHECK MAX NUMBER THIS CAN REACH


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
static void countdownTimerInt3(void);
void configTA1(void);
void startatFreq(float);
void int31(void);
void findInitialGain(void);
void Evaluate(void);
void finalSmoothingMedian(void);
void finalSmoothingImpedance(void);
void finalSmoothingAverage(void);
void clearAdc(void);
static void InitializeAppVariables();
void doSingleSweep(void);
void setupSweep(void);
int cellServer(unsigned short);

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//
//! This function initializes the application variables
//!
//! \param[in]    None
//!
//! \return None
//!
//*****************************************************************************
static void InitializeAppVariables()
{
    g_ulStatus = 0;
    g_ulGatewayIP = 0;
    memset(g_ucConnectionSSID,0,sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID,0,sizeof(g_ucConnectionBSSID));
    g_ulDestinationIp = IP_ADDR;
    g_uiPortNum = PORT_NUM;
}


//****************************************************************************
//
//! Confgiures the mode in which the device will work
//!
//! \param iMode is the current mode of the device
//!
//! This function
//!    1. prompt user for desired configuration and accordingly configure the
//!          networking mode(STA or AP).
//!       2. also give the user the option to configure the ssid name in case of
//!       AP mode.
//!
//! \return sl_start return value(int).
//
//****************************************************************************

static int ConfigureMode(int iMode)
{
    char    pcSsidName[33]="spectrumAp";
    char    password[33]="emp1emp1";
    char    val = SL_SEC_TYPE_WPA_WPA2;
    long    lRetVal = -1;
    unsigned short  len = strlen(password);

    lRetVal = sl_WlanSetMode(ROLE_AP);
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SSID, strlen(pcSsidName),
                            (unsigned char*)pcSsidName);
    ASSERT_ON_ERROR(lRetVal);


    lRetVal = sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SECURITY_TYPE, 1, (_u8 *)&val);
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_PASSWORD, len, (_u8 *)password);
    ASSERT_ON_ERROR(lRetVal);

    //UART_PRINT("Device is configured in AP mode\n\r");

    /* Restart Network processor */
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    // reset status bits
    CLR_STATUS_BIT_ALL(g_ulStatus);

    return sl_Start(NULL,NULL,NULL);
}

//*****************************************************************************
//
//! \brief Handler for parsing JSON data
//!
//! \param[in]  ptr - Pointer to http response body data
//!
//! \return 0 on success else error code on failure
//!
//*****************************************************************************
int ParseJSONData(char *ptr)
{
    long lRetVal = 0;
    int noOfToken;
    jsmn_parser parser;
    jsmntok_t   *tokenList;
    char keyString[30],dataString[30];//the strings are pretty small
    unsigned int toklength;
    jsmntok_t key,data;


    /* Initialize JSON PArser */
    jsmn_init(&parser);

    /* Get number of JSON token in stream as we we dont know how many tokens need to pass */
    noOfToken = jsmn_parse(&parser, (const char *)ptr, strlen((const char *)ptr), NULL, 10);
    if(noOfToken <= 0)
    {
        UART_PRINT("Failed to initialize JSON parser\n\r");
        return -1;

    }

    /* Allocate memory to store token */
    tokenList = (jsmntok_t *) malloc(noOfToken*sizeof(jsmntok_t));
    if(tokenList == NULL)
    {
        UART_PRINT("Failed to allocate memory\n\r");
        return -1;
    }

    /* Initialize JSON Parser again */
    jsmn_init(&parser);
    noOfToken = jsmn_parse(&parser, (const char *)ptr, strlen((const char *)ptr), tokenList, noOfToken);
    if(noOfToken < 0)
    {
        UART_PRINT("Failed to parse JSON tokens\n\r");
        lRetVal = noOfToken;
    }
    else
    {
        UART_PRINT("Successfully parsed %ld JSON tokens\n\r", noOfToken);


        int tok;
        for (tok=1;tok<noOfToken;tok+=2){//every 2 tokens is a key token. Token 0 is the outer object.
            key = tokenList[tok];
            toklength = key.end - key.start;
            memcpy(keyString, &ptr[key.start], toklength);
            keyString[toklength] = '\0';


            data = tokenList[tok+1];
            toklength = data.end - data.start;
            memcpy(dataString, &ptr[data.start], toklength);
            dataString[toklength] = '\0';

            if(!strcmp(keyString,"waveform")){
                mode=atoi(dataString);
            }
            if(!strcmp(keyString,"startFreq")){
                startFreq=atoi(dataString);
            }
            if(!strcmp(keyString,"endFreq")){
                endFreq=atoi(dataString);
            }
            if(!strcmp(keyString,"stepFreq")){
                stepFreq=atoi(dataString);
            }
            if(!strcmp(keyString,"interval")){
                interval=atoi(dataString);
            }
        }
        freqCount=(endFreq - startFreq) / stepFreq;

    }


    free(tokenList);

    return lRetVal;
}


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
    MAP_UtilsDelay(1000);               //todo: IS THIS PROBLEM?? wait for DDS. datasheet says wait 8 MCLK cycles. Worst case if 1Mhz,wait 640 cc3200 cycles.

}

void findInitialGain(){
    count=0;//counter for each frequency change
    freq=endFreq;//START FROM FINAL FREQUENCY TO BE ABLE TO ADJUST GAIN FASTER
    startatFreq(freq);
    char i10=0;
    D=0;
    while(i10<100){//successively approach proper gain. 100 loops.
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

        Evaluate();//find edges,only maxValue1,minValue1 interests us here.


        if(((maxValue1-minValue1)>1900)||(minValue1<700)||(maxValue1>3500)||(clipped==true)){   // we have big amplitude.reduce gain. (consider the perturbations from the solution potential that can overdrive the in-amp.)
            D=D-1;//decrease gain to avoid clipping.
            if(D<0){
                D=0;
            }
            changeGain(D);
        }
        if(((maxValue1-minValue1)<200)){//for inductive loads. MAYBE NOT NEEDED. 1st order circuits with inductors dont change significanlty in the frequencies we use. need to go to MHZ frequencies.
            D=D+1;
            if(D>7){
                D=7;
            }
            changeGain(D);
        }
        i10++;
    }
    if(D==7){
        UART_PRINT("Sample very conductive\n\r");
    }

}
//find edges & calculate phase difference and amplitude
void Evaluate(){
    int i1;
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

    //pk_pk1[count]= (maxValue1-minValue1)*1.467/4096.0;//in volts TODO:try the other formula i have written down too.
    impedance[count]=(maxValue1-minValue1)*1.467/4096.0*1e5/1.87/gain;//p-p current is 18.7uA with 33k Radj

}


//Median filter according to wikipedia. Change each value according to its neigborhood's median. smoothen out response waveforms using rolling median
void finalSmoothingImpedance(){
    //smoothen out impedance
        unsigned char smoothingInterval1=5;//todo:test this WITH THE WIFI.
        char i3;
        int i1,i2;
        float smoothWindow[300],temp2;//!!let smoothingInterval<300//TODO DOES THIS OVERFLOW?CHECK.

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
void configTA3(){   //countdown timer A3
    Timer_IF_Init(PRCM_TIMERA3, TIMERA3_BASE, TIMER_CFG_ONE_SHOT, TIMER_A, 0);  //countdown timer
    TimerControlStall(TIMERA3_BASE, TIMER_A,true);  //enable timer stall on debug breakpoint
    Timer_IF_IntSetup(TIMERA3_BASE, TIMER_A, countdownTimerInt3);
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
static void countdownTimerInt3()
{
    if (TimerIntStatus(TIMERA3_BASE,true)==0x1){   // timeout int has happened
        TimerIntClear(TIMERA3_BASE,TIMER_TIMA_TIMEOUT);
        TA3running=false;
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

void setupSweep(){
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
        MAP_UtilsDelay(MILLISECONDS_TO_TICKS(200));//todo: DALAY A BIT FOR THE FIRST WAVE TO SETTLE?
}

void doSingleSweep(){
    while(freq>=startFreq){   //START FROM FINAL FREQ
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

         if(((maxValue1-minValue1)>1900)||(minValue1<700)||(maxValue1>3500)||(clipped==true)){   // we have big amplitude.reduce gain. (consider the perturbations from the solution potential that can overdrive the in-amp.)
             D=D-1;//decrease gain to avoid clipping.
             if(D<0){
                 D=0;
             }
             changeGain(D);
         }
         if(((maxValue1-minValue1)<200)){//for inductive loads. MAYBE NOT NEEDED. 1st order circuits with inductors dont change significanlty in the frequencies we use. need to go to MHZ frequencies.
             D=D+1;
             if(D>7){
                 D=7;
             }
             changeGain(D);
         }

         //todo:this could be false for extremely inductive loads which will increase gain in lower freqs
         if(count==0){
             noRolloffFreq=(float)GBWP/gain/10;    //calculate this from gain used in the max used frequency. I assume zero rolloff happens in BW/10.
         }
         count+=1;
         freq-=stepFreq; //next frequency

    }
    freq+=stepFreq; //store the final frequency(it's the smallest scanned)
    finalSmoothingMedian();//SMOOTHEN PHASE DIFF
    finalSmoothingAverage();//FURTHER SMOOTHEN USING ROLLING AVG FILTER
    finalSmoothingImpedance();//SMOOTHEN imp USING THE ROLLING MEDIAN FILTER
    count-=1;//store the frequency count

    UART_PRINT("0 Rolloff max frequency for this sample: %.1f\n\r",noRolloffFreq);
    UART_PRINT("Sweep finished\n\r");
}


//****************************************************************************
//
//! \brief Opening a TCP server side socket and receiving data
//!
//! This function opens a TCP socket in Listen mode and waits for an incoming
//!    TCP connection.
//! Then waits for the instructions and does sweeps until a stop command or disconnection.
//!
//!
//! \param[in] port number on which the server will be listening on
//!
//! \return     0 on success, -1 on error.
//!
//! \note   This function will wait for an incoming connection till
//!                     one is established
//
//****************************************************************************
int cellServer(unsigned short usPort)
{
    SlSockAddrIn_t  sAddr;
    SlSockAddrIn_t  sLocalAddr;
    int             transmitCount=0;
    int             iAddrSize;
    int             iSockID;
    int             iStatus;
    int             iNewSockID;
    long            lLoopCount = 0;
    long            lNonBlocking = 1;
    long            lBlocking = 0;
    int             iTestBufLen;
    long lRetVal = 0;
    int i3,i4,i5;//clock interval counter

    memset(g_cBsdBuf,0,sizeof(g_cBsdBuf));//clear whole buffer.
    //filling the TCP server socket address
    sLocalAddr.sin_family = SL_AF_INET;
    sLocalAddr.sin_port = sl_Htons((unsigned short)usPort);
    sLocalAddr.sin_addr.s_addr = 0;

    // creating a TCP socket
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if( iSockID < 0 )
    {
        // error
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }

    iAddrSize = sizeof(SlSockAddrIn_t);
    //OPTION, needs to be blocking
    iStatus = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
                            &lBlocking, sizeof(lBlocking));
    if( iStatus < 0 )
    {
        // error
        sl_Close(iSockID);
        ASSERT_ON_ERROR(SOCKET_OPT_ERROR);
    }
    // binding the TCP socket to the TCP server address
    iStatus = sl_Bind(iSockID, (SlSockAddr_t *)&sLocalAddr, iAddrSize);
    if( iStatus < 0 )
    {
        // error
        sl_Close(iSockID);
        ASSERT_ON_ERROR(BIND_ERROR);
    }

    // putting the socket for listening to the incoming TCP connection
    iStatus = sl_Listen(iSockID, 0);
    if( iStatus < 0 )
    {
        sl_Close(iSockID);
        ASSERT_ON_ERROR(LISTEN_ERROR);
    }

    if( iStatus < 0 )
    {
        sl_Close(iSockID);
        ASSERT_ON_ERROR(SOCKET_OPT_ERROR);
    }

    //Accepts a connection form a TCP client, if there is any
    iNewSockID = sl_Accept(iSockID, ( struct SlSockAddr_t *)&sAddr,
                            (SlSocklen_t*)&iAddrSize);

    if( iNewSockID < 0 )
    {
        // error
        sl_Close(iNewSockID);
        sl_Close(iSockID);
        ASSERT_ON_ERROR(ACCEPT_ERROR);
    }
    //first time we need to be blocking ,expecting first message.
    iStatus = sl_SetSockOpt(iNewSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
                            &lBlocking, sizeof(lBlocking));
    if( iStatus < 0 )
    {
        // error
        sl_Close(iNewSockID);
        sl_Close(iSockID);
        ASSERT_ON_ERROR(SOCKET_OPT_ERROR);
    }
    iStatus = sl_Recv(iNewSockID, g_cBsdBuf, BUF_SIZE, 0);//}
    //TODO sometimes istatus gives 0 without communication??or when disconnected because for some reason the callbacks never work aside from ip leased..
    if( iStatus <= 0 )
    {
      // error

        Report("RECV_ERROR,closed sockets\n\r");
        // close the connected socket
        iStatus = sl_Close(iNewSockID);
        ASSERT_ON_ERROR(iStatus);
        // close the listening socket
        iStatus = sl_Close(iSockID);
        ASSERT_ON_ERROR(iStatus);
        return -1;
    }
    if(!strcmp(g_cBsdBuf,"stop\n")){
        Report("STOP instruction received\n\r");
        //stop measuring
        return -1;
    }

    Report("Received %d BYTES successfully\n\r",iStatus);
    /* Parse JSON data */
    lRetVal = ParseJSONData(g_cBsdBuf);
    //leave 50s for the end..
    i5=(interval-50)/20;//DIV
    i4=(interval-50)-i5*20;//MOD.REMAINDER

    setupSweep();//set the sweep's parameters in here.
    TA3running=false;//countdown for interval.
    while(1){
            _SlNonOsMainLoopTask(); //TODO needed?

            //delay for the last 50s...(If measurement takes longer because step is small,then we will delay more than the requested interval.)
            TA3running=true;
            TimerLoadSet(TIMERA3_BASE,TIMER_A,MILLISECONDS_TO_TICKS(1000.0*50));
            TimerEnable(TIMERA3_BASE,TIMER_A);

            findInitialGain();//find the starting gain in the max frequency.

            if(transmitCount>0){//not blocking in sl_recv after first read.asynchronously expecting any "stop" message
                iStatus = sl_Recv(iNewSockID, g_cBsdBuf, BUF_SIZE, 0);//sometimes istatus gives 0 without communication??or when disconnected because for some reason the callbacks never work aside from ip leased..
                if(iStatus==0){//error,probably disconnected
                    Report("RECV_ERROR,closed sockets\n\r");
                    break;
                }
                else if(iStatus>0){
                    if(!strcmp(g_cBsdBuf,"stop\n")){
                        Report("STOP instruction received\n\r");
                        //stop measuring
                        break;
                    }
                }

            }

            //(the sweep takes max 5s.the sending of data takes max. 20s.)

            //here we are doing the measurements.measurements take max~20s
            doSingleSweep();


            if(transmitCount>0){//not blocking in sl_recv after first read.asynchronously expecting any "stop" message
                iStatus = sl_Recv(iNewSockID, g_cBsdBuf, BUF_SIZE, 0);//sometimes istatus gives 0 without communication??or when disconnected because for some reason the callbacks never work aside from ip leased..
                if(iStatus==0){//error,probably disconnected
                    Report("RECV_ERROR,closed sockets\n\r");
                    break;
                }
                else if(iStatus>0){
                    if(!strcmp(g_cBsdBuf,"stop\n")){
                        Report("STOP instruction received\n\r");
                        //stop measuring
                        break;
                    }
                }
                // setting socket option to make the socket as blocking.we need a blocking sl_send in order to acknowledge when there is disconnection or error and restart program.
                iStatus = sl_SetSockOpt(iNewSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
                                        &lBlocking, sizeof(lBlocking));
                if( iStatus < 0 ){
                    // error
                    break;
                }
            }

            firstdata=count;//the first,smallest frequency is at the final position
            // sending multiple packets to the TCP server
            while (freq <= endFreq)//send all these packets
            {
                memset(g_cBsdBuf,0,sizeof(g_cBsdBuf));//clear whole buffer.
                sprintf(g_cBsdBuf,"{freq:%d,mag:%.2f,phase:%.2f}\n",freq,impedance[count],pk_pk_phaseDiff[count]);
                // sending packet
                iStatus = sl_Send(iNewSockID,g_cBsdBuf,BUF_SIZE,0);
                MAP_UtilsDelay(300000);//TODO some phones like the motorola ,dont pick up the packets fast enough and this delay is needed.found 100000 cycles to be insufficient 140000 seems ok.
                _SlNonOsMainLoopTask(); //needed?
                if( iStatus <= 0 ){// error
                    iStatus = sl_Close(iNewSockID);
                    ASSERT_ON_ERROR(iStatus);
                    // close the listening socket
                    iStatus = sl_Close(iSockID);
                    ASSERT_ON_ERROR(iStatus);
                    Report("send_ERROR\n\r");
                    return -1;
                }
                Report("Sent %s\n\r",g_cBsdBuf);
                count--;
                freq=freq+stepFreq;
            }
            Report("Sent all packets successfully\n\r");//todo:show count
            //break;
            // setting socket option to make the socket as non blocking in order to receive any "stop" command.
            iStatus = sl_SetSockOpt(iNewSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
                                    &lNonBlocking, sizeof(lNonBlocking));
            if( iNewSockID < 0 ){
                // error
                break;
            }

            while(TA3running==true);
            Report("Finished sweep & sent data.\n\r");
            //wait for (i5)*20s...
            for (i3=1;i3<=i5;i3++){
                TA3running=true;
                TimerLoadSet(TIMERA3_BASE,TIMER_A,MILLISECONDS_TO_TICKS(1000.0*20));
                TimerEnable(TIMERA3_BASE,TIMER_A);
                while(TA3running==true);

                //check every 20s for stop/disconnect:
                //not blocking in sl_recv after first read.asynchronously expecting "stop" message
                iStatus = sl_Recv(iNewSockID, g_cBsdBuf, BUF_SIZE, 0);//sometimes istatus gives 0 without communication??or when disconnected because for some reason the callbacks never work aside from ip leased..
                if(iStatus==0){//error,probably disconnected
                    Report("RECV_ERROR,closed sockets\n\r");
                    iStatus = sl_Close(iNewSockID);
                    ASSERT_ON_ERROR(iStatus);
                    // close the listening socket
                    iStatus = sl_Close(iSockID);
                    ASSERT_ON_ERROR(iStatus);
                    return -1;
                }
                else if(iStatus>0){
                    if(!strcmp(g_cBsdBuf,"stop\n")){
                        Report("STOP instruction received\n\r");
                        //stop measuring
                        iStatus = sl_Close(iNewSockID);
                        ASSERT_ON_ERROR(iStatus);
                        // close the listening socket
                        iStatus = sl_Close(iSockID);
                        ASSERT_ON_ERROR(iStatus);
                        return -1;
                    }
                }
            }
            //delay for remainder:
            if(i4>0){
                TA3running=true;
                TimerLoadSet(TIMERA3_BASE,TIMER_A,MILLISECONDS_TO_TICKS(1000.0*i4));
                TimerEnable(TIMERA3_BASE,TIMER_A);
                while(TA3running==true);
            }
            while(TA3running==true);

            memset(g_cBsdBuf,0,sizeof(g_cBsdBuf));//clear whole buffer.
            transmitCount++;

    }

    // close the connected socket after receiving from connected TCP client?
    iStatus = sl_Close(iNewSockID);
    ASSERT_ON_ERROR(iStatus);
    // close the listening socket
    iStatus = sl_Close(iSockID);
    ASSERT_ON_ERROR(iStatus);
    return -1;

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
    long lRetVal = -1;
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

    InitializeAppVariables();

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its default state at start of applicaton
    //
    // Note that all profiles and persistent settings that were done on the
    // device will be lost
    //
    lRetVal = ConfigureSimpleLinkToDefaultState();

    if(lRetVal < 0)
    {
      if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
         UART_PRINT("Failed to configure the device in its default state \n\r");

      LOOP_FOREVER();
    }

    UART_PRINT("Device is configured in default state \n\r");
    //
    // Asumption is that the device is configured in station mode already
    // and it is in its default state
    //
    lRetVal = sl_Start(0, 0, 0);
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    UART_PRINT("Device started as STATION \n\r");



    //
    // Configure the networking mode and ssid name(for AP mode)
    //
    if(lRetVal != ROLE_AP)
    {
        if(ConfigureMode(lRetVal) != ROLE_AP)
        {
            UART_PRINT("Unable to set AP mode, exiting Application...\n\r");
            sl_Stop(SL_STOP_TIMEOUT);
            LOOP_FOREVER();
        }
    }


    while(!IS_IP_ACQUIRED(g_ulStatus))
    {
#ifndef SL_PLATFORM_MULTI_THREADED
      _SlNonOsMainLoopTask();
#endif
    }


    //
    // Reset the spi peripheral
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
    configTA3();//countdown timer 3
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


    while(1){//if we disconnect or end session, the code below cellserver executes.TODO: I THINK ITS NO USE,USE THE OLD INSTRUCTIONS AND DISCONNECT FROM ANDROID INSTEAD.
        _SlNonOsMainLoopTask();
        lRetVal = cellServer(PORT_NUM);
        if(lRetVal < 0)
        {
            UART_PRINT("TCP Server failed\n\r");
        }
        sl_Stop(0xff);
        lRetVal = ConfigureSimpleLinkToDefaultState();
        if(lRetVal < 0)
        {
          if (DEVICE_NOT_IN_STATION_MODE == lRetVal)
             UART_PRINT("Failed to configure the device in its default state \n\r");

          LOOP_FOREVER();
        }

        UART_PRINT("Device is configured in default state \n\r");
        MAP_UtilsDelay(100000000);//TODO:delay before re-establishing connection?

        //
        // Asumption is that the device is configured in station mode already
        // and it is in its default state
        //
        lRetVal = sl_Start(0, 0, 0);
        if (lRetVal < 0)
        {
            UART_PRINT("Failed to start the device \n\r");
            LOOP_FOREVER();
        }

        UART_PRINT("Device started as STATION \n\r");



        //
        // Configure the networking mode and ssid name(for AP mode)
        //
        if(lRetVal != ROLE_AP)
        {
            if(ConfigureMode(lRetVal) != ROLE_AP)
            {
                UART_PRINT("Unable to set AP mode, exiting Application...\n\r");
                sl_Stop(SL_STOP_TIMEOUT);
                LOOP_FOREVER();
            }
        }

        while(!IS_IP_ACQUIRED(g_ulStatus))
        {
    #ifndef SL_PLATFORM_MULTI_THREADED
          _SlNonOsMainLoopTask();
    #endif
        }

    }

    UART_PRINT("Exiting Application ...\n\r");

    //
    // power of the Network processor
    //
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    while (1)
    {
        _SlNonOsMainLoopTask();
    }

}
