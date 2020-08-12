//Copyright (c) 2018 Alex Kallergis

/*
 *

Signals:
    P60-ADC INPUT. notice it must be lower than 1,4v
    P01-PWM Output signal
    P02-PWM Output signal(Conjucate of P01)
*/

// Standard includes
#include <stdlib.h>

// simplelink includes
#include "simplelink.h"

// driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "interrupt.h"
#include "rom_map.h"
#include "prcm.h"
#include "uart.h"
#include "timer.h"
#include "adc.h"

// common interface includes
#include "network_if_MODIFIED_BY_JESU.h"
#ifndef NOTERM
#include "uart_if.h"
#endif

#include "button_if.h"
#include "adc_if.h"
#include "gpio_if.h"
#include "timer_if.h"
#include "common.h"
#include "utils.h"

// Provisioning lib include
#include "provisioning_api.h"
#include "provisioning_defs.h"

#include "sl_mqtt_client.h"

// application specific includes
#include "pinmux.h"

#define SL_PARAM_PRODUCT_VERSION_DATA   "R1.0"
#define PROVISIONING_TIMEOUT            300 //Number of seconds to wait for provisioning completion

#define APPLICATION_VERSION 	"1.4.0"

#define Period_ticks 0x186a00   //Timer ticks for a 20ms period
#define TICKS_FOR_3MS   240000  //need this to define a valid pulse
#define TICKS_FOR_1point5MS   120000
#define TICKS_FOR_CONTROL_ENABLE   93000
#define START_TICKS 85000

/*Operate Lib in MQTT 3.1 mode.*/
#define MQTT_3_1_1              false /*MQTT 3.1.1 */
#define MQTT_3_1                true /*MQTT 3.1*/

#define WILL_TOPIC              "Client"
#define WILL_MSG                "Client Stopped"
#define WILL_QOS                QOS2
#define WILL_RETAIN             false

/*Defining Broker IP address and port Number*/
#define SERVER_ADDRESS           "broker.hivemq.com"
//#define SERVER_IP_ADDRESS        "192.168.178.67" not used
#define PORT_NUMBER              1883
#define SECURED_PORT_NUMBER      8883
#define LOOPBACK_PORT            1882

#define MAX_BROKER_CONN         1

#define SERVER_MODE             MQTT_3_1
/*Specifying Receive time out for the Receive task*/
#define RCV_TIMEOUT             30

/*Background receive task priority*/
#define TASK_PRIORITY           3

/* Keep Alive Timer value*/
#define KEEP_ALIVE_TIMER        30

/*Clean session flag*/
#define CLEAN_SESSION           true

/*Retain Flag. Used in publish message. */
#define RETAIN                  1

/*Defining Publish Topic*/
#define PUB_TOPIC_FOR_SW3       "/cc3200/ButtonPressEvtSw3"
#define PUB_TOPIC_FOR_SW2       "/cc3200/ButtonPressEvtSw2"
#define PUB_TOPIC_FOR_ADC       "/cc3200/ADC"

/*Defining Number of topics*/
#define TOPIC_COUNT             3

/*Defining Subscription Topic Values*/
#define TOPIC1                  "/cc3200/ToggleLEDCmdL1"
#define TOPIC2                  "/cc3200/ToggleLEDCmdL2"
#define TOPIC3                  "/cc3200/servo"

/*Defining QOS levels*/
#define QOS0                    0
#define QOS1                    1
#define QOS2                    2

/*Spawn task priority and OSI Stack Size*/
#define OSI_STACK_SIZE          4096
#define UART_PRINT              Report

#define WLAN_DEL_ALL_PROFILES   0xFF

static OsiLockObj_t lockObj1;   //the mutex object

typedef struct connection_config{
    SlMqttClientCtxCfg_t broker_config;
    void *clt_ctx;
    unsigned char *client_id;
    unsigned char *usr_name;
    unsigned char *usr_pwd;
    bool is_clean;
    unsigned int keep_alive_time;
    SlMqttClientCbs_t CallBAcks;
    int num_topics;
    char *topic[TOPIC_COUNT];
    unsigned char qos[TOPIC_COUNT];
    SlMqttWill_t will_params;
    bool is_connected;
}connect_config;

typedef enum{
     // Choosing this number to avoid overlap w/ host-driver's error codes
    DEVICE_NOT_IN_STATION_MODE = -0x7F0,
    DEVICE_NOT_IN_AP_MODE = DEVICE_NOT_IN_STATION_MODE - 1,
    DEVICE_NOT_IN_P2P_MODE = DEVICE_NOT_IN_AP_MODE - 1,

    STATUS_CODE_MAX = -0xBB8
}e_NetAppStatusCodes;

typedef enum
{   ADC_INT_READY,
    PUSH_BUTTON_SW2_PRESSED,
    PUSH_BUTTON_SW3_PRESSED,
    BROKER_DISCONNECTION
}events;

typedef struct
{
	void * hndl;
	events event;
}event_msg;

//*****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//*****************************************************************************
static void
Mqtt_Recv(void *app_hndl, const char  *topstr, long top_len, const void *payload,
          long pay_len, bool dup,unsigned char qos, bool retain);
static void sl_MqttEvt(void *app_hndl,long evt, const void *buf,
                       unsigned long len);
static void sl_MqttDisconnect(void *app_hndl);
void ADCInterruptHandler();
long ConfigureSimpleLinkToDefaultState(void);
void InitializeAppVariables(void);
void pushButtonInterruptHandler2();
void pushButtonInterruptHandler3();
void ToggleLedState(ledEnum LedNum);
void BoardInit(void);
static void DisplayBanner(char * AppName);
void MqttClient(void *pvParameters);
void ReadADCtask(void *pvParameters);
//static float tdif(unsigned long int m);
void configTA3(void);
static long configDefaultSL(void);
int startProvisioning(void);
void generalTimeoutHandler(void);
void waitmSec(_i32 timeout);
void timeoutHandler(void);
_i8 sl_extlib_ProvEventTimeoutHdl(_u8* event, _i32 timeout);
void sl_extlib_ProvWaitHdl(_i32 timeout);

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#ifdef USE_FREERTOS
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#endif

unsigned short g_usTimerInts;
/* AP Security Parameters */
SlSecParams_t SecurityParams = {0};

/*Message Queue*/
OsiMsgQ_t g_PBQueue;

/* connection configuration */
connect_config usr_connect_config[] =
{
    {
        {
            {
                SL_MQTT_NETCONN_URL,
                SERVER_ADDRESS,
                PORT_NUMBER,
                0,
                0,
                0,
                NULL
            },
            SERVER_MODE,
            true,
        },
        NULL,
        "useralekosReps",
        NULL,
        NULL,
        true,
        KEEP_ALIVE_TIMER,
        {Mqtt_Recv, sl_MqttEvt, sl_MqttDisconnect},
        TOPIC_COUNT,
        {TOPIC1, TOPIC2, TOPIC3},
        {QOS2, QOS2, QOS2},
        {WILL_TOPIC,WILL_MSG,WILL_QOS,WILL_RETAIN},
        false
    }
};

/* library configuration */
SlMqttClientLibCfg_t Mqtt_Client={
    0,
    TASK_PRIORITY,
    30,
    true,
    (long(*)(const char *, ...))UART_PRINT
};

/*Publishing topics and messages*/
const char *pub_topic_sw2 = PUB_TOPIC_FOR_SW2;
const char *pub_topic_sw3 = PUB_TOPIC_FOR_SW3;
const char *pub_topic_adc = PUB_TOPIC_FOR_ADC;
unsigned char *data_sw2={"Push button sw2 is pressed on CC32XX device"};
unsigned char *data_sw3={"Push button sw3 is pressed on CC32XX device"};


unsigned char g_adcBuf[128];
static unsigned int valAdc;
static tBoolean ready;
static tBoolean locktest;
static unsigned long x;
float ms,y;
static long int outmatch=START_TICKS;//initial match value for outmatch
void *app_hndl = (void*)usr_connect_config;
extern volatile unsigned long g_ulStatus;//TODO: MAYBE NOT NEEDED TO DECLARE HERE?

_u8 volatile g_TimerATimedOut;
_u8 volatile g_TimerBTimedOut;


//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

void configTA3(void){
    TimerConfigure(TIMERA3_BASE,(TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM | TIMER_CFG_B_PWM));
    TimerPrescaleSet(TIMERA3_BASE,TIMER_A,0x18);    //209.7152 ms max T,here 20ms
    TimerLoadSet(TIMERA3_BASE,TIMER_A,0x6a00); //20ms
    TimerMatchSet(TIMERA3_BASE, TIMER_A,((Period_ticks-outmatch)&0xffff));
    TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_A,((Period_ticks-outmatch)&0xff0000)>>16);
    TimerPrescaleSet(TIMERA3_BASE,TIMER_B,0x18);    //209.7152 ms max T,here 20ms
    TimerLoadSet(TIMERA3_BASE,TIMER_B,0x69f0); //20ms
    TimerMatchSet(TIMERA3_BASE, TIMER_B,(Period_ticks-TICKS_FOR_3MS+outmatch)&0xffff);
    TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_B,((Period_ticks-TICKS_FOR_3MS+outmatch)&0xff0000)>>16);
    //TimerControlStall(TIMERA3_BASE, TIMER_BOTH,true);

}

//****************************************************************************
//! Defines Mqtt_Pub_Message_Receive event handler.
//! Client App needs to register this event handler with sl_ExtLib_mqtt_Init 
//! API. Background receive task invokes this handler whenever MQTT Client 
//! receives a Publish Message from the broker.
//!
//!\param[out]     topstr => pointer to topic of the message
//!\param[out]     top_len => topic length
//!\param[out]     payload => pointer to payload
//!\param[out]     pay_len => payload length
//!\param[out]     retain => Tells whether its a Retained message or not
//!\param[out]     dup => Tells whether its a duplicate message or not
//!\param[out]     qos => Tells the Qos level
//!
//!\return none
//****************************************************************************
static void
Mqtt_Recv(void *app_hndl, const char  *topstr, long top_len, const void *payload,
                       long pay_len, bool dup,unsigned char qos, bool retain)
{
    bool topicIsServo=false;    //flag for when the topic is "/cc3200/servo"
    char *output_str=(char*)malloc(top_len+1);
    memset(output_str,'\0',top_len+1);
    strncpy(output_str, (char*)topstr, top_len);
    output_str[top_len]='\0';

    if(strncmp(output_str,TOPIC1, top_len) == 0)
    {
        ToggleLedState(LED1);
    }
    else if(strncmp(output_str,TOPIC2, top_len) == 0)
    {
        ToggleLedState(LED2);
    }
    else if(strncmp(output_str,TOPIC3, top_len) == 0)   //servo topic
    {
        topicIsServo=true;
    }

    UART_PRINT("\n\rPublish Message Received");
    UART_PRINT("\n\rTopic: ");
    UART_PRINT("%s",output_str);
    free(output_str);
    UART_PRINT(" [Qos: %d] ",qos);
    if(retain)
      UART_PRINT(" [Retained]");
    if(dup)
      UART_PRINT(" [Duplicate]");
    
    output_str=(char*)malloc(pay_len+1);
    memset(output_str,'\0',pay_len+1);
    strncpy(output_str, (char*)payload, pay_len);
    output_str[pay_len]='\0';
    UART_PRINT("\n\rData is: ");
    UART_PRINT("%s",(char*)output_str);
    UART_PRINT("\n\r");
    if(topicIsServo==true){
        //output_str = strtok(NULL, ":");   //no splitting needed here,we send the value only
        outmatch = (int)strtoul(output_str,0,10);
        if (outmatch<80000){outmatch=80000;}  //border the output..
        if (outmatch>160000){outmatch=160000;}
        //create 2 conjugate/inverted signals:
        TimerMatchSet(TIMERA3_BASE, TIMER_A,((Period_ticks-outmatch)&0xffff));  //1st channel
        TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_A,((Period_ticks-outmatch)&0xff0000)>>16);
        TimerMatchSet(TIMERA3_BASE, TIMER_B,(Period_ticks-TICKS_FOR_3MS+outmatch)&0xffff);  //2nd channel
        TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_B,((Period_ticks-TICKS_FOR_3MS+outmatch)&0xff0000)>>16);

        TimerEnable(TIMERA3_BASE,TIMER_BOTH);   //start timer in case it's not started
    }
    free(output_str);
    



    return;
}

//****************************************************************************
//! Defines sl_MqttEvt event handler.
//! Client App needs to register this event handler with sl_ExtLib_mqtt_Init 
//! API. Background receive task invokes this handler whenever MQTT Client 
//! receives an ack(whenever user is in non-blocking mode) or encounters an error.
//!
//! param[out]      evt => Event that invokes the handler. Event can be of the
//!                        following types:
//!                        MQTT_ACK - Ack Received 
//!                        MQTT_ERROR - unknown error
//!                        
//!  
//! \param[out]     buf => points to buffer
//! \param[out]     len => buffer length
//!       
//! \return none
//****************************************************************************
static void
sl_MqttEvt(void *app_hndl, long evt, const void *buf,unsigned long len)
{
    int i;
    switch(evt)
    {
      case SL_MQTT_CL_EVT_PUBACK:
        UART_PRINT("PubAck:\n\r");
        UART_PRINT("%s\n\r",buf);
        break;
    
      case SL_MQTT_CL_EVT_SUBACK:
        UART_PRINT("\n\rGranted QoS Levels are:\n\r");
        
        for(i=0;i<len;i++)
        {
          UART_PRINT("QoS %d\n\r",((unsigned char*)buf)[i]);
        }
        break;
        
      case SL_MQTT_CL_EVT_UNSUBACK:
        UART_PRINT("UnSub Ack \n\r");
        UART_PRINT("%s\n\r",buf);
        break;
    
      default:
        break;
  
    }
}

//****************************************************************************
//
//! callback event in case of MQTT disconnection
//!
//! \param app_hndl is the handle for the disconnected connection
//!
//! return none
//
//****************************************************************************
static void
sl_MqttDisconnect(void *app_hndl)
{
    connect_config *local_con_conf;
    event_msg msg;
    local_con_conf = app_hndl;
    msg.hndl = app_hndl;
    msg.event = BROKER_DISCONNECTION;

    UART_PRINT("disconnect from broker %s\r\n",
           (local_con_conf->broker_config).server_info.server_addr);
    local_con_conf->is_connected = false;
    //
    // write message indicating publish message
    //
    osi_MsgQWrite(&g_PBQueue,&msg,OSI_NO_WAIT);

}


//****************************************************************************
//
//! ADC HANDLER. Whenever
//! ADC fifo ready. Write message into message queue signaling the
//!    event publish messages
//!
//! \param none
//!
//! return none
//
//****************************************************************************
void ADCInterruptHandler()
{
    event_msg msg;
    //while(!ADCFIFOLvlGet(ADC_BASE, ADC_CH_3));    //no need
    valAdc = ADCFIFORead(ADC_BASE, ADC_CH_3) & 0x3FFF;
    valAdc = valAdc >> 2;
    msg.event = ADC_INT_READY;
    msg.hndl = NULL;

/*    //32bit timer init for measuring...
    Timer_IF_Init(PRCM_TIMERA2, TIMERA2_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    TimerControlStall(TIMERA2_BASE, TIMER_A,true);  //enable timer stall on debug breakpoint
    TimerLoadSet(TIMERA2_BASE,TIMER_A,0xffffffff);
    TimerEnable(TIMERA2_BASE,TIMER_A);*/
    if(locktest){
        locktest=false; //this never supposed to happen
    }
    //x=TimerValueGet(TIMERA2_BASE, TIMER_A); //time reference.warning this method of time measurement can be max ~55s

    //
    // write message indicating publish message
    //
    osi_MsgQWrite(&g_PBQueue,&msg,OSI_NO_WAIT);
}
//****************************************************************************
//
//! Push Button Handler1(GPIOS2). Press push button2 (GPIOSW2) Whenever user
//! wants to publish a message. Write message into message queue signaling the
//!    event publish messages
//!
//! \param none
//!
//! return none
//
//****************************************************************************
void pushButtonInterruptHandler2()
{
	event_msg msg;

    msg.event = PUSH_BUTTON_SW2_PRESSED;
    msg.hndl = NULL;
    //
    // write message indicating publish message
    //
    osi_MsgQWrite(&g_PBQueue,&msg,OSI_NO_WAIT);
}

//****************************************************************************
//
//! Push Button Handler3(GPIOS3). Press push button3 (GPIOSW3) Whenever user
//! wants to publish a message. Write message into message queue signaling the
//!    event publish messages
//!
//! \param none
//!
//! return none
//
//****************************************************************************
void pushButtonInterruptHandler3()
{
	event_msg msg;
	msg.event = PUSH_BUTTON_SW3_PRESSED;
    msg.hndl = NULL;
    //
    // write message indicating exit from sending loop
    //
    osi_MsgQWrite(&g_PBQueue,&msg,OSI_NO_WAIT);

}

//****************************************************************************
//
//!    Toggles the state of GPIOs(LEDs)
//!
//! \param LedNum is the enumeration for the GPIO to be toggled
//!
//!    \return none
//
//****************************************************************************
void ToggleLedState(ledEnum LedNum)
{
    unsigned char ledstate = 0;
    switch(LedNum)
    {
    case LED1:
        ledstate = GPIO_IF_LedStatus(MCU_RED_LED_GPIO);
        if(!ledstate)
        {
            GPIO_IF_LedOn(MCU_RED_LED_GPIO);
        }
        else
        {
            GPIO_IF_LedOff(MCU_RED_LED_GPIO);
        }
        break;
    case LED2:
        ledstate = GPIO_IF_LedStatus(MCU_ORANGE_LED_GPIO);
        if(!ledstate)
        {
            GPIO_IF_LedOn(MCU_ORANGE_LED_GPIO);
        }
        else
        {
            GPIO_IF_LedOff(MCU_ORANGE_LED_GPIO);
        }
        break;
    case LED3:
        ledstate = GPIO_IF_LedStatus(MCU_GREEN_LED_GPIO);
        if(!ledstate)
        {
            GPIO_IF_LedOn(MCU_GREEN_LED_GPIO);
        }
        else
        {
            GPIO_IF_LedOff(MCU_GREEN_LED_GPIO);
        }
        break;
    default:
        break;
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
void BoardInit(void)
{
    /* In case of TI-RTOS vector table is initialize by OS itself */
    #ifndef USE_TIRTOS
    //
    // Set vector table base
    //
    #if defined(ccs)
        IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    #endif
    #if defined(ewarm)
        IntVTableBaseSet((unsigned long)&__vector_table);
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
//! Application startup display on UART
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
static void
DisplayBanner(char * AppName)
{

    UART_PRINT("\n\n\n\r");
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\t\t    CC3200 %s Application       \n\r", AppName);
    UART_PRINT("\t\t *************************************************\n\r");
    UART_PRINT("\n\n\n\r");
}
  
void ReadADCtask(void *pvParameters){
    while(1){
        osi_Sleep(500);//introducing halt
        if (ready){
            ADC_IF_EnableInterrupt();
        }
    }
}

//*****************************************************************************
//
//! \brief Function for handling provisioning (adding a profile to a new AP).
//!
//! This function assumes running mostly on first time configurations, so
//! one time settings are handled here.
//!
//! \param[in]      None
//!
//! \return         None
//!
//*****************************************************************************
int startProvisioning(void)
{
    long                lRetVal = -1;
    long                FileHandle = 0;
    slExtLibProvCfg_t   cfg;
    SlFsFileInfo_t      FsFileInfo;


    UART_PRINT("Starting Provisioning..\n\r");


    // Enable RX Statistics
    sl_WlanRxStatStart();

    // Check if version token file exists in the device FS.
    // If not, than create a file and write the required token

    // Creating the param_product_version.txt file once
    if (SL_FS_ERR_FILE_NOT_EXISTS == sl_FsGetInfo(SL_FILE_PARAM_PRODUCT_VERSION, 0 , &FsFileInfo))
    {
        sl_FsOpen(SL_FILE_PARAM_PRODUCT_VERSION, FS_MODE_OPEN_CREATE(100, _FS_FILE_OPEN_FLAG_COMMIT), NULL, &FileHandle);
        sl_FsWrite(FileHandle, NULL, SL_PARAM_PRODUCT_VERSION_DATA, strlen(SL_PARAM_PRODUCT_VERSION_DATA));
        sl_FsClose(FileHandle, NULL, NULL, NULL);
    }

    // Creating the config result file once
    if (SL_FS_ERR_FILE_NOT_EXISTS == sl_FsGetInfo(SL_FILE_PARAM_CFG_RESULT, 0 , &FsFileInfo))
    {
        sl_FsOpen(SL_FILE_PARAM_CFG_RESULT, FS_MODE_OPEN_CREATE(100, _FS_FILE_OPEN_FLAG_COMMIT), NULL, &FileHandle);
        sl_FsWrite(FileHandle, NULL, GET_CFG_RESULT_TOKEN, strlen(GET_CFG_RESULT_TOKEN));
        sl_FsClose(FileHandle, NULL, NULL, NULL);
    }

    // Creating the param device name file once/
    if (SL_FS_ERR_FILE_NOT_EXISTS == sl_FsGetInfo(SL_FILE_PARAM_DEVICE_NAME, 0 , &FsFileInfo))
    {
        sl_FsOpen(SL_FILE_PARAM_DEVICE_NAME, FS_MODE_OPEN_CREATE(100, _FS_FILE_OPEN_FLAG_COMMIT), NULL, &FileHandle);
        sl_FsWrite(FileHandle, NULL, GET_DEVICE_NAME_TOKEN, strlen(GET_DEVICE_NAME_TOKEN));
        sl_FsClose(FileHandle, NULL, NULL, NULL);
    }

    // Creating the netlist name file once/
    if (SL_FS_ERR_FILE_NOT_EXISTS == sl_FsGetInfo(SL_FILE_NETLIST, 0 , &FsFileInfo))
    {
        sl_FsOpen(SL_FILE_NETLIST, FS_MODE_OPEN_CREATE(100, _FS_FILE_OPEN_FLAG_COMMIT), NULL, &FileHandle);
        sl_FsWrite(FileHandle, NULL, SL_SET_NETLIST_TOKENS, strlen(SL_SET_NETLIST_TOKENS));
        sl_FsClose(FileHandle, NULL, NULL, NULL);
    }

    // Initializes configuration
    cfg.IsBlocking         = 1;    //Unused
    cfg.AutoStartEnabled   = 0;
    cfg.Timeout10Secs      = PROVISIONING_TIMEOUT/10;
    cfg.ModeAfterFailure   = ROLE_STA;
    cfg.ModeAfterTimeout   = ROLE_STA;

    lRetVal = sl_extlib_ProvisioningStart(ROLE_STA, &cfg);
    ASSERT_ON_ERROR(lRetVal);

    // Wait for WLAN Event
    while(!IS_IP_ACQUIRED(g_ulStatus))
    {
#ifndef SL_PLATFORM_MULTI_THREADED
        _SlNonOsMainLoopTask();
#else
        osi_Sleep(1);
#endif
    }

    //
    // Turn ON the RED LED to indicate connection success
    //
    //GPIO_IF_LedOn(MCU_RED_LED_GPIO);

    //wait for few moments
    osi_Sleep(1);

    return SUCCESS;
}

//todo: modify to not remove previous profiles and for fast connect...
static long configDefaultSL()
{
    SlVersionFull   ver = {{0}};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {{0}};

    unsigned char ucVal = 1;
    unsigned char ucConfigOpt = 0;
    unsigned char ucConfigLen = 0;
    unsigned char ucPower = 0;

    long lRetVal = -1;
    long lMode = -1;

    lMode = sl_Start(0, 0, 0);
    ASSERT_ON_ERROR(lMode);

    // If the device is not in station-mode, try configuring it in station-mode
    if (ROLE_STA != lMode)
    {
        if (ROLE_AP == lMode)
        {
            // If the device is in AP mode, we need to wait for this event
            // before doing anything
            while(!IS_IP_ACQUIRED(g_ulStatus))
            {
#ifndef SL_PLATFORM_MULTI_THREADED
                _SlNonOsMainLoopTask();
#else
                osi_Sleep(1);
#endif
            }
        }

        // Switch to STA role and restart
        lRetVal = sl_WlanSetMode(ROLE_STA);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Stop(0xFF);
        ASSERT_ON_ERROR(lRetVal);

        lRetVal = sl_Start(0, 0, 0);
        ASSERT_ON_ERROR(lRetVal);

        // Check if the device is in station again
        if (ROLE_STA != lRetVal)
        {
            // We don't want to proceed if the device is not coming up in STA-mode
            ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
        }
    }

    // Get the device's version-information
    ucConfigOpt = SL_DEVICE_GENERAL_VERSION;
    ucConfigLen = sizeof(ver);
    lRetVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &ucConfigOpt,
                                &ucConfigLen, (unsigned char *)(&ver));
    ASSERT_ON_ERROR(lRetVal);

    UART_PRINT("Host Driver Version: %s\n\r",SL_DRIVER_VERSION);
    UART_PRINT("Build Version %d.%d.%d.%d.31.%d.%d.%d.%d.%d.%d.%d.%d\n\r",
    ver.NwpVersion[0],ver.NwpVersion[1],ver.NwpVersion[2],ver.NwpVersion[3],
    ver.ChipFwAndPhyVersion.FwVersion[0],ver.ChipFwAndPhyVersion.FwVersion[1],
    ver.ChipFwAndPhyVersion.FwVersion[2],ver.ChipFwAndPhyVersion.FwVersion[3],
    ver.ChipFwAndPhyVersion.PhyVersion[0],ver.ChipFwAndPhyVersion.PhyVersion[1],
    ver.ChipFwAndPhyVersion.PhyVersion[2],ver.ChipFwAndPhyVersion.PhyVersion[3]);

    // Set connection policy to Fast
    //      (Device's default connection policy)
    lRetVal = sl_WlanPolicySet(SL_POLICY_CONNECTION,
                                SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove all profiles
    lRetVal = sl_WlanProfileDel(0xFF);
    ASSERT_ON_ERROR(lRetVal);

    //
    // Device in station-mode. Disconnect previous connection if any
    // The function returns 0 if 'Disconnected done', negative number if already
    // disconnected Wait for 'disconnection' event if 0 is returned, Ignore
    // other return-codes
    //
    lRetVal = sl_WlanDisconnect();
    if(0 == lRetVal)
    {
        // Wait
        while(IS_CONNECTED(g_ulStatus))
        {
#ifndef SL_PLATFORM_MULTI_THREADED
              _SlNonOsMainLoopTask();
#else
              osi_Sleep(1);
#endif
        }
    }

    // Enable DHCP client
    lRetVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&ucVal);
    ASSERT_ON_ERROR(lRetVal);

    // Disable scan
    ucConfigOpt = SL_SCAN_POLICY(0);
    lRetVal = sl_WlanPolicySet(SL_POLICY_SCAN , ucConfigOpt, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Set Tx power level for station mode
    // Number between 0-15, as dB offset from max power - 0 will set max power
    ucPower = 0;
    lRetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID,
            WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (unsigned char *)&ucPower);
    ASSERT_ON_ERROR(lRetVal);

    // Set PM policy to normal
    lRetVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Unregister mDNS services
    lRetVal = sl_NetAppMDNSUnRegisterService(0, 0);
    ASSERT_ON_ERROR(lRetVal);

    // Remove  all 64 filters (8*8)
    memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
    lRetVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
                       sizeof(_WlanRxFilterOperationCommandBuff_t));
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
    ASSERT_ON_ERROR(lRetVal);

    InitializeAppVariables();

    return lRetVal; // Success
}

//*****************************************************************************
//
//! Task implementing MQTT client communication to other web client through
//!    a broker
//!
//! \param  none
//!
//! This function
//!    1. Initializes network driver and connects to the default AP
//!    2. Initializes the mqtt library and set up MQTT connection configurations
//!    3. set up the button events and their callbacks(for publishing)
//!    4. handles the callback signals
//!
//! \return None
//!
//*****************************************************************************


void MqttClient(void *pvParameters)
{
    long lRetVal = -1;
    int iCount = 0;
    int iNumBroker = 0;
    int iConnBroker = 0;
    event_msg RecvQue;
    unsigned char policyVal;
    
    connect_config *local_con_conf = (connect_config *)app_hndl;

        configDefaultSL();
        //Start simplelink
        lRetVal = sl_Start(0,0,0);
        if (lRetVal < 0 || ROLE_STA != lRetVal)
        {
            UART_PRINT("Failed to start the device \n\r");
            LOOP_FOREVER();
        }

        UART_PRINT("Device started as STATION \n\r");


        if(!IS_IP_ACQUIRED(g_ulStatus)){

            // Connect to our AP using SmartConfig method
            lRetVal = startProvisioning();
            if(lRetVal < 0)
            {
                ERR_PRINT(lRetVal);
            }
            else
            {
                UART_PRINT("Provisioning Succedded \n\r");
            }
        }
        //
    //
    // Register Push Button Handlers,ADC Handler
    //
    Button_IF_Init(pushButtonInterruptHandler2,pushButtonInterruptHandler3);
    ADC_IF_Init(ADCInterruptHandler);
    //
    // Initialze MQTT client lib
    //
    lRetVal = sl_ExtLib_MqttClientInit(&Mqtt_Client);
    if(lRetVal != 0)
    {
        // lib initialization failed
        UART_PRINT("MQTT Client lib initialization failed\n\r");
        LOOP_FOREVER();
    }
    
    /******************* connection to the broker ***************************/
    iNumBroker = sizeof(usr_connect_config)/sizeof(connect_config);
    if(iNumBroker > MAX_BROKER_CONN)
    {
        UART_PRINT("Num of brokers are more then max num of brokers\n\r");
        LOOP_FOREVER();
    }

connect_to_broker:
    while(iCount < iNumBroker)
    {
        //create client context
        local_con_conf[iCount].clt_ctx =
        sl_ExtLib_MqttClientCtxCreate(&local_con_conf[iCount].broker_config,
                                      &local_con_conf[iCount].CallBAcks,
                                      &(local_con_conf[iCount]));

        //
        // Set Client ID
        //
        sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                            SL_MQTT_PARAM_CLIENT_ID,
                            local_con_conf[iCount].client_id,
                            strlen((char*)(local_con_conf[iCount].client_id)));

        //
        // Set will Params
        //
        if(local_con_conf[iCount].will_params.will_topic != NULL)
        {
            sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                                    SL_MQTT_PARAM_WILL_PARAM,
                                    &(local_con_conf[iCount].will_params),
                                    sizeof(SlMqttWill_t));
        }

        //
        // setting username and password
        //
        if(local_con_conf[iCount].usr_name != NULL)
        {
            sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                                SL_MQTT_PARAM_USER_NAME,
                                local_con_conf[iCount].usr_name,
                                strlen((char*)local_con_conf[iCount].usr_name));

            if(local_con_conf[iCount].usr_pwd != NULL)
            {
                sl_ExtLib_MqttClientSet((void*)local_con_conf[iCount].clt_ctx,
                                SL_MQTT_PARAM_PASS_WORD,
                                local_con_conf[iCount].usr_pwd,
                                strlen((char*)local_con_conf[iCount].usr_pwd));
            }
        }

        //
        // connectin to the broker
        //
        if((sl_ExtLib_MqttClientConnect((void*)local_con_conf[iCount].clt_ctx,
                            local_con_conf[iCount].is_clean,
                            local_con_conf[iCount].keep_alive_time) & 0xFF) != 0)
        {
            UART_PRINT("\n\rBroker connect fail for conn no. %d \n\r",iCount+1);
            
            //delete the context for this connection
            sl_ExtLib_MqttClientCtxDelete(local_con_conf[iCount].clt_ctx);
            
            break;
        }
        else
        {
            UART_PRINT("\n\rSuccess: conn to Broker no. %d\n\r ", iCount+1);
            local_con_conf[iCount].is_connected = true;
            iConnBroker++;
        }

        //
        // Subscribe to topics
        //

        if(sl_ExtLib_MqttClientSub((void*)local_con_conf[iCount].clt_ctx,
                                   local_con_conf[iCount].topic,
                                   local_con_conf[iCount].qos, TOPIC_COUNT) < 0)
        {
            UART_PRINT("\n\r Subscription Error for conn no. %d\n\r", iCount+1);
            UART_PRINT("Disconnecting from the broker\r\n");
            sl_ExtLib_MqttClientDisconnect(local_con_conf[iCount].clt_ctx);
            local_con_conf[iCount].is_connected = false;
            
            //delete the context for this connection
            sl_ExtLib_MqttClientCtxDelete(local_con_conf[iCount].clt_ctx);
            iConnBroker--;
            break;
        }
        else
        {
            int iSub;
            UART_PRINT("Client subscribed on following topics:\n\r");
            for(iSub = 0; iSub < local_con_conf[iCount].num_topics; iSub++)
            {
                UART_PRINT("%s\n\r", local_con_conf[iCount].topic[iSub]);
            }
        }
        iCount++;
    }

    if(iConnBroker < 1)
    {
        //
        // no succesful connection to broker
        //
        goto end;
    }

    iCount = 0;
    ready=true; //now app is ready to start communicating
    for(;;)
    {
        osi_MsgQRead( &g_PBQueue, &RecvQue, OSI_WAIT_FOREVER);

        if(PUSH_BUTTON_SW2_PRESSED == RecvQue.event)
        {
            //!NOTE:Interrupt was disabled in the ISR when it occurred,so now need to re-enable it:
            Button_IF_EnableInterrupt(SW2);
            //
            // send publish message
            //
            sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
                    pub_topic_sw2,data_sw2,strlen((char*)data_sw2),QOS2,RETAIN);
            UART_PRINT("\n\r CC3200 Publishes the following message \n\r");
            UART_PRINT("Topic: %s\n\r",pub_topic_sw2);
            UART_PRINT("Data: %s\n\r",data_sw2);
        }
        else if(PUSH_BUTTON_SW3_PRESSED == RecvQue.event)
        {
            Button_IF_EnableInterrupt(SW3);
            //
            // send publish message
            //
            sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
                    pub_topic_sw3,data_sw3,strlen((char*)data_sw3),QOS2,RETAIN);
            UART_PRINT("\n\r CC3200 Publishes the following message \n\r");
            UART_PRINT("Topic: %s\n\r",pub_topic_sw3);
            UART_PRINT("Data: %s\n\r",data_sw3);
        }
        else if(ADC_INT_READY == RecvQue.event)
        {   ready=false;    //avoid going to int for now ,because we use VALADC here
            //
            // send publish message
            //
            sprintf(g_adcBuf,"Voltage= %1.2f V",(valAdc*1.466/4095.0));

            sl_ExtLib_MqttClientSend((void*)local_con_conf[iCount].clt_ctx,
                                     pub_topic_adc,g_adcBuf,strlen((char*)g_adcBuf),QOS2,RETAIN);
            osi_LockObjLock(&lockObj1,OSI_WAIT_FOREVER);    //example use of lock,not really needed since we're only reading x here, not writing to it
            locktest=true;
            //y=tdif(x);  //us
            osi_LockObjUnlock(&lockObj1);
            //ms=y/1000;  //ms
            locktest=false;
            //TimerDisable(TIMERA2_BASE,TIMER_A);

            UART_PRINT("\n\r CC3200 Publishes the following message \n\r");
            UART_PRINT("Topic: %s\n\r",pub_topic_adc);
            UART_PRINT("Data: %s\n\r",g_adcBuf);
            UART_PRINT("Took %4.3f ms to send\n\r",ms);
            memset(g_adcBuf,0,sizeof(g_adcBuf));  //clear whole buffer.
            ready=true; //can go  to int now
        }
        else if(BROKER_DISCONNECTION == RecvQue.event)
        {
            iConnBroker--;
            /* Derive the value of the local_con_conf or clt_ctx from the message */
			sl_ExtLib_MqttClientCtxDelete(((connect_config*)(RecvQue.hndl))->clt_ctx);
            
            if(!IS_CONNECTED(g_ulStatus))
            {
                UART_PRINT("device has disconnected from AP \n\r");
                
                UART_PRINT("retry connection to the AP\n\r");
                
                while(!(IS_CONNECTED(g_ulStatus)) || !(IS_IP_ACQUIRED(g_ulStatus)))
                {
                    osi_Sleep(10);
                }
                goto connect_to_broker;
                
            }
            if(iConnBroker < 1)
            {
                //
                // device not connected to any broker
                //
                goto end;
            }
        }

    }

end:
    //
    // Deinitializating the client library
    //
    sl_ExtLib_MqttClientExit();
    UART_PRINT("\n\r Exiting the Application\n\r");
    
    LOOP_FOREVER();
}

/*static float tdif(unsigned long int m){
    unsigned long h=0;
    float k;
    h=TimerValueGet(TIMERA2_BASE, TIMER_A);
    if(h<m){
        k=(m-h)*12.5/1000;
    }
    else{

    }
    return k;
}*/

//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- End
//*****************************************************************************

void generalTimeoutHandler(void)
{
    Timer_IF_InterruptClear(TIMERA1_BASE);
    g_TimerBTimedOut++;
}

// General waiting function using timer
void waitmSec(_i32 timeout)
{
    //Initializes & Starts timer
    Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_ONE_SHOT, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA1_BASE, TIMER_A, generalTimeoutHandler);
    TimerControlStall(TIMERA1_BASE, TIMER_A,true);
    g_TimerBTimedOut = 0;

    Timer_IF_Start(TIMERA1_BASE, TIMER_A, timeout);

    while(g_TimerBTimedOut != 1)
    {
        // waiting...
#ifndef SL_PLATFORM_MULTI_THREADED
        _SlNonOsMainLoopTask();
#endif
    }

    //Stops timer
    Timer_IF_Stop(TIMERA1_BASE, TIMER_A);
    Timer_IF_DeInit(TIMERA1_BASE, TIMER_A);
}

void timeoutHandler(void)
{
    Timer_IF_InterruptClear(TIMERA0_BASE);
    g_TimerATimedOut++;
}

//*****************************************************************************
// Provisioning Callbacks
//*****************************************************************************
_i8 sl_extlib_ProvEventTimeoutHdl(_u8* event, _i32 timeout)
{
    if(timeout == SL_EXT_PROV_WAIT_FOREVER)
    {
        // Waiting forever, no timeout
        while(*event == FALSE)
        {
            // waiting...
#ifndef SL_PLATFORM_MULTI_THREADED
            _SlNonOsMainLoopTask();
#endif
        }
    }
    else
    {
        //On CC3200, a value greater than 53687 will overflow the buffer,
        //therefore divide the timeout into smaller pieces.
        int divider = 10;

        //Initializes & Starts timer
        Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
        Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, timeoutHandler);
        TimerControlStall(TIMERA0_BASE, TIMER_A,true);
        g_TimerATimedOut = 0;

        Timer_IF_Start(TIMERA0_BASE, TIMER_A, timeout/divider);

        //Check event or wait until timeout
        while(*event == FALSE && g_TimerATimedOut != divider)
        {
            // waiting...
#ifndef SL_PLATFORM_MULTI_THREADED
            _SlNonOsMainLoopTask();
#endif
        }

        //Stops timer
        Timer_IF_Stop(TIMERA0_BASE, PRCM_TIMERA0);
        Timer_IF_DeInit(TIMERA0_BASE, PRCM_TIMERA0);

        // check if timeout occured
        if(g_TimerATimedOut == divider)
        {
            return -1;
        }

    }


    return 0;
}

void sl_extlib_ProvWaitHdl(_i32 timeout)
{
    waitmSec(timeout);
}



//

//*****************************************************************************
//
//! Main 
//!
//! \param  none
//!
//! This function
//!    1. Invokes the SLHost task
//!    2. Invokes the MqttClient
//!
//! \return None
//!
//*****************************************************************************
void main()
{ 
    long lRetVal = -1;
    //
    // Initialize the board configurations
    //
    BoardInit();

    //
    // Pinmux for UART
    //
    PinMuxConfig();

    //
    // Configuring UART
    //
    InitTerm();

    //
    // Display Application Banner
    //
    DisplayBanner("MQTT_Client");

    configTA3();        //timer for PWM signals

    //
    // Start the SimpleLink Host
    //
    lRetVal = VStartSimpleLinkSpawnTask(SPAWN_TASK_PRIORITY);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    //
    // Start the MQTT Client task
    //
    osi_MsgQCreate(&g_PBQueue,"PBQueue",sizeof(event_msg),10);
    lRetVal = osi_TaskCreate(MqttClient,
                            (const signed char *)"Mqtt Client App",
                            OSI_STACK_SIZE, NULL, 2, NULL );

    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    osi_LockObjCreate(&lockObj1);    //init lock object

    lRetVal = osi_TaskCreate(ReadADCtask,
                            (const signed char *)"Read ADC task",
                            OSI_STACK_SIZE, NULL, 2, NULL );

    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }
    //
    // Start the task scheduler
    //
    osi_start();
}

