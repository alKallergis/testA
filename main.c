//Copyright (c) 2018 Alex Kallergis

/*This program receives 2 input servo channels(coming from a RC receiver) and outputs a servo channel controlled by a PD loop.
 * The timers are 24-bit PWM. TimerA2A is the input signal from the receiver. The timerA2B input determines the gains Kp & Kd of the control system.
 * When this channel's pulse is bigger than ~1.06ms,the gains are proportional to the pulse width. When it is smaller,the receiver signal(A2A) is
 * passed straight to the output.
 * An ITG-1010 gyroscope sensor is interfaced via i2c and set to work @ 1KHZ with a 184HZ LPF. Max rate is +-1000deg/s
 * When the control works,the output channel is the output of a closed loop system where the error is the TimerA2A input signal
 * converted to degrees/second,minus the gyroscope x axis detected rate which is also in degrees/sec.
 * With a considered pulse width of 1-2ms and a period 20ms,the conversion is arbitrarily set to -500deg/s for 1ms and 500 deg/s for 2ms.
 * Note that any inputs and outputs outside the 1-2ms bounds are brought back to bounds.
 *
 * An ADC is also used with interrupts to calculate an onboard voltage.
 * This cc3200 also an AP open where one can connect and receive the UDP packets with the telemetry.The packets
 * are sent every transmissionPeriodPrescale times a positive edge is detected from the input channel.

Signals:
    P60-ADC INPUT. notice it mut be lower than 1,4v
    P03-I2C SCL to ITG-1010
    P04-I2C SDA to ITG-1010
    P15-TIMERA2A
    P05-TIMERA2B
    P01-Output signal
    P02-Output signal(Conjucate of P01)



*/

// Standard includes
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// simplelink includes
#include "simplelink.h"
#include "wlan.h"

// driverlib includes
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "wdt.h"
#include "wdt_if.h"
#include "prcm.h"
#include "utils.h"
#include "uart.h"
#include "hw_adc.h"
#include "adc.h"
#include <inttypes.h>
#include "timer.h"
#include "timer_if.h"
// common interface includes
#include "i2c_if.h"
#include "common.h"
#include "pin_mux_config.h"


#ifndef NOTERM
#include "uart_if.h"
#endif

#include "pin_mux_config.h"

#define PORT_NUM           5001
#define BUF_SIZE           128

#define UART_PRINT              Report
#define FOREVER                 1
#define CONSOLE                 UARTA0_BASE
#define FAILURE                 -1
#define SUCCESS                 0
#define WD_PERIOD_MS                 200    //wdt int every 200ms
#define RETERR_IF_TRUE(condition) {if(condition) return FAILURE;}
#define RET_IF_ERR(Func)          {int iRetVal = (Func); \
                                   if (SUCCESS != iRetVal) \
                                     return  iRetVal;}
#define Period_ticks 0x186a00   //TimerA2 ticks for a 20ms period
#define TICKS_FOR_3MS   240000  //need this to define a valid pulse
#define TICKS_FOR_1point5MS   120000
#define TICKS_FOR_CONTROL_ENABLE   93000
#define START_TICKS 85000

// Application specific status/error codes
typedef enum{
    // Choosing -0x7D0 to avoid overlap w/ host-driver's error codes
    SOCKET_CREATE_ERROR = -0x7D0,
    BIND_ERROR = SOCKET_CREATE_ERROR - 1,
    SEND_ERROR = BIND_ERROR - 1,
    RECV_ERROR = SEND_ERROR -1,
    SOCKET_CLOSE = RECV_ERROR -1,  
    DEVICE_NOT_IN_STATION_MODE = SOCKET_CLOSE - 1,
    STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;



//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
static void BoardInit();
static void InitializeAppVariables();
static long ConfigureSimpleLinkToDefaultState();
int IpAddressParser(char *ucCMD);
void UDPTRANSMITSETUP(unsigned short usPort);
void UDPTRANSMIT(void);
float tdif(unsigned long int m);
void i2csetupAndTest(void);
tBoolean readI2Crates(void);
void calculatecoefficients(void);
void TimA2AIntHandler(void);
void TimA2BIntHandler(void);
void configTA2(void);
void configTA3(void);
void myUartReport(void);
void adcSetup(void);
void WatchdogIntHandler(void);
void configWatchdog(void);

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
volatile unsigned long  g_ulStatus,temp;//SimpleLink Status
unsigned long  g_ulStaIp;
unsigned long  g_ulPingPacketsRecv;
unsigned long  g_uiGatewayIP;
unsigned long  DestIp;        //dest IP address
unsigned long g_ulDestinationIp;
unsigned int   g_uiPortNum = PORT_NUM;
unsigned long  g_ulIpAddr,x,lLoopCount;
unsigned char g_cBsdBuf[BUF_SIZE],temp2[8];
short sTestBufLen;
int iRetVal,NODATA,dataavailable,transmissionPeriodPrescale=10;;
unsigned char ucDHCP;
long lRetVal = -1,lNonBlocking = 1,intenabled;
float ms,ms1,ms2,y,xrate[2],yrate[2],zrate[2],indeg[2],outMatchDeg;
double xerr,yerr,zerr,xcumul,ycumul,zcumul;
double lamda1,lamda2,beta1,beta2;   //coefficients
double P,D,Kp,Kd,outmatchdeg;
unsigned int i[2],m,c,n,samplesA[2],samplesB[2],invalid_pulses,valid_pulses;
unsigned int waiting,udpSendCounter,valAdc,g_ulWatchdogCycles;
long int outmatch=Period_ticks-START_TICKS;//initial match value for outmatch,close to edge
unsigned int L,H,intTest;
tBoolean above,rdy,coeffrdy,inputAlarm,transmitEnable,bRetcode;
SlSockAddrIn_t  sAddr;
tBoolean withinLoop;
tBoolean insideInt;
unsigned const char itg1010addr=0x68;
unsigned const char WHO_AM_I = 0x75;
unsigned const char PWR_MGMT_ADDR = 0x6B;
unsigned const char PWR_MGMT_CONF = 0x00;   //INTERNAL OSC
unsigned const char ITG_CONFIG_ADDR = 0x1A;
unsigned const char ITG_CONFIG_CONF = 0x01;  //Fs=1KHZ,184 HZ LPF, FIFO MODE=0,FSYNC DISABLED
unsigned const char GYRO_CONFIG_ADDR = 0x1A;
unsigned const char GYRO_CONFIG_CONF = 0x10;  //fs_sel=1000deg/s, fchoice_b=0
unsigned const char FIFO_EN_ADDR = 0x23;
unsigned const char FIFO_EN_CONF = 0x00;  //FOR NOW
unsigned const char INT_PIN_ADDR = 0x37;
unsigned const char INT_PIN_CONF = 0x40;  //int pin=open drain to avoid short circuit because on board it is tied to gnd.
unsigned const char SMPRT_DIV_addr=0x19;
unsigned const char SMPRT_DIV_conf=0x00;   //DIVIDER=0
unsigned const char INT_ENABLE_ADDR = 0x38;
unsigned const char INT_ENABLE_CONF = 0X01; //DATA_RDY_EN
unsigned const char USER_CTRL_ADDR = 0x6A;
unsigned const char USER_CTRL_CONF = 0X00; //FOR NOW
unsigned const char INT_STATUS_ADDR = 0x3A;
float Sensitivity_Scale_Factor=32.8; // for fs_sel=1000
unsigned const char gyro_x_h=0X43;  //reg address
_i8 i2cbuf[10],drdy; //for some reason char or unsigned char wont work
unsigned char settingsbuf[2];
int k,iStatus,iSockID,iAddrSize;

#if defined(ccs) || defined(gcc)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
// SimpleLink Asynchronous Event Handlers -- Start
//*****************************************************************************


//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]  pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pSlWlanEvent)
{
    switch(pSlWlanEvent->Event)
    {
        case SL_WLAN_CONNECT_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            UART_PRINT("Wlan connection event \n\r");
            //
            // Information about the connected AP (like name, MAC etc) will be
            // available in 'slWlanConnectAsyncResponse_t'-Applications
            // can use it if required
            //
            //  slWlanConnectAsyncResponse_t *pEventData = NULL;
            // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
            //
            //
        }
        break;

        case SL_WLAN_DISCONNECT_EVENT:
        {
            slWlanConnectAsyncResponse_t*  pEventData = NULL;

            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

            pEventData = &pSlWlanEvent->EventData.STAandP2PModeDisconnected;

            // If the user has initiated 'Disconnect' request,
            //'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION
            if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
            {
                UART_PRINT("Device disconnected from the AP on application's "
                            "request \n\r");
            }
            else
            {
                UART_PRINT("Device disconnected from the AP on an ERROR..!! \n\r");
            }

        }
        break;

        case SL_WLAN_STA_CONNECTED_EVENT:
        {
            // when device is in AP mode and any client connects to device cc3xxx
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            //
            // Information about the connected client (like SSID, MAC etc) will be
            // available in 'slPeerInfoAsyncResponse_t' - Applications
            // can use it if required
            //
            // slPeerInfoAsyncResponse_t *pEventData = NULL;
            // pEventData = &pSlWlanEvent->EventData.APModeStaConnected;
            //

        }
        break;

        case SL_WLAN_STA_DISCONNECTED_EVENT:
        {
            // when client disconnects from device (AP)
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            //
            // Information about the connected client (like SSID, MAC etc) will
            // be available in 'slPeerInfoAsyncResponse_t' - Applications
            // can use it if required
            //
            // slPeerInfoAsyncResponse_t *pEventData = NULL;
            // pEventData = &pSlWlanEvent->EventData.APModestaDisconnected;
            //
        }
        break;

        default:
        {
            UART_PRINT("[WLAN EVENT] Unexpected event \n\r");
        }
        break;
    }
}



//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!           leased, IP released etc.
//!
//! \param[in]  pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    switch(pNetAppEvent->Event)
    {
    case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
            {
                SlIpV4AcquiredAsync_t *pEventData = NULL;

                SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);

                //Ip Acquired Event Data
                pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

                g_ulIpAddr = pEventData->ip;

                //Gateway IP address
                g_uiGatewayIP = pEventData->gateway;

                UART_PRINT("[NETAPP EVENT] IP Acquired: IP=%d.%d.%d.%d , "
                            "Gateway=%d.%d.%d.%d\n\r",

                            SL_IPV4_BYTE(g_ulIpAddr,3),
                            SL_IPV4_BYTE(g_ulIpAddr,2),
                            SL_IPV4_BYTE(g_ulIpAddr,1),
                            SL_IPV4_BYTE(g_ulIpAddr,0),
                            SL_IPV4_BYTE(g_uiGatewayIP,3),
                            SL_IPV4_BYTE(g_uiGatewayIP,2),
                            SL_IPV4_BYTE(g_uiGatewayIP,1),
                            SL_IPV4_BYTE(g_uiGatewayIP,0));
            }
            break;
        case SL_NETAPP_IPV6_IPACQUIRED_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_AQUIRED);
            UART_PRINT("IPV6 ACQUIRED\n\r");
        }
        break;

        case SL_NETAPP_IP_LEASED_EVENT:
        {
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            g_ulStaIp = (pNetAppEvent)->EventData.ipLeased.ip_address;

            UART_PRINT("[NETAPP EVENT] IP Leased to Client: IP=%d.%d.%d.%d , \n\r",
                        SL_IPV4_BYTE(g_ulStaIp,3), SL_IPV4_BYTE(g_ulStaIp,2),
                        SL_IPV4_BYTE(g_ulStaIp,1), SL_IPV4_BYTE(g_ulStaIp,0));
        }
        break;

        case SL_NETAPP_IP_RELEASED_EVENT:
        {
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            UART_PRINT("[NETAPP EVENT] IP Released for Client: IP=%d.%d.%d.%d , \n\r",
                        SL_IPV4_BYTE(g_ulStaIp,3), SL_IPV4_BYTE(g_ulStaIp,2),
                        SL_IPV4_BYTE(g_ulStaIp,1), SL_IPV4_BYTE(g_ulStaIp,0));

        }
        break;

        default:
        {
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Event);
        }
        break;
    }
}



//*****************************************************************************
//
//! \brief This function handles HTTP server events
//!
//! \param[in]  pServerEvent - Contains the relevant event information
//! \param[in]    pServerResponse - Should be filled by the user with the
//!                                      relevant response information
//!
//! \return None
//!
//****************************************************************************
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
                                  SlHttpServerResponse_t *pHttpResponse)
{
    // Unused in this application
}

//*****************************************************************************
//
//! \brief This function handles General Events
//!
//! \param[in]     pDevEvent - Pointer to General Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if(!pDevEvent)
    {
        return;
    }

    //
    // Most of the general errors are not FATAL are are to be handled
    // appropriately by the application
    //
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->EventData.deviceEvent.status,
               pDevEvent->EventData.deviceEvent.sender);
}


//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]      pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    //
    // This application doesn't work w/ socket - Events are not expected
    //
    switch( pSock->Event )
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch( pSock->socketAsyncEvent.SockTxFailData.status)
            {
                case SL_ECLOSE:
                    UART_PRINT("[SOCK ERROR] - close socket (%d) operation "
                                "failed to transmit all queued packets\n\n",
                                    pSock->socketAsyncEvent.SockTxFailData.sd);
                    break;
                default:
                    UART_PRINT("[SOCK ERROR] - TX FAILED  :  socket %d , reason "
                                "(%d) \n\n",
                                pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
                  break;
            }
            break;

        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n",pSock->Event);
          break;
    }

}

//*****************************************************************************
//
//! \brief This function handles ping report events
//!
//! \param[in]     pPingReport - Ping report statistics
//!
//! \return None
//
//****************************************************************************
void SimpleLinkPingReport(SlPingReport_t *pPingReport)
{
    SET_STATUS_BIT(g_ulStatus, STATUS_BIT_PING_DONE);
    g_ulPingPacketsRecv = pPingReport->PacketsReceived;
}

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
    g_uiGatewayIP = 0;
    g_ulDestinationIp = 0;
    g_uiPortNum = PORT_NUM;
}

//*****************************************************************************
//! \brief This function puts the device in its default state. It:
//!           - Set the mode to STATION
//!           - Configures connection policy to Auto and AutoSmartConfig
//!           - Deletes all the stored profiles
//!           - Enables DHCP
//!           - Disables Scan policy
//!           - Sets Tx power to maximum
//!           - Sets power policy to normal
//!           - Unregister mDNS services
//!           - Remove all filters
//!
//! \param   none
//! \return  On success, zero is returned. On error, negative is returned
//*****************************************************************************
static long ConfigureSimpleLinkToDefaultState()
{
    SlVersionFull   ver = {0};
    _WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

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
            return DEVICE_NOT_IN_STATION_MODE;
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

    // Set connection policy to Auto + SmartConfig 
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
    char    pcSsidName[33]="cctestAP";
    long   lRetVal = -1;
    _u8    channel=8;

    lRetVal = sl_WlanSetMode(ROLE_AP);
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SSID, strlen(pcSsidName),
                            (unsigned char*)pcSsidName);
    ASSERT_ON_ERROR(lRetVal);
    //UART_PRINT("Device is configured in AP mode\n\r");
    lRetVal = sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_CHANNEL, strlen(channel),     //set channel to 8
                            (unsigned char*)&channel);
    ASSERT_ON_ERROR(lRetVal);

    /* Restart Network processor */
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);

    // reset status bits
    CLR_STATUS_BIT_ALL(g_ulStatus);

    return sl_Start(NULL,NULL,NULL);
}


void UDPTRANSMITSETUP(unsigned short usPort)
{
    sTestBufLen  = BUF_SIZE;
    DestIp = g_uiGatewayIP & 0xffffffff;        // final program target is broadcast .255
    //filling the UDP server socket address
    sAddr.sin_family = SL_AF_INET;
    sAddr.sin_port = sl_Htons((unsigned short)usPort);
    sAddr.sin_addr.s_addr = sl_Htonl((unsigned int)DestIp);
    UART_PRINT("Default settings: SSID Name: %s, PORT = %d, "
                    "Destination IP: %d.%d.%d.%d\n\r",
                    SSID_NAME, g_uiPortNum,
                    SL_IPV4_BYTE(DestIp,3),
                    SL_IPV4_BYTE(DestIp,2),
                    SL_IPV4_BYTE(DestIp,1),
                    SL_IPV4_BYTE(DestIp,0));

    iAddrSize = sizeof(SlSockAddrIn_t);

    // creating a UDP socket
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, 0);
    if( iSockID < 0 )
    {
        // error
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }

    // setting socket option to make the socket as non blocking
    iStatus = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
                            &lNonBlocking, sizeof(lNonBlocking));
    if (iStatus!=0){
        UART_PRINT("Couldn't set non-blocking!!\n\r");
    }

}

void UDPTRANSMIT(void){

    sprintf(g_cBsdBuf,"%1.2f V",(valAdc*1.466/4095.0));

        // sending packet
    iStatus = sl_SendTo(iSockID, g_cBsdBuf, sTestBufLen, 0,
                            (SlSockAddr_t *)&sAddr, iAddrSize);
    if (iStatus<0){
        if( iStatus == SL_EAGAIN ){
            UART_PRINT("eagain\n\r");   //TRY AGAIN..
            }
        else{
            UART_PRINT("error code:   %d\n\r",
                       iStatus);}
    }
    else{
        UART_PRINT("Sent message:   %s,\n\r",g_cBsdBuf);
    }
    #ifndef SL_PLATFORM_MULTI_THREADED
                  _SlNonOsMainLoopTask();
    #endif

    memset(g_cBsdBuf,0,sizeof(g_cBsdBuf));  //clear whole buffer.
    lLoopCount++;
    //UART_PRINT("Sent %u packets successfully\n\r",g_ulPacketCount);

}

void i2csetupAndTest(void){
    #ifndef SL_PLATFORM_MULTI_THREADED
                  _SlNonOsMainLoopTask();
    #endif

    while(I2C_IF_ReadFrom(itg1010addr,&WHO_AM_I,sizeof(WHO_AM_I),i2cbuf,1)<0);   //read whoami to confirm 0x68. loop if no connection.

    sprintf(settingsbuf,"%c%c",PWR_MGMT_ADDR,PWR_MGMT_CONF);
    I2C_IF_Write(itg1010addr,settingsbuf,2,1);      //write the GYRO configuration

    sprintf(settingsbuf,"%c%c",GYRO_CONFIG_ADDR,GYRO_CONFIG_CONF);
    I2C_IF_Write(itg1010addr,settingsbuf,2,1);      //write the GYRO configuration

    sprintf(settingsbuf,"%c%c",ITG_CONFIG_ADDR,ITG_CONFIG_CONF);
    I2C_IF_Write(itg1010addr,settingsbuf,2,1);      //write the conf configuration

    sprintf(settingsbuf,"%c%c",SMPRT_DIV_addr,SMPRT_DIV_conf);
    I2C_IF_Write(itg1010addr,settingsbuf,2,1);      //write the smprt configuration

    sprintf(settingsbuf,"%c%c",FIFO_EN_ADDR,FIFO_EN_CONF);
    I2C_IF_Write(itg1010addr,settingsbuf,2,1);

    sprintf(settingsbuf,"%c%c",INT_PIN_ADDR,INT_PIN_CONF);
    I2C_IF_Write(itg1010addr,settingsbuf,2,1);

    sprintf(settingsbuf,"%c%c",USER_CTRL_ADDR,USER_CTRL_CONF);
    I2C_IF_Write(itg1010addr,settingsbuf,2,1);

    sprintf(settingsbuf,"%c%c",INT_ENABLE_ADDR,INT_ENABLE_CONF);
    I2C_IF_Write(itg1010addr,settingsbuf,2,1);

}

tBoolean readI2Crates(void){
    #ifndef SL_PLATFORM_MULTI_THREADED
                  _SlNonOsMainLoopTask();
    #endif
    I2C_IF_ReadFrom(itg1010addr,&INT_STATUS_ADDR,
                      sizeof(INT_STATUS_ADDR),&drdy,1);   // Find whether raw int status is asserted,it is cleared after it is read here
    if(!(drdy&1)){        //no data ready
      NODATA+=1;
      return false; //exit without new values
      }
    else{
      dataavailable+=1;
      xrate[1]=xrate[0];    //shift values in fifo
      yrate[1]=yrate[0];
      zrate[1]=zrate[0];
      I2C_IF_ReadFrom(itg1010addr,&gyro_x_h,1,i2cbuf,6);      //READ GYRO REGISTERS
      xrate[0]=((i2cbuf[0]<<8)|i2cbuf[1])/Sensitivity_Scale_Factor-xerr;    //new values
      yrate[0]=(i2cbuf[2]<<8|i2cbuf[3])/Sensitivity_Scale_Factor-yerr;
      zrate[0]=(i2cbuf[4]<<8|i2cbuf[5])/Sensitivity_Scale_Factor-zerr;
      if (k<1000){        //these 11 lines are for calculating an average drift initially and then use it for compensation...
          k++;
          xcumul+=xrate[0];      //cumulative errors..
          ycumul+=yrate[0];
          zcumul+=zrate[0];
          if (k==1000){       //find average errors from the gyroscope and center value from receiver channel
              xerr=xcumul/1000.0;
              yerr=ycumul/1000.0;
              zerr=zcumul/1000.0;
              }
      }
      return true;  //new values have been read
    }
}

void calculatecoefficients(void){
    #ifndef SL_PLATFORM_MULTI_THREADED
                  _SlNonOsMainLoopTask();
    #endif
    while(!rdy){            //wait for 1st valid input pulse to be detected.
        waiting++;
        MAP_UtilsDelay(8000);  //100 us
    }
    if ((i[0]>90000)&&(i[0]<150000)){ //ensure stick has been acceptably centered or dont proceed. Also this avoids lamda being too small and bad division occurring.
        //considering stick is centered so we proceed with calculating the coefficients as per notebook:
        m=i[0];
        lamda1=(m-80000.0)/40000;
        lamda2=(160000.0-m)/40000;
        beta1=80000-80000*lamda1;
        beta2=160000-160000*lamda2;
        coeffrdy=true;  //ready to make calculations of errors. otherwise divisions by 0 would occur
    }
}

void TimA2AIntHandler(void){
//    if (GPIOPinRead(GPIOA0_BASE,GPIO_PIN_0)&0x1){ //CCP INPUT       //!! WOULD FIGURE OUT POS EDGE,BUT DOESN'T WORK
    while(insideInt==true); //trap. TODO: remove later
    insideInt=true; //keep track of when inside interrupts for testing purposes
    unsigned int temp; //temp value
    if (TimerIntStatus(TIMERA2_BASE,true)==TIMER_CAPA_EVENT){   //  has happened
        TimerIntClear(TIMERA2_BASE,TIMER_CAPA_EVENT);
        samplesA[1]=samplesA[0];
        samplesA[0]=TimerValueGet(TIMERA2_BASE, TIMER_A);
        temp=samplesA[1]-samplesA[0];
        if (temp<TICKS_FOR_3MS){   //be careful with wrap around arithmetic. here it won't work because the timers are 24,not 32-bit
            i[1]=i[0];  //shift fifo
            if(temp<80000){ //smaller than 1ms,need to correct
                    i[0]=80000;}//1ms
            else if (temp>160000){   //bigger than 2ms,need to correct
                i[0]=160000;    //2ms
            }
            else{
                i[0]=temp;}    //update input ticks array with new captured pulse
            if(coeffrdy&&above){   //if coefficients are ready &slider is above threshold,
                indeg[1]=indeg[0];
                //use the adjusted formula. this formula gives indeg=0 deg/s for y=m
                if(i[0]<=m){     //first part of the curve
                    indeg[0]=(i[0]-beta1)/lamda1/80-1500;}  // TODO check,does this conversion to float work??
                else{   //2nd part of the curve
                    indeg[0]=(i[0]-beta2)/lamda2/80-1500;
                }
                zrate[1]=zrate[0];  //calculate new errors..
                P=Kp*(indeg[0]-zrate[0]);    //proportional term
                D=Kd*(indeg[0]-zrate[0]-indeg[1]+zrate[1])/40.0;   //differential
                outMatchDeg=P+D;
                outmatch=(outMatchDeg+1500)*80; //using the regular formula now
                myUartReport(); //monitor
            }
            else{   //just pass signal
                outmatch=i[0];
            }
            valid_pulses++;
            ms1=i[0]*12.5/1000000;  //input in milliseconds
            rdy=true;       //first result has been received
            inputAlarm=false;// FEED A WATCHDOG HERE. IF IT IS NOT FED (LOSS OF CONNECTION TO INPUT),SET INDEG[0]=0. THEN IN NEXT MAIN LOOP CYCLES, INPUTS WILL BE CONSIDERED CENTERED.
        }
        else{   //rising edge.change ta3match here as per notebook...
            if (outmatch<80000){outmatch=80000;}  //border the output..
            else if (outmatch>160000){outmatch=160000;}
            invalid_pulses++;   //this case was not a valid pos->neg pulse
            //create 2 conjugate/inverted signals:
            TimerMatchSet(TIMERA3_BASE, TIMER_A,((Period_ticks-outmatch)&0xffff));
            TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_A,((Period_ticks-outmatch)&0xff0000)>>16);
            TimerMatchSet(TIMERA3_BASE, TIMER_B,(Period_ticks-TICKS_FOR_3MS+outmatch)&0xffff);
            TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_B,((Period_ticks-TICKS_FOR_3MS+outmatch)&0xff0000)>>16);
            udpSendCounter++;   //count for knowing when to send packet
        }
        //L=HWREG(TIMERA2_BASE + TIMER_O_TAR);    //read LSBS of capture value as per datasheet. Actually captures all 32 bits.
                                                //Also, actually the TnR doesn't capture the edge ,and rolls along with TnV instead..
        //H=HWREG(TIMERA2_BASE + TIMER_O_TAPR);   //read MSBS -//-.. Doesn't work,is always read 255 here
    }
    insideInt=false;
}

void TimA2BIntHandler(void){
    while(insideInt==true); //trap. remove later
    insideInt=true; //keep track of when inside interrupts for testing purposes
    if (TimerIntStatus(TIMERA2_BASE,true)==TIMER_CAPB_EVENT){   //  has happened
        TimerIntClear(TIMERA2_BASE,TIMER_CAPB_EVENT);
        samplesB[1]=samplesB[0];
        samplesB[0]=TimerValueGet(TIMERA2_BASE, TIMER_B);
        if ((samplesB[1]-samplesB[0])<TICKS_FOR_3MS){   //be careful with wrap around arithmetic. here it won't work because the timers are 24,not 32-bit
            c=samplesB[1]-samplesB[0];
            ms2=c*12.5/1000000;  //in milliseconds
            if (c > TICKS_FOR_CONTROL_ENABLE){     //signal more than half,assert boolean
                above=true;
                Kp=10.0*(c-93000.0)/100000;
                Kd=30.0*(c-93000.0)/100000;
            }
            else{above=false;}
        }
    }
    insideInt=false;
}
void WatchdogIntHandler(void){

    //
    // Clear the watchdog interrupt.
    //
    MAP_WatchdogIntClear(WDT_BASE);

    if(inputAlarm==true){
        indeg[0]=0;
    }
    inputAlarm=true;    //assert the alarm for the input servo so the TIMA2A int can de-assert it
    transmitEnable=true;    //transmit once
    g_ulWatchdogCycles++;

}

void configTA2(void){
    Timer_IF_Init(PRCM_TIMERA2,TIMERA2_BASE,(TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_CAP_TIME|TIMER_CFG_B_CAP_TIME),
                  TIMER_BOTH,0xff); //TIMER0 A&B CAPTURE, T=209.7152 ms
    TimerControlEvent(TIMERA2_BASE,TIMER_BOTH,TIMER_EVENT_BOTH_EDGES);       //capture both edges in both timers
    TimerIntRegister(TIMERA2_BASE, TIMER_A, TimA2AIntHandler);
    TimerIntRegister(TIMERA2_BASE, TIMER_B, TimA2BIntHandler);
    TimerIntEnable(TIMERA2_BASE,TIMER_CAPA_EVENT|TIMER_CAPB_EVENT);      //INT capture event enable
    TimerControlStall(TIMERA2_BASE, TIMER_BOTH,true);
    TimerLoadSet(TIMERA2_BASE,TIMER_BOTH,0xffff);  //for T=209.7152 ms
}

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
void configWatchdog(void){
    WDT_IF_Init(WatchdogIntHandler, MILLISECONDS_TO_TICKS(WD_PERIOD_MS));
    WatchdogStallEnable(WDT_BASE);
    bRetcode = MAP_WatchdogRunning(WDT_BASE);
    if(!bRetcode)
    {
       WDT_IF_DeInit();
    }
}

void adcSetup(void){
    // clear data
    while(ADCFIFOLvlGet(ADC_BASE, ADC_CH_3)) {
        ADCFIFORead(ADC_BASE, ADC_CH_3);}
// initialize ADC on channel 3 and 0
    ADCChannelEnable(ADC_BASE, ADC_CH_3);
    ADCTimerConfig(ADC_BASE,0x1ffff);  //or 20000?
    ADCTimerEnable(ADC_BASE);
    ADCEnable(ADC_BASE);
}

void myUartReport(void){

    UART_PRINT("Indeg=%3.3f\n\r",indeg[0]);
    UART_PRINT("Zrate=%3.3f\n\r",zrate[0]);
    UART_PRINT("P=%3.3f\n\r",P);
    UART_PRINT("D=%3.3f\n\r",D);
    UART_PRINT("Out=%3.3f\n\r",outMatchDeg);
}
float tdif(unsigned long int m){
    unsigned long h=0;
    float k;
    h=TimerValueGet(TIMERA1_BASE, TIMER_A);
    if(h<m){
        k=(m-h)*12.5/1000;
    }
    else{

    }
    return k;
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
#if defined(ccs) || defined(gcc)
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

//****************************************************************************
//                            MAIN FUNCTION
//****************************************************************************
void main()
{
    long lRetVal = -1;

    //
    // Board Initialization
    //
    BoardInit();


    //
    // Configure the pinmux settings for the peripherals exercised
    //
    PinMuxConfig();

    //
    // Configuring UART
    //
    InitTerm();
    //
    // I2C Init
    //
    I2C_IF_Open(I2C_MASTER_MODE_FST);

    InitializeAppVariables();

    //
    // Following function configure the device to default state by cleaning
    // the persistent settings stored in NVMEM (viz. connection profiles &
    // policies, power policy etc)
    //
    // Applications may choose to skip this step if the developer is sure
    // that the device is in its desired state at start of applicaton
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
    lRetVal = sl_Start(NULL,NULL,NULL); //here takes ~100ms...


    if (lRetVal < 0)
    {
        UART_PRINT("Failed to start the device \n\r");
        LOOP_FOREVER();
    }

    //UART_PRINT("Device started as STATION \n\r");

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

    unsigned char len = sizeof(SlNetCfgIpV4Args_t);
    SlNetCfgIpV4Args_t ipV4 = {0};

    // get network configuration
    lRetVal = sl_NetCfgGet(SL_IPV4_AP_P2P_GO_GET_INFO,&ucDHCP,&len,
                            (unsigned char *)&ipV4);
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to get network configuration \n\r");
        LOOP_FOREVER();
    }
    UART_PRINT("\n\rDevice IP: %d.%d.%d.%d\n\r\n\r",
                      SL_IPV4_BYTE(g_ulIpAddr,3),
                      SL_IPV4_BYTE(g_ulIpAddr,2),
                      SL_IPV4_BYTE(g_ulIpAddr,1),
                      SL_IPV4_BYTE(g_ulIpAddr,0));

    adcSetup();
    configTA2();        //timer for capture
    configTA3();        //timer for PWM signals

    MAP_UtilsDelay(96000000);  //~6 s

    TimerEnable(TIMERA2_BASE,TIMER_BOTH);
    TimerEnable(TIMERA3_BASE,TIMER_BOTH);
    configWatchdog();

    i2csetupAndTest();
    UDPTRANSMITSETUP(PORT_NUM);
    calculatecoefficients();
    while (1){
        #ifndef SL_PLATFORM_MULTI_THREADED
                      _SlNonOsMainLoopTask();
        #endif
        if(above){   //if slider is above threshold,

            IntDisable(INT_TIMERA2A);   //disable ints here because we need stable values
            IntDisable(INT_TIMERA2B);
            withinLoop=true;    //when we are within this loop,set flag for testing purposes
            if(readI2Crates()){ //If the sensor is ready,read the rates from the gyro in deg/s
                indeg[1]=indeg[0];  //shift values in fifo
                P=Kp*(indeg[0]-zrate[0]);    //proportional term
                D=Kd*(indeg[0]-zrate[0]-indeg[1]+zrate[1])/40.0;   //differential
                outMatchDeg=P+D;
                outmatch=(outMatchDeg+1500)*80; //using the regular formula now
            }
            withinLoop=false;
            IntEnable(INT_TIMERA2A);    //enable ints again,new values have been set
            IntEnable(INT_TIMERA2B);
        }
        else if (coeffrdy==false){  //input is not properly centered to begin with, so we center the outputs to 120000.
            TimerMatchSet(TIMERA3_BASE, TIMER_A,((Period_ticks-120000)&0xffff));
            TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_A,((Period_ticks-120000)&0xff0000)>>16);
            TimerMatchSet(TIMERA3_BASE, TIMER_B,(Period_ticks-TICKS_FOR_3MS+120000)&0xffff);
            TimerPrescaleMatchSet(TIMERA3_BASE, TIMER_B,((Period_ticks-TICKS_FOR_3MS+120000)&0xff0000)>>16);
        }
        //else{}   just pass latest input signal.It is passed in last ta2a neg edge..

        if(transmitEnable==true){   //send data every 200ms
/*
            //32bit timer init for measuring...
            Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
            TimerControlStall(TIMERA1_BASE, TIMER_A,true);  //enable timer stall on debug breakpoint
            TimerLoadSet(TIMERA1_BASE,TIMER_A,0xffffffff);
            TimerEnable(TIMERA1_BASE,TIMER_A);
            x=TimerValueGet(TIMERA1_BASE, TIMER_A); //time reference.warning this method of time measurement can be max ~55s
*/
            //while(!ADCFIFOLvlGet(ADC_BASE, ADC_CH_3));    //no need
            valAdc = ADCFIFORead(ADC_BASE, ADC_CH_3) & 0x3FFF;
            valAdc = valAdc >> 2;
            UDPTRANSMIT();
/*            y=tdif(x);  //us
            ms=y/1000;  //ms
            TimerDisable(TIMERA1_BASE,TIMER_A);*/
            transmitEnable=false;
        }
    }
    sl_Close(iSockID);
    UART_PRINT("Exiting Application ...\n\r");

    //
    // power off the network processor
    //
    lRetVal = sl_Stop(SL_STOP_TIMEOUT);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
