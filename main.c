







//USER_INPUT_ENABLE undefined




// Standard includes
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// simplelink includes 
#include "simplelink.h"
#include "wlan.h"
#include "network_if.h"

// driverlib includes 
#include "hw_ints.h"
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "uart.h"
#include "utils.h"

// common interface includes 
#include "udma_if.h"
#include "common.h"
#ifndef NOTERM
#include "uart_if.h"
#endif

// JSON Parser
#include "jsmn.h"

#include "pin_mux_config.h"

#define APPLICATION_NAME        "TCP Socket"
#define APPLICATION_VERSION     "1.1.1"

#define IP_ADDR             0xc0a80167 // 192.168.1.103  TODO:CHANGE IF TO ACT AS CLIENT
#define PORT_NUM            5009    //TODO:CHANGE PORT?
#define BUF_SIZE            100
#define TCP_PACKET_COUNT    4000

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

struct teststruct
{
    _u32  I;
    _u32  Gat;
    _u32  Mak;
    _u32  Ds[2];
    _u32  DcSever;
    _u32  LeeTie;
    _u8   DhState;
    _u8   Resved[3];
};


typedef struct
{
    _u32  p;
    _u32  Gatewy;
    _u32  Mas;
    _u32  Dn[2];
    _u32  DhcServer;
    _u32  Leaseime;
    _u8   DhcpStte;
    _u8   Reserve[3];
}teststruct2;


//****************************************************************************
//                      LOCAL FUNCTION PROTOTYPES
//****************************************************************************
int BsdTcpClient(unsigned short usPort);
int BsdTcpServer(unsigned short usPort);
static long WlanConnect();
static void DisplayBanner();
static void BoardInit();
static void InitializeAppVariables();


//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
extern volatile unsigned long  g_ulStatus;//SimpleLink Status
extern unsigned long  g_ulGatewayIP; //Network Gateway IP address
extern unsigned char  g_ucConnectionSSID; //Connection SSID
extern unsigned char  g_ucConnectionBSSID; //Connection BSSID
unsigned long  g_ulDestinationIp = IP_ADDR;
unsigned int   g_uiPortNum = PORT_NUM,NewSockID,temp;
static int c,i;
static int waveform,startFreq,stepFreq,endFreq,freqCount;
volatile unsigned long  g_ulPacketCount = TCP_PACKET_COUNT;
unsigned char  g_ucConnectionStatus = 0,ucConfigOpt=0;
unsigned char  g_ucSimplelinkstarted = 0;
unsigned long  g_ulIpAddr = 0;
char g_cBsdBuf[BUF_SIZE];
teststruct2 ayyyyyy;
struct teststruct tests1,tests2;

#if defined(ccs) || defined (gcc)
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
    g_ulPacketCount = TCP_PACKET_COUNT;
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

static int ConfigureMode(int iMode) //TODO: add password
{
    char    pcSsidName[33]="cctestAP";
    long   lRetVal = -1;

    lRetVal = sl_WlanSetMode(ROLE_AP);
    ASSERT_ON_ERROR(lRetVal);

    lRetVal = sl_WlanSet(SL_WLAN_CFG_AP_ID, WLAN_AP_OPT_SSID, strlen(pcSsidName),
                            (unsigned char*)pcSsidName);
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
    int dataInt;


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
                waveform=atoi(dataString);
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
        }
        freqCount=(endFreq - startFreq) / stepFreq;

    }


    free(tokenList);

    return lRetVal;
}


//****************************************************************************
//
//! \brief Opening a TCP client side socket and sending data
//!
//! This function opens a TCP socket and tries to connect to a Server IP_ADDR
//!    waiting on port PORT_NUM.
//!    If the socket connection is successful then the function will send 1000
//! TCP packets to the server.
//!
//! \param[in]      port number on which the server will be listening on
//!
//! \return    0 on success, -1 on Error.
//
//****************************************************************************
int BsdTcpClient(unsigned short usPort)
{
    int             iCounter;
    short           sTestBufLen;
    SlSockAddrIn_t  sAddr;
    int             iAddrSize;
    int             iSockID;
    int             iStatus;
    long            lLoopCount = 0;

    // filling the buffer
    for (iCounter=0 ; iCounter<BUF_SIZE ; iCounter++)
    {
        g_cBsdBuf[iCounter] = (char)(iCounter % 256);
    }

    sTestBufLen  = BUF_SIZE;

    //filling the TCP server socket address
    sAddr.sin_family = SL_AF_INET;
    sAddr.sin_port = sl_Htons((unsigned short)usPort);
    sAddr.sin_addr.s_addr = sl_Htonl((unsigned int)g_ulDestinationIp);

    iAddrSize = sizeof(SlSockAddrIn_t);

    // creating a TCP socket
    iSockID = sl_Socket(SL_AF_INET,SL_SOCK_STREAM, 0);
    if( iSockID < 0 )
    {
        ASSERT_ON_ERROR(SOCKET_CREATE_ERROR);
    }

    // connecting to TCP server
    iStatus = sl_Connect(iSockID, ( SlSockAddr_t *)&sAddr, iAddrSize);
    if( iStatus < 0 )
    {
        // error
        sl_Close(iSockID);       
        ASSERT_ON_ERROR(CONNECT_ERROR);
    }

    // sending multiple packets to the TCP server
    while (lLoopCount < g_ulPacketCount)
    {
        // sending packet
        iStatus = sl_Send(iSockID, g_cBsdBuf, sTestBufLen, 0 );
        if( iStatus < 0 )
        {
            // error
            sl_Close(iSockID);
            ASSERT_ON_ERROR(SEND_ERROR);
        }
        lLoopCount++;
    }

    Report("Sent %u packets successfully\n\r",g_ulPacketCount);

    iStatus = sl_Close(iSockID);
    //closing the socket after sending 1000 packets
    ASSERT_ON_ERROR(iStatus);

    return SUCCESS;
}

//****************************************************************************
//
//! \brief Opening a TCP server side socket and receiving data
//!
//! This function opens a TCP socket in Listen mode and waits for an incoming
//!    TCP connection.
//! If a socket connection is established then the function will try to read
//!    1000 TCP packets from the connected client.
//!
//! \param[in] port number on which the server will be listening on
//!
//! \return     0 on success, -1 on error.
//!
//! \note   This function will wait for an incoming connection till
//!                     one is established
//
//****************************************************************************
int BsdTcpServer(unsigned short usPort)
{
    SlSockAddrIn_t  sAddr;
    SlSockAddrIn_t  sLocalAddr;
    int             iCounter;
    int             iAddrSize;
    int             iSockID;
    int             iStatus;
    int             iNewSockID;
    long            lLoopCount = 0;
    long            lNonBlocking = 1;
    int             iTestBufLen;
    long lRetVal = 0;


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

    // setting socket option to make the socket as non blocking
    iStatus = sl_SetSockOpt(iSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING, 
                            &lNonBlocking, sizeof(lNonBlocking));
    if( iStatus < 0 )
    {
        sl_Close(iSockID);
        ASSERT_ON_ERROR(SOCKET_OPT_ERROR);
    }

    // waiting for an incoming TCP connection
    while(1){
        _SlNonOsMainLoopTask(); //needed?
        iNewSockID = SL_EAGAIN;
        // Retry in case client gets disconnected.
        //Accepts a connection form a TCP client, if there is any
        // otherwise returns SL_EAGAIN
        iNewSockID = sl_Accept(iSockID, ( struct SlSockAddr_t *)&sAddr,
                                (SlSocklen_t*)&iAddrSize);

        if( iNewSockID == SL_EAGAIN )
        {
           MAP_UtilsDelay(10000);
        }
        else if( iNewSockID < 0 )
        {
            // error
            sl_Close(iNewSockID);
            sl_Close(iSockID);
            ASSERT_ON_ERROR(ACCEPT_ERROR);
        }
        else{
            NewSockID=iNewSockID;//keep,this is the successful socket
            // setting socket option to make the socket as non blocking
            iStatus = sl_SetSockOpt(iNewSockID, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
                                    &lNonBlocking, sizeof(lNonBlocking));   //TODO : test

        }
        if (NewSockID>0){
            do{
                _SlNonOsMainLoopTask(); //needed.
                iStatus = sl_Recv(NewSockID, g_cBsdBuf, BUF_SIZE, 0);}
            while(iStatus<=0);//TODO sometimes istatus gives 0 without communication??
            /*if( iStatus <= 0 )
            {
              // error
              sl_Close(NewSockID);
              //sl_Close(iSockID);
              Report("RECV_ERROR");
            }*/ //NOT NOW
            /*ELSE*/

                Report("Received %d BYTES successfully\n\r",iStatus);
                /* Parse JSON data */
                lRetVal = ParseJSONData(g_cBsdBuf);
                //MAP_UtilsDelay(80000000);//TODO:delay because the phone needs to switch to activity for receiving??
                // sending multiple packets to the TCP server
                lLoopCount=0;
                while (lLoopCount <= freqCount)//send all these packets
                {
                    memset(g_cBsdBuf,0,sizeof(g_cBsdBuf));//clear whole buffer.
                    sprintf(g_cBsdBuf,"{%.2f:{%.2f,%.2f}}\n",(float)lLoopCount,(float)(lLoopCount % 10),(float)(lLoopCount % 5));
                    // sending packet
                    do{
                        iStatus = sl_Send(NewSockID,g_cBsdBuf,BUF_SIZE,0);
                        MAP_UtilsDelay(500000);//TODO remove later.phone picks up the packets fast enough.
                        _SlNonOsMainLoopTask(); //needed.
                        sl_Recv(NewSockID, g_cBsdBuf, BUF_SIZE, 0);//dummy for dumping wrong inputs before finishing
                    }
                    while(iStatus<0);
                    Report("Sent %s\n\r",g_cBsdBuf);
                    lLoopCount++;
                }
                Report("Sent %u packets successfully\n\r",g_ulPacketCount);
                //break;
        }
    }
    
    // close the connected socket after receiving from connected TCP client?
    iStatus = sl_Close(iNewSockID);
    ASSERT_ON_ERROR(iStatus);
    // close the listening socket
    iStatus = sl_Close(iSockID);
    ASSERT_ON_ERROR(iStatus);   
//TODO: maybe switch off nWP here?
    return SUCCESS;
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

    Report("\n\n\n\r");
    Report("\t\t *************************************************\n\r");
    Report("\t\t      CC3200 %s Application       \n\r", AppName);
    Report("\t\t *************************************************\n\r");
    Report("\n\n\n\r");
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
#if defined(ccs) || defined (gcc)
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
    // Initialize the uDMA
    //
    UDMAInit();

    //
    // Configure the pinmux settings for the peripherals exercised
    //
    PinMuxConfig();

    //
    // Configuring UART
    //
    InitTerm();

    //
    // Display banner
    //
    DisplayBanner(APPLICATION_NAME);
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



/*    lRetVal = BsdTcpClient(PORT_NUM);
    if(lRetVal < 0)
    {
        UART_PRINT("TCP Client failed\n\r");
        LOOP_FOREVER();
    }*/
    while(1){
        _SlNonOsMainLoopTask();
        lRetVal = BsdTcpServer(PORT_NUM);
        if(lRetVal < 0)
        {
            UART_PRINT("TCP Server failed\n\r");
            LOOP_FOREVER();
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

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
