
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

// Driverlib includes
#include "utils.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_adc.h"
#include "hw_ints.h"
#include "hw_gprcm.h"
#include "rom.h"
#include "rom_map.h"
#include "interrupt.h"
#include "prcm.h"
#include "uart.h"
#include "pinmux.h"
#include "pin.h"

#include "uart_if.h"

#define UART_PRINT         Report


//*****************************************************************************
//                      GLOBAL VARIABLES
//*****************************************************************************
#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif


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
char k[32],*p,a[128];
int size;
void
main()
{
    BoardInit();

    PinMuxConfig();
    InitTerm();

    while(1)
    {
        p="m8";
        sprintf(k,"\n\r ok,%s.write on: \n\r",p);
        //UART_PRINT(k);        //or,
        UART_PRINT("%s",k);
        //
        // Read inputs from user
        //
        size = sizeof(a) / sizeof(a[0]);
        GetCmd(a,size);
        UART_PRINT("You wrote:\n\r%s\n\r",a);

        MAP_UtilsDelay(10000000);
        }
        //UART_PRINT("\n\rVoltage is %f\n\r",((pulAdcSamples[4] >> 2 ) & 0x0FFF)*1.4/4096);
        UART_PRINT("\n\r");

    }


//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
