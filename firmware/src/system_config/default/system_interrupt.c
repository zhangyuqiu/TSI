/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system/common/sys_common.h"
#include "app_uart.h"
#include "app_can.h"
#include "app_driver_state_fsm.h"
#include "app_i2c.h"
#include "app_dataio.h"
#include "system_definitions.h"
uint32_t count;
uint32_t timer1Count = 0;
uint32_t timer2Count = 0;
uint32_t timer3Count = 0;
uint32_t buzz_Count = 0;
// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

 

void __ISR(_I2C4_MASTER_VECTOR, ipl2AUTO) _IntHandlerDrvI2CMasterInstance0(void)
{
    uint32_t A4 = PORTBbits.RB4;
    PORTBbits.RB4 = ~A4;
	DRV_I2C0_Tasks();
}


void __ISR(_I2C4_BUS_VECTOR, ipl2AUTO) _IntHandlerDrvI2CErrorInstance0(void)
{
    /* TODO: Add code to process interrupt here */
    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_I2C_4_BUS);
}

void __ISR(_TIMER_1_VECTOR, ipl1AUTO) IntHandlerDrvTmrInstance0(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_1);
    
    timer1Count++;
    if (timer1Count == 2) {
        set_MCP23016_READ_flag(0);

    }
    if (timer1Count == 3) {

        set_NCD9830_READ_flag(0);
        set_NCD9830_READ_flag(1);
        set_NCD9830_READ_flag(2);
        set_NCD9830_READ_flag(3);
        set_NCD9830_READ_flag(4);
        set_NCD9830_READ_flag(5);
        set_NCD9830_READ_flag(6);
        set_NCD9830_READ_flag(7);
        timer1Count = 0;
    }
    

    
//    if (count > 2) {
////        uint32_t A = PORTBbits.RB3;
////        PORTBbits.RB3 = ~A;
////        PORTBbits.RB3 = return_Throttp();
//        PORTBbits.RB3 = throttleImplausibility;
//
////        set_NCD9830_READ_flag(0);
////        set_NCD9830_READ_flag(1);
////        set_NCD9830_READ_flag(2);
////        set_NCD9830_READ_flag(3);
////        set_NCD9830_READ_flag(4);
////        set_NCD9830_READ_flag(5);
////        set_NCD9830_READ_flag(6);
////        set_NCD9830_READ_flag(7);
//        send_conditions();
//        count = 0;
////        can_send_bytes(0x300,(uint8_t)get_MCP23016B(),0,0,0,0,0,0,0);
//    }
//    if (!(timer1Count % NCD9830CH0_freq)) {
//        set_NCD9830_READ_flag(0);
////        BSP_LEDToggle(BSP_LED_3);
////        h1 = DRV_I2C0_Transmit(0x90,WriteString,1,NULL);
////        h1 = DRV_I2C0_Receive (0x90,ReadString,1,NULL);
//    }
//    if (!(timer1Count % NCD9830CH1_freq)) {
//        set_NCD9830_READ_flag(1);
////        h1 = DRV_I2C0_Transmit(0x90,0b10010100,1,NULL);
////        h1 = DRV_I2C0_Receive (0x90,ReadString,1,NULL);
//    }
//    if (!(timer1Count % NCD9830CH2_freq)) {
//        set_NCD9830_READ_flag(2);
//        //BSP_LEDToggle(BSP_LED_3);
//    }
//    if (!(timer1Count % NCD9830CH3_freq)) {
//        set_NCD9830_READ_flag(3);
//        //BSP_LEDToggle(BSP_LED_3);
//    }
//    if (!(timer1Count % NCD9830CH4_freq)) {
//        set_NCD9830_READ_flag(4);
//        //BSP_LEDToggle(BSP_LED_3);
//    }
//    if (!(timer1Count % NCD9830CH5_freq)) {
//        set_NCD9830_READ_flag(5);
//        //BSP_LEDToggle(BSP_LED_3);
//    }
//    if (!(timer1Count % NCD9830CH6_freq)) {
//        set_NCD9830_READ_flag(6);
//        //BSP_LEDToggle(BSP_LED_3);
//    }
//    if (!(timer1Count % NCD9830CH7_freq)) {
//        set_NCD9830_READ_flag(7);
//        //BSP_LEDToggle(BSP_LED_3);
//    }
//    if (timer1Count == 10) timer1Count = 0;

//    
//    if (!(timer2Count % MCP23016GPB_freq)) {
//        set_MCP23016_READ_flag(0);
//        //BSP_LEDToggle(BSP_LED_3);
//    }
}
void __ISR(_TIMER_2_VECTOR, ipl3AUTO) IntHandlerDrvTmrInstance1(void)
{
    timer3Count++;
    if (timer3Count % 50) {
        send_conditions();
//
//        uint32_t B3 = PORTBbits.RB3;
//        PORTBbits.RB3 = ~B3;
        timer3Count = 0;
    }
    if (states == 0x4 && buzz_flag == 0 ) {
        if (buzz_Count < 100) {
            buzz_Count ++;
        }
        else {
            buzz_flag = 1;
        }
    }
    if (states != 0x4) {
        buzz_flag = 0;
        buzz_Count = 0;
    }
//    if (states == 0x04 && timer3Count < 30) {
//        BUZZER_CTRL = 1;
//        
//    }
//    else {
//        BUZZER_CTRL = 0;
//        buzz_flag = 0;
//    }
//    
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}
void __ISR(_TIMER_4_VECTOR, ipl3AUTO) IntHandlerDrvTmrInstance2(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_4);
}
 
void __ISR(_CAN1_VECTOR, IPL2AUTO) _IntHandlerDrvCANInstance0(void)
{
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_CAN_1);
}



 /*******************************************************************************
 End of File
*/
