/*************************************************************//**
* @file      	msp430_sys_init.c
* @brief        Basic System Initialization File
* @version      1.0
* @date         22. Jan. 2014
* @author       Pankil.Edutech
***********************************************************************/

/** @addtogroup System_Init        System Initialization
 *  @{
 */
#include "msp430_sys_init.h"

/** @addtogroup System_Init_Public_function        System Public Function
 *  @{
 */
/*****************************************************************//**
* @brief        		General System Initialization function
* @param[in]        	None
* @return           	None
**********************************************************************/
void Sys_Init(void)
{
	WDT_A_hold(WDT_A_BASE);								// Holds the WatchDog Timer

	Timer_Init(TIMER_0);								// Initialize Timers for Operation

	Port_Init();										// Initialize GPIO Ports for Operation

	 _bis_SR_register(GIE);								// Enable All the Global Interrupts
}

/***************************************************************************//**
* @brief        	Port Initialization Fucntion For All Leds And Input Switch
* @param[in]        None
* @return           None
*******************************************************************************/
void Port_Init(void)
{
	GPIO_setAsOutputPin(5,_SBF(0,0x0F));
	GPIO_setOutputHighOnPin(5,_SBF(0,0x0F));

	GPIO_setAsInputPin(1,_SBF(6,0x03));
	GPIO_setAsInputPinWithPullUpresistor(1,_SBF(6,0x03));
}
/*
 * @}
 */

/*  ---------------------------------End of Functions-------------------------- */

/*
 * @}
 */
/* --------------------------------- End Of File ------------------------------ */
