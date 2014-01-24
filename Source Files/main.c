/*****************************************************************************
* $Id$			main.c
*//**
* @file        	main.c
* @brief       	This example is about to Toggle LED
* @version     	1.0
* @date        	22. Jan. 2014
* @author       Pankil.Edutech
******************************************************************************/

#include "msp430_sys_init.h"

/* Example group ----------------------------------------------------------- */
/** @defgroup LED_Blinking        Timer Interrupt
 * @ingroup GPIO_Examples
 * @{
 */

/*-------------------------MAIN Page------------------------------------------*/
/** @mainpage LED_Blinking: GPIO Test Example
*   @par Description:
*   - Connect The Experiment Kit As Required
*   - Install Necessary Driver For Operation Of Experiment Kit
*   - Run The Program In CCS Appropriate Version.
*
*   @par Activity - LED Will Get Blink.
*/
/*-------------------------MAIN FUNCTION--------------------------------------*/
/****************************************************************************//**
 * @brief        Main LED Blinking Program
 *               Blink the LED Which is Selected
 * @param[in]	 LED Which We Want to Blink
 * @return		 LED will Get Blink
 *******************************************************************************/
void main (void)
{
	Sys_Init();												// Initialize the System

	/* Loop Forever */
	while(1)
	{
		GPIO_toggleOutputOnPin(GPIO_PORT_P5,GPIO_PIN1);		// Toggle the Output Pin Selected

		Delay_Ms(1000);										// Provides Delay in ms
	}
}


/*
 * @}
 */
