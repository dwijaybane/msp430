/**********************************************************************
* $Id$				msp430_sys_init.h
*//**
* @file            	msp430_sys_init.h
* @brief        	Contains all functions declaration for SYSTEM INIT library on MSP430F6736
* @date             20 Jan. 2014
* @author			Pankil.EduTech
***********************************************************************/

/* Peripheral group ----------------------------------------------------------- */
/** @defgroup System_Init System Init
 *  @ingroup Library_Drivers
 *  @{
 */
#ifndef MSP430_SYS_INIT_H_
#define MSP430_SYS_INIT_H_

/* includes----------------------------------------------------- */
#include "ucs.h"
#include "hw_memmap.h"
#include "hw_types.h"

/* Peripherals include------------------------------------------ */
#include "wdt_a.h"
#include "gpio.h"
#include "timer_a.h"
#include "timer_b.h"
#include "timer_d.h"



/* Public Functions ----------------------------------------------------------- */
/** @defgroup System_Init_Public_function
* @{
*/
void Sys_Init(void);
void Port_Init(void);

/**
* @}
*/

#endif /* MSP430_SYS_INIT_H_ */

/**
 * @}
 */

