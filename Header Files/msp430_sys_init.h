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
#include "stdarg.h"

/* Peripherals include------------------------------------------ */
#include "wdt_a.h"
#include "gpio.h"
#include "timer_a.h"
#include "timer_b.h"
#include "timer_d.h"
#include "pmm.h"
#include "eusci_uart.h"
#include "messages.h"
#include "test_diag.h"
#include "lcd_c.h"
#include "adc10_a.h"
#include "sd24_b.h"
#include "mpy32.h"

/* Global Variables---------------------------------------------*/
extern unsigned int hrt_bt_timer;
extern unsigned int delay_timer;

extern char String[10];
extern unsigned int p;
extern unsigned int word_count;
extern unsigned int Esc_Flag;
extern unsigned int r_flag;
extern int RFID_data_status;
extern char RFID_data[14];

extern char RFID_1[10];
extern char RFID_2[10];
extern char RFID_3[10];
extern char RFID_4[10];
extern unsigned int pressed;
extern unsigned int p_conf;
extern unsigned int r_conf;
extern int Esc_flag;
extern char uarta1_buffer[100];
extern unsigned int index;

extern unsigned int page_ctr;
extern unsigned int page_len;
extern unsigned int i2c_eeprom_add;
extern unsigned int pankil;
extern char lcd[50];
extern char done;

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

