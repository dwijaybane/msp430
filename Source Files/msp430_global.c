/********************************************************************//**
* $Id$		msp430_global.c
*//**
* @file		msp430_global.c
* @brief	Contains Global Variables of MSP430f6736
* @version	1.0
* @date		28. Jan. 2014
* @author	Pankil.Edutech
***********************************************************************/

/** @addtogroup GLOBAL_Variables
 * @{
 */

/**
 *   Systick
 */

unsigned int hrt_bt_timer = 100;			// Initialized for 1 ms.
unsigned int delay_timer = 0;
char String[10];
unsigned int p;
unsigned int word_count = 0;
unsigned int Esc_Flag = 0;
unsigned int r_flag = 0;
unsigned int page_ctr = 0;
unsigned int page_len = 0;
unsigned int i2c_eeprom_add = 0;
unsigned int pankil = 0;
/**
 * @}
 */



/* End of Public Functions ---------------------------------------------------- */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
