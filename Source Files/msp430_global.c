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
int RFID_data_status = 0;
char RFID_data[14];
char RFID_1[10] = {'4','E','0','0','F','0','F','5','5','1'};
char RFID_2[10] = {'4','E','0','0','F','0','F','5','5','2'};
char RFID_3[10] = {'4','E','0','0','F','0','F','5','5','3'};
char RFID_4[10] = {'4','E','0','0','F','0','F','5','5','4'};
unsigned int pressed=0;
unsigned int p_conf=0;
unsigned int r_conf=0;
int Esc_flag = 0;
char uarta1_buffer[100]={0};
unsigned int index = 0;
unsigned int page_ctr = 0;
unsigned int page_len = 0;
unsigned int i2c_eeprom_add = 0;
unsigned int pankil = 0;
char done = 0;

char lcd[50];
/**
 * @}
 */



/* End of Public Functions ---------------------------------------------------- */

/**
 * @}
 */

/* --------------------------------- End Of File ------------------------------ */
