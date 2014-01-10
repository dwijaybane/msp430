#include "msp430.h"
/*
 * clock_calib.c
 *
 *  Created on: Dec 31, 2013
 *      Author: acer
 */
void clock_calib()
{
	if (CALBC1_1MHZ == 0xFF||CALDCO_1MHZ == 0xFF)
		while(1);	//TRAP THE CPU IF CALIBRATION CONSTANTS ARE ERASED.
	BCSCTL1 = CALBC1_1MHZ;		//set the range
	DCOCTL = CALDCO_1MHZ;		//set the DCO step + mod bits.

	BCSCTL3 |= LFXT1S_2;		//select VLO
	IFG1 &= ~OFIFG;				//clear the oscillator fault flag.
	BCSCTL2 = SELM_0 + DIVM_3;	//select the DCO for MCLK and divide MCLK by 8.OSCFault flag
}
