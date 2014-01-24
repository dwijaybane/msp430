/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#ifndef __MSP430WARE_TIMER_D_H__
#define __MSP430WARE_TIMER_D_H__

#define __MSP430_HAS_TxD7__
//*****************************************************************************
//
//The following are values that can be passed to the
//TIMER_D_startContinuousMode(), TIMER_D_startUpMode(), TIMER_D_startUpDownMode(),
//TIMER_D_generatePWM() APIs as the clockSource parameter.
//
//*****************************************************************************
#define TIMER_D_CLOCKSOURCE_EXTERNAL_TDCLK            TDSSEL__TACLK
#define TIMER_D_CLOCKSOURCE_ACLK                      TDSSEL__ACLK
#define TIMER_D_CLOCKSOURCE_SMCLK                     TDSSEL__SMCLK
#define TIMER_D_CLOCKSOURCE_INVERTED_EXTERNAL_TDCLK   TDSSEL__INCLK

//*****************************************************************************
//
//The following are values that can be passed to the
//TIMER_D_startContinuousMode(), TIMER_D_startUpMode(), TIMER_D_startUpDownMode(),
//TIMER_D_generatePWM() APIs as the clockSource parameter.
//
//*****************************************************************************
#define TIMER_D_CLOCKINGMODE_EXTERNAL_CLOCK 		    	TDCLKM_0
#define TIMER_D_CLOCKINGMODE_HIRES_LOCAL_CLOCK			TDCLKM_1
#define TIMER_D_CLOCKINGMODE_AUXILIARY_CLK			TDCLKM_2

//*****************************************************************************
//
//The following are values that can be passed to the
//TIMER_D_configureHighResGeneratorInRegulatedMode()
// API as the highResMultiplyFactor parameter.
//
//*****************************************************************************
#define TIMER_D_HIGHRES_CLK_MULTIPLY_FACTOR_8x	TDHM_0
#define TIMER_D_HIGHRES_CLK_MULTIPLY_FACTOR_16x	TDHM_1

//*****************************************************************************
//
//The following are values that can be passed to the
//TIMER_D_configureHighResGeneratorInRegulatedMode()
// API as the highResClockDivider parameter.
//
//*****************************************************************************
#define TIMER_D_HIGHRES_CLK_DIVIDER_1	TDHD__1
#define TIMER_D_HIGHRES_CLK_DIVIDER_2	TDHD__2
#define TIMER_D_HIGHRES_CLK_DIVIDER_4	TDHD__4
#define TIMER_D_HIGHRES_CLK_DIVIDER_8	TDHD__8

//*****************************************************************************
//
//The following are values that can be passed to the
//TIMER_D_startContinuousMode(), TIMER_D_startUpMode(), TIMER_D_startUpDownMode(),
//TIMER_D_generatePWM() APIs as the clockSourceDivider parameter.
//
//*****************************************************************************
#define TIMER_D_CLOCKSOURCE_DIVIDER_1     0x01
#define TIMER_D_CLOCKSOURCE_DIVIDER_2     0x02
#define TIMER_D_CLOCKSOURCE_DIVIDER_4     0x04
#define TIMER_D_CLOCKSOURCE_DIVIDER_8     0x08
#define TIMER_D_CLOCKSOURCE_DIVIDER_3     0x03
#define TIMER_D_CLOCKSOURCE_DIVIDER_5     0x05
#define TIMER_D_CLOCKSOURCE_DIVIDER_6     0x06
#define TIMER_D_CLOCKSOURCE_DIVIDER_7     0x07
#define TIMER_D_CLOCKSOURCE_DIVIDER_10    0x0A
#define TIMER_D_CLOCKSOURCE_DIVIDER_12    0x0C
#define TIMER_D_CLOCKSOURCE_DIVIDER_14    0x0E
#define TIMER_D_CLOCKSOURCE_DIVIDER_16    0x10
#define TIMER_D_CLOCKSOURCE_DIVIDER_20    0x14
#define TIMER_D_CLOCKSOURCE_DIVIDER_24    0x18
#define TIMER_D_CLOCKSOURCE_DIVIDER_28    0x1C
#define TIMER_D_CLOCKSOURCE_DIVIDER_32    0x20
#define TIMER_D_CLOCKSOURCE_DIVIDER_40    0x28
#define TIMER_D_CLOCKSOURCE_DIVIDER_48    0x30
#define TIMER_D_CLOCKSOURCE_DIVIDER_56    0x38
#define TIMER_D_CLOCKSOURCE_DIVIDER_64    0x40

//*****************************************************************************
//
//The following are values that can be passed to TIMER_D_startContinuousMode()
//TIMER_D_startUpMode(),  TIMER_D_startUpDownMode() as the timerClear parameter.
//
//*****************************************************************************
#define TIMER_D_DO_CLEAR      TDCLR
#define TIMER_D_SKIP_CLEAR    0x00

//*****************************************************************************
//
//The following are values that can be passed to the
//TIMER_D_getSynchronizedCaptureCompareInput() API as the synchronized
//parameter.
//
//*****************************************************************************
#define TIMER_D_CAPTURECOMPARE_INPUT     CCI

//*****************************************************************************
//
//The following are values that is returned by the
//TIMER_D_getSynchronizedCaptureCompareInput() API
//
//*****************************************************************************
#define TIMER_D_CAPTURECOMPARE_INPUT_HIGH    0x01
#define TIMER_D_CAPTURECOMPARE_INPUT_LOW     0x00


//*****************************************************************************
//
//The following are values that is returned by the
//TIMER_D_getOutputForOutputModeOutBitValue() and passed to
//TIMER_D_setOutputForOutputModeOutBitValue() as
//outputModeOutBitValue parameter
//
//*****************************************************************************
#define TIMER_D_OUTPUTMODE_OUTBITVALUE_HIGH    OUT
#define TIMER_D_OUTPUTMODE_OUTBITVALUE_LOW     0x00

//*****************************************************************************
//
//The following are values can be passed to the mask parameter of
//TIMER_D_captureCompareInterruptStatus() API
//
//*****************************************************************************
#define TIMER_D_CAPTURE_OVERFLOW                  COV
#define TIMER_D_CAPTURECOMPARE_INTERRUPT_FLAG     CCIFG

//*****************************************************************************
//
//The following are values can be passed to the mask parameter of
//TIMER_D_enableHighResInterrupt(), TIMER_D_getHighResInterruptStatus(),
// TIMER_D_disableHighResInterrupt() API
//
//*****************************************************************************
#define TIMER_D_HIGH_RES_FREQUENCY_UNLOCK   TDHUNLKIE
#define TIMER_D_HIGH_RES_FREQUENCY_LOCK     TDHLKIE
#define TIMER_D_HIGH_RES_FAIL_HIGH          TDHFHIE
#define TIMER_D_HIGH_RES_FAIL_LOW     	  TDHFLIE

//*****************************************************************************
//
//The following are values can be passed to the timerInterruptEnable_TDIE
//parameter of TIMER_D_startContinuousMode(), TIMER_D_startUpMode(),
//TIMER_D_startUpDownMode()
//
//*****************************************************************************
#define TIMER_D_TDIE_INTERRUPT_ENABLE            TDIE
#define TIMER_D_TDIE_INTERRUPT_DISABLE           0x00

//*****************************************************************************
//
//The following are values can be passed to the
//captureCompareInterruptEnable_CCR0_CCIE parameter of TIMER_D_startUpMode(),
//TIMER_D_startUpDownMode API
//
//*****************************************************************************
#define TIMER_D_CCIE_CCR0_INTERRUPT_ENABLE   CCIE
#define TIMER_D_CCIE_CCR0_INTERRUPT_DISABLE  0x00

//*****************************************************************************
//
//The following are timer modes possible.
//
//*****************************************************************************
#define TIMER_D_STOP_MODE         MC_0
#define TIMER_D_UP_MODE           MC_1
#define TIMER_D_CONTINUOUS_MODE   MC_2
#define TIMER_D_UPDOWN_MODE       MC_3

//*****************************************************************************
//
//The following are values can be passed to the
//compareRegister, captureCompareRegister or captureRegister parameter
//of TIMER_D_initCapture(), TIMER_D_enableCaptureCompareInterrupt(),
//TIMER_D_disableCaptureCompareInterrupt(),TIMER_D_captureCompareInterruptStatus(),
//TIMER_D_getSynchronizedCaptureCompareInput(),TIMER_D_initCompare()
//
//*****************************************************************************
#define TIMER_D_CAPTURECOMPARE_REGISTER_0     0x08
#define TIMER_D_CAPTURECOMPARE_REGISTER_1     0x0E
#define TIMER_D_CAPTURECOMPARE_REGISTER_2     0x14
#define TIMER_D_CAPTURECOMPARE_REGISTER_3     0x1A
#define TIMER_D_CAPTURECOMPARE_REGISTER_4     0x20
#define TIMER_D_CAPTURECOMPARE_REGISTER_5     0x28
#define TIMER_D_CAPTURECOMPARE_REGISTER_6     0x2E

//*****************************************************************************
//
//The following are values can be passed to the
//captureMode parameter of TIMER_D_initCompare(), TIMER_D_initCapture(),
//TIMER_D_generatePWM(),
//
//*****************************************************************************
#define TIMER_D_OUTPUTMODE_OUTBITVALUE        OUTMOD_0
#define TIMER_D_OUTPUTMODE_SET                OUTMOD_1
#define TIMER_D_OUTPUTMODE_TOGGLE_RESET       OUTMOD_2
#define TIMER_D_OUTPUTMODE_SET_RESET          OUTMOD_3
#define TIMER_D_OUTPUTMODE_TOGGLE             OUTMOD_4
#define TIMER_D_OUTPUTMODE_RESET              OUTMOD_5
#define TIMER_D_OUTPUTMODE_TOGGLE_SET         OUTMOD_6
#define TIMER_D_OUTPUTMODE_RESET_SET          OUTMOD_7

//*****************************************************************************
//
//The following are values can be passed to the
//captureMode parameter of TIMER_D_initCapture() API
//
//*****************************************************************************
#define TIMER_D_CAPTUREMODE_NO_CAPTURE                CM_0
#define TIMER_D_CAPTUREMODE_RISING_EDGE               CM_1
#define TIMER_D_CAPTUREMODE_FALLING_EDGE              CM_2
#define TIMER_D_CAPTUREMODE_RISING_AND_FALLING_EDGE   CM_3

//*****************************************************************************
//
//The following are values can be passed to the
//synchronizeCaptureSource parameter of TIMER_D_initCapture() API
//
//*****************************************************************************
#define TIMER_D_CAPTURE_ASYNCHRONOUS                  0x00
#define TIMER_D_CAPTURE_SYNCHRONOUS                   SCS

//*****************************************************************************
//
//The following are values can be passed to the
//channelCaptureMode parameter of TIMER_D_initCapture() API
//
//*****************************************************************************
#define TIMER_D_SINGLE_CAPTURE_MODE                 0x00
#define TIMER_D_DUAL_CAPTURE_MODE                   0x01
//*****************************************************************************
//
//The following are values can be passed to the
//captureInterruptEnable, compareInterruptEnable parameter of
//TIMER_D_initCapture() API
//
//*****************************************************************************
#define TIMER_D_CAPTURECOMPARE_INTERRUPT_ENABLE       CCIE
#define TIMER_D_CAPTURECOMPARE_INTERRUPT_DISABLE      0x00

//*****************************************************************************
//
//The following are values can be passed to the
//captureInputSelect parameter of TIMER_D_initCapture() API
//
//*****************************************************************************
#define TIMER_D_CAPTURE_INPUTSELECT_CCIxA             CCIS_0
#define TIMER_D_CAPTURE_INPUTSELECT_CCIxB             CCIS_1
#define TIMER_D_CAPTURE_INPUTSELECT_GND               CCIS_2
#define TIMER_D_CAPTURE_INPUTSELECT_Vcc               CCIS_3

//*****************************************************************************
//
//The following are values that may be returned by
//TIMER_D_getInterruptStatus() API
//
//*****************************************************************************
#define TIMER_D_INTERRUPT_NOT_PENDING     0x00
#define TIMER_D_INTERRUPT_PENDING         0x01

//*****************************************************************************
//
//The following are values can be passed to the
//synchronized parameter of TIMER_D_getSynchronizedCaptureCompareInput() API
//
//*****************************************************************************
#define TIMER_D_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT SCCI
#define TIMER_D_READ_CAPTURE_COMPARE_INPUT            CCI

//*****************************************************************************
//
//The following are values can be passed to the
//desiredHighResFrequency parameter of 
//TIMER_D_configureHighResGeneratorInFreeRunningMode() API
//
//*****************************************************************************
#define TIMER_D_HIGHRES_64MHZ	0x00
#define TIMER_D_HIGHRES_128MHZ	0x01
#define TIMER_D_HIGHRES_200MHZ	0x02
#define TIMER_D_HIGHRES_256MHZ	0x03   

//*****************************************************************************
//
//The following are values can be passed to the
//TDOutput parameter of 
//TIMER_D_combineTDCCRToGeneratePWM() API
//
//*****************************************************************************
#define TIMER_D_COMBINE_CCR1_CCR2	2
#define TIMER_D_COMBINE_CCR3_CCR4	4
#define TIMER_D_COMBINE_CCR5_CCR6	6

//*****************************************************************************
//
//The following are values can be passed to the groupLatch parameter of 
//TIMER_D_combineTDCCRToGeneratePWM() API
//
//*****************************************************************************
#define TIMER_D_GROUP_NONE			TDCLGRP_0
#define TIMER_D_GROUP_CL12_CL23_CL56	TDCLGRP_1
#define TIMER_D_GROUP_CL123_CL456	TDCLGRP_2
#define TIMER_D_GROUP_ALL			TDCLGRP_3

//*****************************************************************************
//
//The following are values can be passed to the counterLength parameter of 
//TIMER_D_selectCounterLength() API
//
//*****************************************************************************
#define TIMER_D_COUNTER_16BIT		CNTL_0
#define TIMER_D_COUNTER_12BIT		CNTL_1
#define TIMER_D_COUNTER_10BIT		CNTL_2
#define TIMER_D_COUNTER_8BIT			CNTL_3

//*****************************************************************************
//
//The following are values can be passed to the compareLatchLoadEvent parameter 
//of TIMER_D_initCompareLatchLoadEvent() API
//
//*****************************************************************************
#define TIMER_D_LATCH_ON_WRITE_TO_TDxCCRn_COMPARE_REGISTER				CLLD_0
#define TIMER_D_LATCH_WHEN_COUNTER_COUNTS_TO_0_IN_UP_OR_CONT_MODE		CLLD_1
#define TIMER_D_LATCH_WHEN_COUNTER_COUNTS_TO_0_IN_UPDOWN_MODE			CLLD_2
#define TIMER_D_LATCH_WHEN_COUNTER_COUNTS_TO_CURRENT_COMPARE_LATCH_VALUE	CLLD_3

//*****************************************************************************
//
//The following are values can be passed to the highResCoarseClockRange
//parameter of  TIMER_D_selectHighResClockRange() API
//
//*****************************************************************************
#define TIMER_D_CLOCK_RANGE0 		0x0000
#define TIMER_D_CLOCK_RANGE1			0x2000
#define TIMER_D_CLOCK_RANGE2			0x4000

//*****************************************************************************
//
//The following are values can be passed to the
//TDOutput parameter of
//TIMER_D_selectHighResCoarseClockRange() API
//
//*****************************************************************************
#define TIMER_D_HIGHRES_BELOW_15MHz  0x00
#define TIMER_D_HIGHRES_ABOVE_15MHz  TDHCLKCR


//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************
extern void TIMER_D_startCounter ( unsigned int baseAddress,
    unsigned int timerMode
    );
extern void TIMER_D_configureContinuousMode ( unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int clockingMode,
    unsigned int timerInterruptEnable_TDIE,
    unsigned int timerClear
    );
extern void TIMER_D_configureUpMode (   unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int clockingMode,
    unsigned int timerPeriod,
    unsigned int timerInterruptEnable_TDIE,
    unsigned int captureCompareInterruptEnable_CCR0_CCIE,
    unsigned int timerClear
    );
extern void TIMER_D_configureUpDownMode (
    unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int clockingMode,
    unsigned int timerPeriod,
    unsigned int timerInterruptEnable_TDIE,
    unsigned int captureCompareInterruptEnable_CCR0_CCIE,
    unsigned int timerClear
    );
extern void TIMER_D_initCapture (unsigned int baseAddress,
    unsigned int captureRegister,
    unsigned int captureMode,
    unsigned int captureInputSelect,
    unsigned short synchronizeCaptureSource,
    unsigned short captureInterruptEnable,
    unsigned int captureOutputMode,
    unsigned char channelCaptureMode
    );
extern void TIMER_D_initCompare (  unsigned int baseAddress,
    unsigned int compareRegister,
    unsigned short compareInterruptEnable,
    unsigned int compareOutputMode,
    unsigned int compareValue
    );
extern void TIMER_D_enableTimerInterrupt (unsigned int baseAddress);
extern void TIMER_D_enableHighResInterrupt (unsigned int baseAddress,
				unsigned int mask);
extern void TIMER_D_disableTimerInterrupt (unsigned int baseAddress);

extern void TIMER_D_disableHighResInterrupt (unsigned int baseAddress,
						unsigned int mask);
extern unsigned long TIMER_D_getTimerInterruptStatus (unsigned int baseAddress);
extern void TIMER_D_enableCaptureCompareInterrupt (unsigned int baseAddress,
    unsigned int captureCompareRegister
    );
extern void TIMER_D_disableCaptureCompareInterrupt (unsigned int baseAddress,
    unsigned int captureCompareRegister
    );
extern unsigned long TIMER_D_getCaptureCompareInterruptStatus (unsigned int baseAddress,
	    unsigned int captureCompareRegister,
	    unsigned int mask
	    );
extern unsigned int TIMER_D_getHighResInterruptStatus (unsigned int baseAddress,
    unsigned int mask);

extern void TIMER_D_clear (unsigned int baseAddress);
extern void TIMER_D_clearHighResInterruptStatus (unsigned int baseAddress,
    unsigned int mask);
extern unsigned short TIMER_D_getSynchronizedCaptureCompareInput
    (unsigned int baseAddress,
    unsigned int captureCompareRegister,
    unsigned short synchronized
    );
extern unsigned char TIMER_D_getOutputForOutputModeOutBitValue
    (unsigned int baseAddress,
    unsigned int captureCompareRegister
    );
extern unsigned int TIMER_D_getCaptureCompareCount
    (unsigned int baseAddress,
    unsigned int captureCompareRegister
    );
extern unsigned int TIMER_D_getCaptureCompareLatchCount
    (unsigned int baseAddress,
    unsigned int captureCompareRegister
    );
extern unsigned char TIMER_D_getCaptureCompareInputSignal
    (unsigned int baseAddress,
    unsigned int captureCompareRegister
    );
extern void TIMER_D_setOutputForOutputModeOutBitValue
    (unsigned int baseAddress,
    unsigned int captureCompareRegister,
    unsigned char outputModeOutBitValue
    );
extern void TIMER_D_generatePWM (  unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int clockingMode,
    unsigned int timerPeriod,
    unsigned int compareRegister,
    unsigned int compareOutputMode,
    unsigned int dutyCycle
    );
extern void TIMER_D_stop ( unsigned int baseAddress );
extern void TIMER_D_setCompareValue (  unsigned int baseAddress,
    unsigned int compareRegister,
    unsigned int compareValue
    );
extern void TIMER_D_clearTimerInterruptFlag (unsigned int baseAddress);
extern void TIMER_D_clearCaptureCompareInterruptFlag (unsigned int baseAddress,
    unsigned int captureCompareRegister
    );
extern unsigned char TIMER_D_configureHighResGeneratorInFreeRunningMode
	(unsigned int baseAddress,
    unsigned char desiredHighResFrequency
    );
extern void TIMER_D_configureHighResGeneratorInRegulatedMode (unsigned int baseAddress,
     unsigned int clockSource, 
     unsigned int clockSourceDivider,
     unsigned int clockingMode, 
     unsigned char highResClockMultiplyFactor,
     unsigned char highResClockDivider
    );
extern void TIMER_D_combineTDCCRToGeneratePWM (  unsigned int baseAddress,
	    unsigned int clockSource,
	    unsigned int clockSourceDivider,
            unsigned int clockingMode,
	    unsigned int timerPeriod,
	    unsigned int combineCCRRegistersCombination,
	    unsigned int compareOutputMode,
	    unsigned int dutyCycle1,
	    unsigned int dutyCycle2
	    );
extern void TIMER_D_selectLatchingGroup(unsigned int  baseAddress,
		unsigned int  groupLatch);
extern void TIMER_D_selectCounterLength (unsigned int  baseAddress,
		unsigned int counterLength
		);
extern void TIMER_D_initCompareLatchLoadEvent(unsigned int  baseAddress,
		unsigned int  compareRegister,
		unsigned int  compareLatchLoadEvent
		);
extern void TIMER_D_disableHighResFastWakeup (unsigned int baseAddress);
extern void TIMER_D_enableHighResFastWakeup (unsigned int baseAddress);
extern void TIMER_D_disableHighResClockEnhancedAccuracy (unsigned int baseAddress);
extern void TIMER_D_enableHighResClockEnhancedAccuracy (unsigned int baseAddress);
extern void TIMER_D_DisableHighResGeneratorForceON (unsigned int baseAddress);
extern void TIMER_D_EnableHighResGeneratorForceON (unsigned int baseAddress);

extern void TIMER_D_selectHighResCoarseClockRange (unsigned int baseAddress,
		unsigned int highResCoarseClockRange
		);
extern void TIMER_D_selectHighResClockRange (unsigned int baseAddress,
		unsigned int highResClockRange
		);

//*****************************************************************************
//
//Close the Doxygen group.
//! @}
//
//*****************************************************************************

#endif
