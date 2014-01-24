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
#ifndef __MSP430WARE_TIMER_B_H__
#define __MSP430WARE_TIMER_B_H__

#define __MSP430_HAS_TxB7__
//*****************************************************************************
//
//The following are values that can be passed to the
//TIMER_B_startContinuousMode();, TIMER_B_startUpMode();, TIMER_B_startUpDownMode();,
//TIMER_B_generatePWM(); APIs as the clockSource parameter.
//
//*****************************************************************************
#define TIMER_B_CLOCKSOURCE_EXTERNAL_TXCLK            TBSSEL__TACLK
#define TIMER_B_CLOCKSOURCE_ACLK                      TBSSEL__ACLK
#define TIMER_B_CLOCKSOURCE_SMCLK                     TBSSEL__SMCLK
#define TIMER_B_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK   TBSSEL__INCLK

//*****************************************************************************
//
//The following are values that can be passed to the
//TIMER_B_startContinuousMode();, TIMER_B_startUpMode();, TIMER_B_startUpDownMode();,
//TIMER_B_generatePWM(); APIs as the clockSourceDivider parameter.
//
//*****************************************************************************
#define TIMER_B_CLOCKSOURCE_DIVIDER_1     0x01
#define TIMER_B_CLOCKSOURCE_DIVIDER_2     0x02
#define TIMER_B_CLOCKSOURCE_DIVIDER_4     0x04
#define TIMER_B_CLOCKSOURCE_DIVIDER_8     0x08
#define TIMER_B_CLOCKSOURCE_DIVIDER_3     0x03
#define TIMER_B_CLOCKSOURCE_DIVIDER_5     0x05
#define TIMER_B_CLOCKSOURCE_DIVIDER_6     0x06
#define TIMER_B_CLOCKSOURCE_DIVIDER_7     0x07
#define TIMER_B_CLOCKSOURCE_DIVIDER_10    0x0A
#define TIMER_B_CLOCKSOURCE_DIVIDER_12    0x0C
#define TIMER_B_CLOCKSOURCE_DIVIDER_14    0x0E
#define TIMER_B_CLOCKSOURCE_DIVIDER_16    0x10
#define TIMER_B_CLOCKSOURCE_DIVIDER_20    0x14
#define TIMER_B_CLOCKSOURCE_DIVIDER_24    0x18
#define TIMER_B_CLOCKSOURCE_DIVIDER_28    0x1C
#define TIMER_B_CLOCKSOURCE_DIVIDER_32    0x20
#define TIMER_B_CLOCKSOURCE_DIVIDER_40    0x28
#define TIMER_B_CLOCKSOURCE_DIVIDER_48    0x30
#define TIMER_B_CLOCKSOURCE_DIVIDER_56    0x38
#define TIMER_B_CLOCKSOURCE_DIVIDER_64    0x40

//*****************************************************************************
//
//The following are values that can be passed to TIMER_B_startContinuousMode();
//TIMER_B_startUpMode();,  TIMER_B_startUpDownMode(); as the timerClear parameter.
//
//*****************************************************************************
#define TIMER_B_DO_CLEAR      TBCLR
#define TIMER_B_SKIP_CLEAR    0x00

//*****************************************************************************
//
//The following are values that can be passed to the
//TIMER_B_getSynchronizedCaptureCompareInput(); API as the synchronized
//parameter.
//
//*****************************************************************************
#define TIMER_B_CAPTURECOMPARE_INPUT                  SCCI
#define TIMER_B_SYNCHRONIZED_CAPTURECOMPARE_INPUT     CCI

//*****************************************************************************
//
//The following are values that is returned by the
//TIMER_B_getSynchronizedCaptureCompareInput(); API
//
//*****************************************************************************
#define TIMER_B_CAPTURECOMPARE_INPUT_HIGH    0x01
#define TIMER_B_CAPTURECOMPARE_INPUT_LOW     0x00


//*****************************************************************************
//
//The following are values that is returned by the
//TIMER_B_getOutputForOutputModeOutBitValue(); and passed to
//TIMER_B_setOutputForOutputModeOutBitValue(); as
//outputModeOutBitValue parameter
//
//*****************************************************************************
#define TIMER_B_OUTPUTMODE_OUTBITVALUE_HIGH    OUT
#define TIMER_B_OUTPUTMODE_OUTBITVALUE_LOW     0x00

//*****************************************************************************
//
//The following are values can be passed to the mask parameter of
//TIMER_B_captureCompareInterruptStatus(); API
//
//*****************************************************************************
#define TIMER_B_CAPTURE_OVERFLOW                  COV
#define TIMER_B_CAPTURECOMPARE_INTERRUPT_FLAG     CCIFG

//*****************************************************************************
//
//The following are values can be passed to the timerInterruptEnable_TBIE
//parameter of TIMER_B_startContinuousMode();, TIMER_B_startUpMode();,
//TIMER_B_startUpDownMode();
//
//*****************************************************************************
#define TIMER_B_TBIE_INTERRUPT_ENABLE            TBIE
#define TIMER_B_TBIE_INTERRUPT_DISABLE           0x00

//*****************************************************************************
//
//The following are values can be passed to the
//captureCompareInterruptEnable_CCR0_CCIE parameter of TIMER_B_startUpMode();,
//TIMER_B_startUpDownMode API
//
//*****************************************************************************
#define TIMER_B_CCIE_CCR0_INTERRUPT_ENABLE   CCIE
#define TIMER_B_CCIE_CCR0_INTERRUPT_DISABLE  0x00

//*****************************************************************************
//
//The following are timer modes possible.
//
//*****************************************************************************
#define TIMER_B_STOP_MODE         MC_0
#define TIMER_B_UP_MODE           MC_1
#define TIMER_B_CONTINUOUS_MODE   MC_2
#define TIMER_B_UPDOWN_MODE       MC_3

//*****************************************************************************
//
//The following are values can be passed to the
//compareRegister, captureCompareRegister or captureRegister parameter
//of TIMER_B_initCapture();, TIMER_B_enableCaptureCompareInterrupt();,
//TIMER_B_disableCaptureCompareInterrupt();,TIMER_B_captureCompareInterruptStatus();,
//TIMER_B_getSynchronizedCaptureCompareInput();,TIMER_B_initCompare();
//
//*****************************************************************************
#define TIMER_B_CAPTURECOMPARE_REGISTER_0     0x02
#define TIMER_B_CAPTURECOMPARE_REGISTER_1     0x04
#define TIMER_B_CAPTURECOMPARE_REGISTER_2     0x06
#define TIMER_B_CAPTURECOMPARE_REGISTER_3     0x08
#define TIMER_B_CAPTURECOMPARE_REGISTER_4     0x0A
#define TIMER_B_CAPTURECOMPARE_REGISTER_5     0x0C
#define TIMER_B_CAPTURECOMPARE_REGISTER_6     0x0E

//*****************************************************************************
//
//The following are values can be passed to the
//captureMode parameter of TIMER_B_initCompare();, TIMER_B_initCapture();,
//TIMER_B_generatePWM();,
//
//*****************************************************************************
#define TIMER_B_OUTPUTMODE_OUTBITVALUE        OUTMOD_0
#define TIMER_B_OUTPUTMODE_SET                OUTMOD_1
#define TIMER_B_OUTPUTMODE_TOGGLE_RESET       OUTMOD_2
#define TIMER_B_OUTPUTMODE_SET_RESET          OUTMOD_3
#define TIMER_B_OUTPUTMODE_TOGGLE             OUTMOD_4
#define TIMER_B_OUTPUTMODE_RESET              OUTMOD_5
#define TIMER_B_OUTPUTMODE_TOGGLE_SET         OUTMOD_6
#define TIMER_B_OUTPUTMODE_RESET_SET          OUTMOD_7

//*****************************************************************************
//
//The following are values can be passed to the
//captureMode parameter of TIMER_B_initCapture(); API
//
//*****************************************************************************
#define TIMER_B_CAPTUREMODE_NO_CAPTURE                CM_0
#define TIMER_B_CAPTUREMODE_RISING_EDGE               CM_1
#define TIMER_B_CAPTUREMODE_FALLING_EDGE              CM_2
#define TIMER_B_CAPTUREMODE_RISING_AND_FALLING_EDGE   CM_3

//*****************************************************************************
//
//The following are values can be passed to the
//synchronizeCaptureSource parameter of TIMER_B_initCapture(); API
//
//*****************************************************************************
#define TIMER_B_CAPTURE_ASYNCHRONOUS                  0x00
#define TIMER_B_CAPTURE_SYNCHRONOUS                   SCS

//*****************************************************************************
//
//The following are values can be passed to the
//captureInterruptEnable, compareInterruptEnable parameter of
//TIMER_B_initCapture(); API
//
//*****************************************************************************
#define TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE       CCIE
#define TIMER_B_CAPTURECOMPARE_INTERRUPT_DISABLE      0x00

//*****************************************************************************
//
//The following are values can be passed to the
//captureInputSelect parameter of TIMER_B_initCapture(); API
//
//*****************************************************************************
#define TIMER_B_CAPTURE_INPUTSELECT_CCIxA             CCIS_0
#define TIMER_B_CAPTURE_INPUTSELECT_CCIxB             CCIS_1
#define TIMER_B_CAPTURE_INPUTSELECT_GND               CCIS_2
#define TIMER_B_CAPTURE_INPUTSELECT_Vcc               CCIS_3

//*****************************************************************************
//
//The following are values can be passed to the
//counterLength parameter of TIMER_B_selectCounterLength(); API
//
//*****************************************************************************
#define TIMER_B_COUNTER_8BIT CNTL_0
#define TIMER_B_COUNTER_10BIT CNTL_1
#define TIMER_B_COUNTER_12BIT CNTL_2
#define TIMER_B_COUNTER_16BIT CNTL_3

//*****************************************************************************
//
//The following are values can be passed to the
//groupLatch parameter of TIMER_B_selectLatchingGroup(); API
//
//*****************************************************************************
#define TIMER_B_GROUP_NONE				TBCLGRP_0
#define TIMER_B_GROUP_CL12_CL23_CL56	TBCLGRP_1
#define TIMER_B_GROUP_CL123_CL456		TBCLGRP_2
#define TIMER_B_GROUP_ALL				TBCLGRP_3


//*****************************************************************************
//
//The following are values can be passed to the
//compareLatchLoadEvent parameter of TIMER_B_initCompareLatchLoadEvent(); API
//
//*****************************************************************************
#define TIMER_B_LATCH_ON_WRITE_TO_TBxCCRn_COMPARE_REGISTER    		CLLD_0
#define TIMER_B_LATCH_WHEN_COUNTER_COUNTS_TO_0_IN_UP_OR_CONT_MODE	CLLD_1
#define TIMER_B_LATCH_WHEN_COUNTER_COUNTS_TO_0_IN_UPDOWN_MODE 		CLLD_2
#define TIMER_B_LATCH_WHEN_COUNTER_COUNTS_TO_CURRENT_COMPARE_LATCH_VALUE  CLLD_3


//*****************************************************************************
//
//The following are values that may be returned by
//TIMER_B_getInterruptStatus(); API
//
//*****************************************************************************
#define TIMER_B_INTERRUPT_NOT_PENDING     0x00
#define TIMER_B_INTERRUPT_PENDING         0x01

//*****************************************************************************
//
//The following are values can be passed to the
//synchronized parameter of TIMER_B_getSynchronizedCaptureCompareInput(); API
//
//*****************************************************************************
#define TIMER_B_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT SCCI
#define TIMER_B_READ_CAPTURE_COMPARE_INPUT            CCI

//*****************************************************************************
//
//Prototypes for the APIs.
//
//*****************************************************************************
extern void TIMER_B_startCounter ( unsigned int baseAddress,
    unsigned int timerMode
    );
extern void TIMER_B_configureContinuousMode ( unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerInterruptEnable_TBIE,
    unsigned int timerClear
    );
extern void TIMER_B_configureUpMode (   unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerPeriod,
    unsigned int timerInterruptEnable_TBIE,
    unsigned int captureCompareInterruptEnable_CCR0_CCIE,
    unsigned int timerClear
    );
extern void TIMER_B_configureUpDownMode (
    unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerPeriod,
    unsigned int timerInterruptEnable_TBIE,
    unsigned int captureCompareInterruptEnable_CCR0_CCIE,
    unsigned int timerClear
    );
extern void TIMER_B_startContinuousMode ( unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerInterruptEnable_TBIE,
    unsigned int timerClear
    );
extern void TIMER_B_startContinousMode ( unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerInterruptEnable_TBIE,
    unsigned int timerClear
    );
extern void TIMER_B_startUpMode (   unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerPeriod,
    unsigned int timerInterruptEnable_TBIE,
    unsigned int captureCompareInterruptEnable_CCR0_CCIE,
    unsigned int timerClear
    );
extern void TIMER_B_startUpDownMode (
    unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerPeriod,
    unsigned int timerInterruptEnable_TBIE,
    unsigned int captureCompareInterruptEnable_CCR0_CCIE,
    unsigned int timerClear
    );
extern void TIMER_B_initCapture (unsigned int baseAddress,
    unsigned int captureRegister,
    unsigned int captureMode,
    unsigned int captureInputSelect,
    unsigned short synchronizeCaptureSource,
    unsigned short captureInterruptEnable,
    unsigned int captureOutputMode
    );
extern void TIMER_B_initCompare (  unsigned int baseAddress,
    unsigned int compareRegister,
    unsigned short compareInterruptEnable,
    unsigned int compareOutputMode,
    unsigned int compareValue
    );
extern void TIMER_B_enableInterrupt (unsigned int baseAddress);
extern void TIMER_B_disableInterrupt (unsigned int baseAddress);
extern unsigned long TIMER_B_getInterruptStatus (unsigned int baseAddress);
extern void TIMER_B_enableCaptureCompareInterrupt (unsigned int baseAddress,
    unsigned int captureCompareRegister
    );
extern void TIMER_B_disableCaptureCompareInterrupt (unsigned int baseAddress,
    unsigned int captureCompareRegister
    );
extern unsigned long TIMER_B_getCaptureCompareInterruptStatus (unsigned int baseAddress,
		 unsigned int captureCompareRegister,
		 unsigned int mask
		 );
extern void TIMER_B_clear (unsigned int baseAddress);
unsigned short TIMER_B_getSynchronizedCaptureCompareInput
    (unsigned int baseAddress,
    unsigned int captureCompareRegister,
    unsigned short synchronized
    );
extern unsigned char TIMER_B_getOutputForOutputModeOutBitValue
    (unsigned int baseAddress,
    unsigned int captureCompareRegister
    );
extern unsigned int TIMER_B_getCaptureCompareCount
    (unsigned int baseAddress,
    unsigned int captureCompareRegister
    );
extern void TIMER_B_setOutputForOutputModeOutBitValue
    (unsigned int baseAddress,
    unsigned int captureCompareRegister,
    unsigned char outputModeOutBitValue
    );
extern void TIMER_B_generatePWM (  unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerPeriod,
    unsigned int compareRegister,
    unsigned int compareOutputMode,
    unsigned int dutyCycle
    );
extern void TIMER_B_stop ( unsigned int baseAddress );
extern void privateTimerBProcessClockSourceDivider (unsigned int baseAddress,
    unsigned int clockSourceDivider);
extern void TIMER_B_setCompareValue (  unsigned int baseAddress,
    unsigned int compareRegister,
    unsigned int compareValue
    );
extern void TIMER_B_clearTimerInterruptFlag (unsigned int baseAddress);
extern void TIMER_B_clearCaptureCompareInterruptFlag (unsigned int baseAddress,
    unsigned int captureCompareRegister
    );
extern void TIMER_B_selectCounterLength (unsigned int  baseAddress,
		unsigned int counterLength
		);
extern void TIMER_B_selectLatchingGroup(unsigned int  baseAddress,
		unsigned int  groupLatch);
extern void TIMER_B_initCompareLatchLoadEvent(unsigned int  baseAddress,
		unsigned int  compareRegister,
		unsigned int  compareLatchLoadEvent
		);


#endif
