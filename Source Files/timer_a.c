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
//*****************************************************************************
//
//timer_a.c - Driver for the TIMERA Module.
//
//*****************************************************************************
#include "inc/hw_types.h"
#include "debug.h"
#include "timer_a.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "deprecated/CCS/msp430xgeneric.h"
#endif

void privateTimerAProcessClockSourceDivider (unsigned int baseAddress,
    unsigned int clockSourceDivider);

/******************* ISR ROUTINE ****************************/

#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void)
{
	TIMER_A_clearTimerInterruptFlag(__MSP430_BASEADDRESS_T0A3__);
}



//*****************************************************************************
//
//! Starts TimerA counter
//!
//! \param baseAddress is the base address of the TimerA module.
//! \param clockSource selects Clock source. Valid values are
//!         \b TIMER_A_CONTINUOUS_MODE [Default value]
//!         \b TIMER_A_UPDOWN_MODE
//!         \b TIMER_A_UP_MODE

//! Modified register is \b TAxCTL
//!
//!NOTE: This function assumes that the timer has been previously configured
//! using TIMER_A_configureContinuousMode,  TIMER_A_configureUpMode or
//!TIMER_A_configureUpDownMode.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_startCounter ( unsigned int baseAddress,
    unsigned int timerMode
    )
{
    ASSERT(
        (TIMER_A_UPDOWN_MODE == timerMode) ||
        (TIMER_A_CONTINUOUS_MODE == timerMode) ||
        (TIMER_A_UP_MODE == timerMode)
         );


    HWREG(baseAddress + OFS_TAxCTL) |= timerMode;
}

//*****************************************************************************
//
//! Configures TimerA in continuous mode.
//!
//! \param baseAddress is the base address of the TimerA module.
//! \param clockSource selects Clock source. Valid values are
//!         \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default value]
//!         \b TIMER_A_CLOCKSOURCE_ACLK
//!         \b TIMER_A_CLOCKSOURCE_SMCLK
//!         \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the divider for Clock source. Valid values are
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default value]
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerInterruptEnable_TAIE is to enable or disable TimerA interrupt
//!        Valid values are
//!        \b TIMER_A_TAIE_INTERRUPT_ENABLE
//!        \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default value]
//! \param timerClear decides if TimerA clock divider, count direction, count
//!        need to be reset. Valid values are
//!        \b TIMER_A_DO_CLEAR
//!        \b TIMER_A_SKIP_CLEAR [Default value]
//!
//! Modified reister is \b TAxCTL
//!
//!This API does not start the timer. Timer needs to be started when required
//!using the TIMER_A_startCounter API.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_configureContinuousMode ( unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerInterruptEnable_TAIE,
    unsigned int timerClear
    )
{
    ASSERT(
        (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_ACLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_SMCLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == clockSource)
        );

    ASSERT(
        (TIMER_A_DO_CLEAR == timerClear) ||
        (TIMER_A_SKIP_CLEAR == timerClear)
        );

    ASSERT(
        (TIMER_A_TAIE_INTERRUPT_ENABLE == timerInterruptEnable_TAIE) ||
        (TIMER_A_TAIE_INTERRUPT_DISABLE == timerInterruptEnable_TAIE)
        );

    ASSERT(
        (TIMER_A_CLOCKSOURCE_DIVIDER_1 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_2 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_4 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_8 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_3 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_5 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_6 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_7 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_10 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_12 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_14 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_16 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_20 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_24 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_28 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_32 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_40 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_48 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_56 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_64 == clockSourceDivider)
        );


    privateTimerAProcessClockSourceDivider(baseAddress,
        clockSourceDivider
        );

    HWREG(baseAddress +
        OFS_TAxCTL) &= ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK +
        				 TIMER_A_STOP_MODE +
                         TIMER_A_DO_CLEAR +
                         TIMER_A_TAIE_INTERRUPT_ENABLE
                         );

    HWREG(baseAddress + OFS_TAxCTL)  |= ( clockSource + TIMER_A_CONTINUOUS_MODE +
                                          timerClear +
                                          timerInterruptEnable_TAIE);
}
//*****************************************************************************
//
//! Configures TimerA in up mode.
//!
//! \param baseAddress is the base address of the TimerA module.
//! \param clockSource selects Clock source. Valid values are
//!         \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default value]
//!         \b TIMER_A_CLOCKSOURCE_ACLK
//!         \b TIMER_A_CLOCKSOURCE_SMCLK
//!         \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the divider for Clock source. Valid values are
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default value]
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerPeriod is the specified TimerA period. This is the value that gets
//!         written into the CCR0. Limited to 16 bits[unsigned int]
//! \param timerInterruptEnable_TAIE is to enable or disable TimerA interrupt
//!        Valid values are
//!        \b TIMER_A_TAIE_INTERRUPT_ENABLE and
//!        \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default value]
//! \param captureCompareInterruptEnable_CCR0_CCIE is to enable or disable
//!         TimerA CCR0 captureComapre interrupt. Valid values are
//!        \b TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE and
//!        \b TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE [Default value]
//! \param timerClear decides if TimerA clock divider, count direction, count
//!        need to be reset. Valid values are
//!        \b TIMER_A_DO_CLEAR
//!        \b TIMER_A_SKIP_CLEAR [Default value]
//!
//! Modified registers are  \b TAxCTL, \b TAxCCR0, \b TAxCCTL0
//!
//!This API does not start the timer. Timer needs to be started when required
//!using the TIMER_A_startCounter API.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_configureUpMode (   unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerPeriod,
    unsigned int timerInterruptEnable_TAIE,
    unsigned int captureCompareInterruptEnable_CCR0_CCIE,
    unsigned int timerClear
    )
{
    ASSERT(
        (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_ACLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_SMCLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == clockSource)
        );

    ASSERT(
        (TIMER_A_DO_CLEAR == timerClear) ||
        (TIMER_A_SKIP_CLEAR == timerClear)
        );

    ASSERT(
        (TIMER_A_DO_CLEAR == timerClear) ||
        (TIMER_A_SKIP_CLEAR == timerClear)
        );

    privateTimerAProcessClockSourceDivider(baseAddress,
        clockSourceDivider
        );

    HWREG(baseAddress + OFS_TAxCTL) &=
        ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK +
          TIMER_A_STOP_MODE +
          TIMER_A_DO_CLEAR +
          TIMER_A_TAIE_INTERRUPT_ENABLE
          );

    HWREG(baseAddress + OFS_TAxCTL)  |= ( clockSource +
                                          TIMER_A_UP_MODE +
                                          timerClear +
                                          timerInterruptEnable_TAIE
                                          );

    if (TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ==
        captureCompareInterruptEnable_CCR0_CCIE){
        HWREG(baseAddress + OFS_TAxCCTL0)  |= TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    } else   {
        HWREG(baseAddress + OFS_TAxCCTL0)  &= ~TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    }

    HWREG(baseAddress + OFS_TAxCCR0)  = timerPeriod;
}

//*****************************************************************************
//
//! Configures TimerA in up down mode.
//!
//! \param baseAddress is the base address of the TimerA module.
//! \param clockSource selects Clock source. Valid values are
//!         \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default value]
//!         \b TIMER_A_CLOCKSOURCE_ACLK
//!         \b TIMER_A_CLOCKSOURCE_SMCLK
//!         \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the divider for Clock source. Valid values are
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default value]
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerPeriod is the specified TimerA period
//! \param timerInterruptEnable_TAIE is to enable or disable TimerA interrupt
//!        Valid values are
//!        \b TIMER_A_TAIE_INTERRUPT_ENABLE
//!        \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default value]
//! \param captureCompareInterruptEnable_CCR0_CCIE is to enable or disable
//!         TimerA CCR0 captureComapre interrupt. Valid values are
//!        \b TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE and
//!        \b TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE [Default value]
//! \param timerClear decides if TimerA clock divider, count direction, count
//!        need to be reset. Valid values are
//!        \b TIMER_A_DO_CLEAR
//!        \b TIMER_A_SKIP_CLEAR [Default value]
//!
//! Modified registers are  \b TAxCTL, \b TAxCCR0, \b TAxCCTL0
//!
//!This API does not start the timer. Timer needs to be started when required
//!using the TIMER_A_startCounter API.
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_configureUpDownMode (
    unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerPeriod,
    unsigned int timerInterruptEnable_TAIE,
    unsigned int captureCompareInterruptEnable_CCR0_CCIE,
    unsigned int timerClear
    )
{
    ASSERT(
        (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_ACLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_SMCLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == clockSource)
        );

    ASSERT(
        (TIMER_A_DO_CLEAR == timerClear) ||
        (TIMER_A_SKIP_CLEAR == timerClear)
        );

    ASSERT(
        (TIMER_A_DO_CLEAR == timerClear) ||
        (TIMER_A_SKIP_CLEAR == timerClear)
        );

    privateTimerAProcessClockSourceDivider(baseAddress,
        clockSourceDivider
        );

    HWREG(baseAddress + OFS_TAxCTL) &=
        ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK +
          TIMER_A_UPDOWN_MODE +
          TIMER_A_DO_CLEAR +
          TIMER_A_TAIE_INTERRUPT_ENABLE
          );

    HWREG(baseAddress + OFS_TAxCTL)  |= ( clockSource +
    									  TIMER_A_STOP_MODE +
                                          timerClear +
                                          timerInterruptEnable_TAIE
                                          );
    if (TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ==
        captureCompareInterruptEnable_CCR0_CCIE){
        HWREG(baseAddress + OFS_TAxCCTL0)  |= TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    } else   {
        HWREG(baseAddress + OFS_TAxCCTL0)  &= ~TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    }

    HWREG(baseAddress + OFS_TAxCCR0)  = timerPeriod;
}


//*****************************************************************************
//
//! Starts timer in continuous mode.
//!
//! DEPRPECATED - Replaced by TIMER_A_configureContinuousMode and TIMER_A_startCounter
//! API
//!
//! \param baseAddress is the base address of the Timer module.
//! \param clockSource selects Clock source. Valid values are
//!         \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default value]
//!         \b TIMER_A_CLOCKSOURCE_ACLK
//!         \b TIMER_A_CLOCKSOURCE_SMCLK
//!         \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the divider for Clock source. Valid values are
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default value]
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerInterruptEnable_TAIE is to enable or disable timer interrupt
//!        Valid values are
//!        \b TIMER_A_TAIE_INTERRUPT_ENABLE
//!        \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default value]
//! \param timerClear decides if timer clock divider, count direction, count
//!        need to be reset. Valid values are
//!        \b TIMER_A_DO_CLEAR
//!        \b TIMER_A_SKIP_CLEAR [Default value]
//!
//! Modified reister is \b TAxCTL
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_startContinuousMode ( unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerInterruptEnable_TAIE,
    unsigned int timerClear
    )
{
    ASSERT(
        (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_ACLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_SMCLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == clockSource)
        );

    ASSERT(
        (TIMER_A_DO_CLEAR == timerClear) ||
        (TIMER_A_SKIP_CLEAR == timerClear)
        );

    ASSERT(
        (TIMER_A_TAIE_INTERRUPT_ENABLE == timerInterruptEnable_TAIE) ||
        (TIMER_A_TAIE_INTERRUPT_DISABLE == timerInterruptEnable_TAIE)
        );

    ASSERT(
        (TIMER_A_CLOCKSOURCE_DIVIDER_1 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_2 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_4 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_8 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_3 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_5 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_6 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_7 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_10 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_12 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_14 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_16 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_20 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_24 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_28 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_32 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_40 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_48 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_56 == clockSourceDivider) ||
        (TIMER_A_CLOCKSOURCE_DIVIDER_64 == clockSourceDivider)
        );


    privateTimerAProcessClockSourceDivider(baseAddress,
        clockSourceDivider
        );

    HWREG(baseAddress +
        OFS_TAxCTL) &= ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK +
                         TIMER_A_UPDOWN_MODE +
                         TIMER_A_DO_CLEAR +
                         TIMER_A_TAIE_INTERRUPT_ENABLE
                         );

    HWREG(baseAddress + OFS_TAxCTL)  |= ( clockSource + TIMER_A_CONTINUOUS_MODE +
                                          timerClear +
                                          timerInterruptEnable_TAIE);
}

//*****************************************************************************
//
//! DEPRECATED- Spelling Error Fixed
//! Starts timer in continuous mode.
//!
//! \param baseAddress is the base address of the Timer module.
//! \param clockSource selects Clock source. Valid values are
//!         \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default value]
//!         \b TIMER_A_CLOCKSOURCE_ACLK
//!         \b TIMER_A_CLOCKSOURCE_SMCLK
//!         \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the divider for Clock source. Valid values are
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default value]
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerInterruptEnable_TAIE is to enable or disable timer interrupt
//!        Valid values are
//!        \b TIMER_A_TAIE_INTERRUPT_ENABLE
//!        \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default value]
//! \param timerClear decides if timer clock divider, count direction, count
//!        need to be reset. Valid values are
//!        \b TIMER_A_DO_CLEAR
//!        \b TIMER_A_SKIP_CLEAR [Default value]
//!
//! Modified reister is \b TAxCTL
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_startContinousMode ( unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerInterruptEnable_TAIE,
    unsigned int timerClear
    )
{
	TIMER_A_startContinuousMode (baseAddress,
    clockSource,
    clockSourceDivider,
    timerInterruptEnable_TAIE,
    timerClear
    );
}
//*****************************************************************************
//
//! Starts timer in up mode.
//!
//! DEPRPECATED - Replaced by TIMER_A_configureUpMode and TIMER_A_startCounter
//! API
//!
//! \param baseAddress is the base address of the Timer module.
//! \param clockSource selects Clock source. Valid values are
//!         \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default value]
//!         \b TIMER_A_CLOCKSOURCE_ACLK
//!         \b TIMER_A_CLOCKSOURCE_SMCLK
//!         \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the divider for Clock source. Valid values are
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default value]
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerPeriod is the specified timer period. This is the value that gets
//!         written into the CCR0. Limited to 16 bits[unsigned int]
//! \param timerInterruptEnable_TAIE is to enable or disable timer interrupt
//!        Valid values are
//!        \b TIMER_A_TAIE_INTERRUPT_ENABLE and
//!        \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default value]
//! \param captureCompareInterruptEnable_CCR0_CCIE is to enable or disable
//!         timer CCR0 captureComapre interrupt. Valid values are
//!        \b TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE and
//!        \b TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE [Default value]
//! \param timerClear decides if timer clock divider, count direction, count
//!        need to be reset. Valid values are
//!        \b TIMER_A_DO_CLEAR
//!        \b TIMER_A_SKIP_CLEAR [Default value]
//!
//! Modified registers are  \b TAxCTL, \b TAxCCR0, \b TAxCCTL0
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_startUpMode (   unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerPeriod,
    unsigned int timerInterruptEnable_TAIE,
    unsigned int captureCompareInterruptEnable_CCR0_CCIE,
    unsigned int timerClear
    )
{
    ASSERT(
        (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_ACLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_SMCLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == clockSource)
        );

    ASSERT(
        (TIMER_A_DO_CLEAR == timerClear) ||
        (TIMER_A_SKIP_CLEAR == timerClear)
        );

    ASSERT(
        (TIMER_A_DO_CLEAR == timerClear) ||
        (TIMER_A_SKIP_CLEAR == timerClear)
        );

    privateTimerAProcessClockSourceDivider(baseAddress,
        clockSourceDivider
        );

    HWREG(baseAddress + OFS_TAxCTL) &=
        ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK +
          TIMER_A_UPDOWN_MODE +
          TIMER_A_DO_CLEAR +
          TIMER_A_TAIE_INTERRUPT_ENABLE
          );

    HWREG(baseAddress + OFS_TAxCTL)  |= ( clockSource +
                                          TIMER_A_UP_MODE +
                                          timerClear +
                                          timerInterruptEnable_TAIE
                                          );

    if (TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ==
        captureCompareInterruptEnable_CCR0_CCIE){
        HWREG(baseAddress + OFS_TAxCCTL0)  |= TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    } else   {
        HWREG(baseAddress + OFS_TAxCCTL0)  &= ~TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    }

    HWREG(baseAddress + OFS_TAxCCR0)  = timerPeriod;
}

//*****************************************************************************
//
//! Starts timer in up down mode.
//!
//! DEPRPECATED - Replaced by TIMER_A_configureUpDownMode and TIMER_A_startCounter
//! API
//!
//! \param baseAddress is the base address of the Timer module.
//! \param clockSource selects Clock source. Valid values are
//!         \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK [Default value]
//!         \b TIMER_A_CLOCKSOURCE_ACLK
//!         \b TIMER_A_CLOCKSOURCE_SMCLK
//!         \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the divider for Clock source. Valid values are
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_1 [Default value]
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerPeriod is the specified timer period
//! \param timerInterruptEnable_TAIE is to enable or disable timer interrupt
//!        Valid values are
//!        \b TIMER_A_TAIE_INTERRUPT_ENABLE
//!        \b TIMER_A_TAIE_INTERRUPT_DISABLE [Default value]
//! \param captureCompareInterruptEnable_CCR0_CCIE is to enable or disable
//!         timer CCR0 captureComapre interrupt. Valid values are
//!        \b TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE and
//!        \b TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE [Default value]
//! \param timerClear decides if timer clock divider, count direction, count
//!        need to be reset. Valid values are
//!        \b TIMER_A_DO_CLEAR
//!        \b TIMER_A_SKIP_CLEAR [Default value]
//!
//! Modified registers are  \b TAxCTL, \b TAxCCR0, \b TAxCCTL0
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_startUpDownMode (
    unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerPeriod,
    unsigned int timerInterruptEnable_TAIE,
    unsigned int captureCompareInterruptEnable_CCR0_CCIE,
    unsigned int timerClear
    )
{
    ASSERT(
        (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_ACLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_SMCLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == clockSource)
        );

    ASSERT(
        (TIMER_A_DO_CLEAR == timerClear) ||
        (TIMER_A_SKIP_CLEAR == timerClear)
        );

    ASSERT(
        (TIMER_A_DO_CLEAR == timerClear) ||
        (TIMER_A_SKIP_CLEAR == timerClear)
        );

    privateTimerAProcessClockSourceDivider(baseAddress,
        clockSourceDivider
        );

    HWREG(baseAddress + OFS_TAxCTL) &=
        ~(TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK +
          TIMER_A_UPDOWN_MODE +
          TIMER_A_DO_CLEAR +
          TIMER_A_TAIE_INTERRUPT_ENABLE
          );

    HWREG(baseAddress + OFS_TAxCTL)  |= ( clockSource +
                                          TIMER_A_UPDOWN_MODE +
                                          timerClear +
                                          timerInterruptEnable_TAIE
                                          );
    if (TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ==
        captureCompareInterruptEnable_CCR0_CCIE){
        HWREG(baseAddress + OFS_TAxCCTL0)  |= TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    } else   {
        HWREG(baseAddress + OFS_TAxCCTL0)  &= ~TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
    }

    HWREG(baseAddress + OFS_TAxCCR0)  = timerPeriod;
}

//*****************************************************************************
//
//! Initializes Capture Mode
//!
//! \param baseAddress is the base address of the Timer module.
//! \param captureRegister selects the Capture register being used. Valid values
//!     are
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    Refer datasheet to ensure the device has the capture compare register
//!    being used
//! \param captureMode is the capture mode selected. Valid values are
//!        \b TIMER_A_CAPTUREMODE_NO_CAPTURE [Default value]
//!        \b TIMER_A_CAPTUREMODE_RISING_EDGE
//!        \b TIMER_A_CAPTUREMODE_FALLING_EDGE
//!        \b TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE
//! \param captureInputSelect decides the Input Select
//!        \b TIMER_A_CAPTURE_INPUTSELECT_CCIxA [Default value]
//!        \b TIMER_A_CAPTURE_INPUTSELECT_CCIxB
//!        \b TIMER_A_CAPTURE_INPUTSELECT_GND
//!        \b TIMER_A_CAPTURE_INPUTSELECT_Vcc
//! \param synchronizeCaptureSource decides if capture source should be
//!         synchronized with timer clock
//!        Valid values are
//!        \b TIMER_A_CAPTURE_ASYNCHRONOUS [Default value]
//!        \b TIMER_A_CAPTURE_SYNCHRONOUS
//! \param captureInterruptEnable is to enable or disable
//!         timer captureComapre interrupt. Valid values are
//!        \b TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE [Default value]
//!        \b TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE
//! \param captureOutputMode specifies the ouput mode. Valid values are
//!        \b TIMER_A_OUTPUTMODE_OUTBITVALUE [Default value],
//!        \b TIMER_A_OUTPUTMODE_SET,
//!        \b TIMER_A_OUTPUTMODE_TOGGLE_RESET,
//!        \b TIMER_A_OUTPUTMODE_SET_RESET
//!        \b TIMER_A_OUTPUTMODE_TOGGLE,
//!        \b TIMER_A_OUTPUTMODE_RESET,
//!        \b TIMER_A_OUTPUTMODE_TOGGLE_SET,
//!        \b TIMER_A_OUTPUTMODE_RESET_SET
//!
//! Modified register is \b TAxCCTLn
//! \return None
//
//*****************************************************************************
void TIMER_A_initCapture (unsigned int baseAddress,
    unsigned int captureRegister,
    unsigned int captureMode,
    unsigned int captureInputSelect,
    unsigned short synchronizeCaptureSource,
    unsigned short captureInterruptEnable,
    unsigned int captureOutputMode
    )
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureRegister)
        );

    ASSERT((TIMER_A_CAPTUREMODE_NO_CAPTURE == captureMode) ||
        (TIMER_A_CAPTUREMODE_RISING_EDGE == captureMode) ||
        (TIMER_A_CAPTUREMODE_FALLING_EDGE == captureMode) ||
        (TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE == captureMode)
        );

    ASSERT((TIMER_A_CAPTURE_INPUTSELECT_CCIxA == captureInputSelect) ||
        (TIMER_A_CAPTURE_INPUTSELECT_CCIxB == captureInputSelect) ||
        (TIMER_A_CAPTURE_INPUTSELECT_GND == captureInputSelect) ||
        (TIMER_A_CAPTURE_INPUTSELECT_Vcc == captureInputSelect)
        );

    ASSERT((TIMER_A_CAPTURE_ASYNCHRONOUS == synchronizeCaptureSource) ||
        (TIMER_A_CAPTURE_SYNCHRONOUS == synchronizeCaptureSource)
        );

    ASSERT(
        (TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE == captureInterruptEnable) ||
        (TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE == captureInterruptEnable)
        );

    ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == captureOutputMode) ||
        (TIMER_A_OUTPUTMODE_SET == captureOutputMode) ||
        (TIMER_A_OUTPUTMODE_TOGGLE_RESET == captureOutputMode) ||
        (TIMER_A_OUTPUTMODE_SET_RESET == captureOutputMode) ||
        (TIMER_A_OUTPUTMODE_TOGGLE == captureOutputMode) ||
        (TIMER_A_OUTPUTMODE_RESET == captureOutputMode) ||
        (TIMER_A_OUTPUTMODE_TOGGLE_SET == captureOutputMode) ||
        (TIMER_A_OUTPUTMODE_RESET_SET == captureOutputMode)
        );

    if (TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureRegister){
        //CaptureCompare register 0 only supports certain modes
        ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == captureOutputMode) ||
            (TIMER_A_OUTPUTMODE_SET == captureOutputMode) ||
            (TIMER_A_OUTPUTMODE_TOGGLE == captureOutputMode) ||
            (TIMER_A_OUTPUTMODE_RESET == captureOutputMode)
            );
    }

    HWREG(baseAddress + captureRegister ) |=   CAP;

    HWREG(baseAddress + captureRegister) &=
        ~(TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE +
          TIMER_A_CAPTURE_INPUTSELECT_Vcc +
          TIMER_A_CAPTURE_SYNCHRONOUS +
          TIMER_A_DO_CLEAR +
          TIMER_A_TAIE_INTERRUPT_ENABLE +
          CM_3
          );

    HWREG(baseAddress + captureRegister)  |= (captureMode +
                                              captureInputSelect +
                                              synchronizeCaptureSource +
                                              captureInterruptEnable +
                                              captureOutputMode
                                              );
}

//*****************************************************************************
//
//! Initializes Compare Mode
//!
//! \param baseAddress is the base address of the Timer module.
//! \param compareRegister selects the Capture register being used. Valid values
//!     are
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    Refer datasheet to ensure the device has the capture compare register
//!    being used
//! \param compareInterruptEnable is to enable or disable
//!         timer captureComapre interrupt. Valid values are
//!        \b TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE and
//!        \b TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE [Default value]
//! \param compareOutputMode specifies the ouput mode. Valid values are
//!        \b TIMER_A_OUTPUTMODE_OUTBITVALUE [Default value],
//!        \b TIMER_A_OUTPUTMODE_SET,
//!        \b TIMER_A_OUTPUTMODE_TOGGLE_RESET,
//!        \b TIMER_A_OUTPUTMODE_SET_RESET
//!        \b TIMER_A_OUTPUTMODE_TOGGLE,
//!        \b TIMER_A_OUTPUTMODE_RESET,
//!        \b TIMER_A_OUTPUTMODE_TOGGLE_SET,
//!        \b TIMER_A_OUTPUTMODE_RESET_SET
//! \param compareValue is the count to be compared with in compare mode
//!
//! Modified register is \b TAxCCTLn and \b TAxCCRn
//! \return None
//
//*****************************************************************************
void TIMER_A_initCompare (  unsigned int baseAddress,
    unsigned int compareRegister,
    unsigned short compareInterruptEnable,
    unsigned int compareOutputMode,
    unsigned int compareValue
    )
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_1 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_2 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_3 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_4 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_5 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_6 == compareRegister)
        );

   ASSERT((TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE == compareInterruptEnable) ||
        (TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE == compareInterruptEnable)
        );

    ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == compareOutputMode) ||
        (TIMER_A_OUTPUTMODE_SET == compareOutputMode) ||
        (TIMER_A_OUTPUTMODE_TOGGLE_RESET == compareOutputMode) ||
        (TIMER_A_OUTPUTMODE_SET_RESET == compareOutputMode) ||
        (TIMER_A_OUTPUTMODE_TOGGLE == compareOutputMode) ||
        (TIMER_A_OUTPUTMODE_RESET == compareOutputMode) ||
        (TIMER_A_OUTPUTMODE_TOGGLE_SET == compareOutputMode) ||
        (TIMER_A_OUTPUTMODE_RESET_SET == compareOutputMode)
        );

    if (TIMER_A_CAPTURECOMPARE_REGISTER_0 == compareRegister){
        //CaptureCompare register 0 only supports certain modes
        ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == compareOutputMode) ||
            (TIMER_A_OUTPUTMODE_SET == compareOutputMode) ||
            (TIMER_A_OUTPUTMODE_TOGGLE == compareOutputMode) ||
            (TIMER_A_OUTPUTMODE_RESET == compareOutputMode)
            );
    }


    HWREG(baseAddress + compareRegister ) &=   ~CAP;

    HWREG(baseAddress + compareRegister) &=
        ~(TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE +
          TIMER_A_OUTPUTMODE_RESET_SET
          );

    HWREG(baseAddress + compareRegister)  |= ( compareInterruptEnable +
                                               compareOutputMode
                                               );

    HWREG(baseAddress + compareRegister + OFS_TAxR) = compareValue;
}

//*****************************************************************************
//
//! Enable timer interrupt
//!
//! \param baseAddress is the base address of the Timer module.
//!
//! Modified register is TAxCTL
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_enableInterrupt (unsigned int baseAddress)
{
    HWREGB(baseAddress + OFS_TAxCTL) &=  ~TAIFG;
    HWREG(baseAddress + OFS_TAxCTL) |= TAIE;
}

//*****************************************************************************
//
//! Disable timer interrupt
//!
//! \param baseAddress is the base address of the Timer module.
//!
//! Modified register is \b TAxCTL
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_disableInterrupt (unsigned int baseAddress)
{
    HWREG(baseAddress + OFS_TAxCTL) &= ~TAIE;
}

//*****************************************************************************
//
//! Get timer interrupt status
//!
//! \param baseAddress is the base address of the Timer module.
//!
//! \return unsigned long. Return interrupt status. Valid values are
//!         \b TIMER_A_INTERRUPT_PENDING
//!         \b TIMER_A_INTERRUPT_NOT_PENDING
//
//*****************************************************************************
unsigned long TIMER_A_getInterruptStatus (unsigned int baseAddress)
{
    return ( HWREG(baseAddress + OFS_TAxCTL) & TAIFG );
}

//*****************************************************************************
//
//! Enable capture compare interrupt
//!
//! \param baseAddress is the base address of the Timer module.
//! \param captureCompareRegister is the selected capture compare regsiter
//!
//! Modified register is \b TAxCCTLn
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_enableCaptureCompareInterrupt (unsigned int baseAddress,
    unsigned int captureCompareRegister
    )
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureCompareRegister)
        );

    HWREGB(baseAddress + captureCompareRegister) &= ~CCIFG;
    HWREG(baseAddress + captureCompareRegister) |= CCIE;
}

//*****************************************************************************
//
//! Disable capture compare interrupt
//!
//! \param baseAddress is the base address of the Timer module.
//! \param captureCompareRegister is the selected capture compare regsiter
//!
//! Modified register is \b TAxCCTLn
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_disableCaptureCompareInterrupt (unsigned int baseAddress,
    unsigned int captureCompareRegister
    )
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureCompareRegister)
        );
    HWREG(baseAddress + captureCompareRegister) &= ~CCIE;
}

//*****************************************************************************
//
//! Return capture compare interrupt status
//!
//! \param baseAddress is the base address of the Timer module.
//! \param captureCompareRegister is the selected capture compare register
//! \param mask is the mask for the interrupt status
//!         Valid values is and OR of
//!         \b TIMER_A_CAPTURE_OVERFLOW,
//!         \b TIMER_A_CAPTURECOMPARE_INTERRUPT_FLAG
//!
//!
//! \returns unsigned long. The mask of the set flags.
//
//*****************************************************************************
unsigned long TIMER_A_getCaptureCompareInterruptStatus (unsigned int baseAddress,
		 unsigned int captureCompareRegister,
		 unsigned int mask
		 )
{
    return ( HWREG(baseAddress + captureCompareRegister) & mask );
}

//*****************************************************************************
//
//! Reset/Clear the timer clock divider, count direction, count
//!
//! \param baseAddress is the base address of the Timer module.
//!
//! Modified register is \b TAxCTL
//!
//! \returns None
//
//*****************************************************************************
void TIMER_A_clear (unsigned int baseAddress)
{
    HWREG(baseAddress + OFS_TAxCTL) |= TACLR;
}

//*****************************************************************************
//
//! Get synchrnozied capturecompare input
//!
//! \param baseAddress is the base address of the Timer module.
//! \param captureRegister selects the Capture register being used. Valid values
//!     are
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    Refer datasheet to ensure the device has the capture compare register
//!    being used
//! \param synchronized is to select type of capture compare input.
//!         Valid values are
//!        \b TIMER_A_READ_CAPTURE_COMPARE_INPUT
//!        \b TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT
//!
//! \return \b TIMER_A_CAPTURECOMPARE_INPUT_HIGH or
//!         \b TIMER_A_CAPTURECOMPARE_INPUT_LOW
//
//*****************************************************************************
unsigned short TIMER_A_getSynchronizedCaptureCompareInput
    (unsigned int baseAddress,
    unsigned int captureCompareRegister,
    unsigned short synchronized
    )
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureCompareRegister)
        );

    ASSERT((TIMER_A_READ_CAPTURE_COMPARE_INPUT == synchronized) ||
        (TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT == synchronized)
        );

    if (HWREG(baseAddress + captureCompareRegister) & synchronized){
        return ( TIMER_A_CAPTURECOMPARE_INPUT_HIGH) ;
    } else   {
        return ( TIMER_A_CAPTURECOMPARE_INPUT_LOW) ;
    }
}

//*****************************************************************************
//
//! Get ouput bit for output mode
//!
//! \param baseAddress is the base address of the Timer module.
//! \param captureRegister selects the Capture register being used. Valid values
//!     are
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    Refer datasheet to ensure the device has the capture compare register
//!    being used
//!
//! \return \b TIMER_A_OUTPUTMODE_OUTBITVALUE_HIGH or
//!         \b TIMER_A_OUTPUTMODE_OUTBITVALUE_LOW
//
//*****************************************************************************
unsigned char TIMER_A_getOutputForOutputModeOutBitValue
    (unsigned int baseAddress,
    unsigned int captureCompareRegister
    )
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureCompareRegister)
        );

    if (HWREG(baseAddress + captureCompareRegister) & OUT){
        return ( TIMER_A_OUTPUTMODE_OUTBITVALUE_HIGH) ;
    } else   {
        return ( TIMER_A_OUTPUTMODE_OUTBITVALUE_LOW) ;
    }
}

//*****************************************************************************
//
//! Get current capturecompare count
//!
//! \param baseAddress is the base address of the Timer module.
//! \param captureRegister selects the Capture register being used. Valid values
//!     are
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    Refer datasheet to ensure the device has the capture compare register
//!    being used
//!
//! \return current count as unsigned int
//
//*****************************************************************************
unsigned int TIMER_A_getCaptureCompareCount
    (unsigned int baseAddress,
    unsigned int captureCompareRegister
    )
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureCompareRegister)
        );

    return  (HWREG(baseAddress + OFS_TAxR + captureCompareRegister));
}

//*****************************************************************************
//
//! Set ouput bit for output mode
//!
//! \param baseAddress is the base address of the Timer module.
//! \param captureCompareRegister selects the Capture register being used.
//!     are
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    Refer datasheet to ensure the device has the capture compare register
//!    being used
//! \param outputModeOutBitValueis the value to be set for out bit
//!     Valid values are \b TIMER_A_OUTPUTMODE_OUTBITVALUE_HIGH
//!                      \b TIMER_A_OUTPUTMODE_OUTBITVALUE_LOW
//!
//! Modified register is \b TAxCCTLn
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_setOutputForOutputModeOutBitValue
    (unsigned int baseAddress,
    unsigned int captureCompareRegister,
    unsigned char outputModeOutBitValue
    )
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureCompareRegister)
        );

    ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE_HIGH == outputModeOutBitValue) ||
        (TIMER_A_OUTPUTMODE_OUTBITVALUE_LOW == outputModeOutBitValue)
        );

    HWREG(baseAddress + captureCompareRegister) &= ~OUT;
    HWREG(baseAddress + captureCompareRegister) |= outputModeOutBitValue;
}

//*****************************************************************************
//
//! Generate a PWM with timer running in up mode
//!
//! \param baseAddress is the base address of the Timer module.
//! \param clockSource selects Clock source. Valid values are
//!         \b TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK
//!         \b TIMER_A_CLOCKSOURCE_ACLK
//!         \b TIMER_A_CLOCKSOURCE_SMCLK
//!         \b TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK
//! \param clockSourceDivider is the divider for Clock source. Valid values are
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_1
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_2
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_4
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_8
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_3
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_5
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_6
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_7
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_10
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_12
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_14
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_16
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_20
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_24
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_28
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_32
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_40
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_48
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_56
//!        \b TIMER_A_CLOCKSOURCE_DIVIDER_64
//! \param timerPeriod selects the desired timer period
//! \param compareRegister selects the compare register being used. Valid values
//!     are
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    Refer datasheet to ensure the device has the capture compare register
//!    being used
//! \param compareOutputMode specifies the ouput mode. Valid values are
//!        \b TIMER_A_OUTPUTMODE_OUTBITVALUE,
//!        \b TIMER_A_OUTPUTMODE_SET,
//!        \b TIMER_A_OUTPUTMODE_TOGGLE_RESET,
//!        \b TIMER_A_OUTPUTMODE_SET_RESET
//!        \b TIMER_A_OUTPUTMODE_TOGGLE,
//!        \b TIMER_A_OUTPUTMODE_RESET,
//!        \b TIMER_A_OUTPUTMODE_TOGGLE_SET,
//!        \b TIMER_A_OUTPUTMODE_RESET_SET
//! \param dutyCycle specifies the dutycycle for the generated waveform
//!
//! Modified registers are \b TAxCTL, \b TAxCCR0, \b TAxCCTL0,\b TAxCCTLn
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_generatePWM (  unsigned int baseAddress,
    unsigned int clockSource,
    unsigned int clockSourceDivider,
    unsigned int timerPeriod,
    unsigned int compareRegister,
    unsigned int compareOutputMode,
    unsigned int dutyCycle
    )
{
    ASSERT(
        (TIMER_A_CLOCKSOURCE_EXTERNAL_TXCLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_ACLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_SMCLK == clockSource) ||
        (TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK == clockSource)
        );

    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_1 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_2 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_3 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_4 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_5 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_6 == compareRegister)
        );


    ASSERT((TIMER_A_OUTPUTMODE_OUTBITVALUE == compareOutputMode) ||
        (TIMER_A_OUTPUTMODE_SET == compareOutputMode) ||
        (TIMER_A_OUTPUTMODE_TOGGLE_RESET == compareOutputMode) ||
        (TIMER_A_OUTPUTMODE_SET_RESET == compareOutputMode) ||
        (TIMER_A_OUTPUTMODE_TOGGLE == compareOutputMode) ||
        (TIMER_A_OUTPUTMODE_RESET == compareOutputMode) ||
        (TIMER_A_OUTPUTMODE_TOGGLE_SET == compareOutputMode) ||
        (TIMER_A_OUTPUTMODE_RESET_SET == compareOutputMode)
        );

    privateTimerAProcessClockSourceDivider(baseAddress,
        clockSourceDivider
        );

    HWREG(baseAddress + OFS_TAxCTL)  &=
        ~( TIMER_A_CLOCKSOURCE_INVERTED_EXTERNAL_TXCLK +
           TIMER_A_UPDOWN_MODE + TIMER_A_DO_CLEAR +
           TIMER_A_TAIE_INTERRUPT_ENABLE
           );

    HWREG(baseAddress + OFS_TAxCTL)  |= ( clockSource +
                                          TIMER_A_UP_MODE +
                                          TIMER_A_DO_CLEAR
                                          );

    HWREG(baseAddress + OFS_TAxCCR0)  = timerPeriod;

    HWREG(baseAddress + OFS_TAxCCTL0)  &=
        ~(TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE +
          TIMER_A_OUTPUTMODE_RESET_SET
          );
    HWREG(baseAddress + compareRegister)  |= compareOutputMode;

    HWREG(baseAddress + compareRegister + OFS_TAxR) = dutyCycle;
}

//*****************************************************************************
//
//! Stops the timer
//!
//! \param baseAddress is the base address of the Timer module.
//!
//! Modified registers are \b TAxCTL
//!
//! \returns None
//
//*****************************************************************************
void TIMER_A_stop ( unsigned int baseAddress )
{
    HWREG(baseAddress + OFS_TAxCTL)  &= ~MC_3;
    HWREG(baseAddress + OFS_TAxCTL)  |= MC_0;
}

//*****************************************************************************
//
//! Private clock source divider helper function
//!
//! \param baseAddress is the base address of the Timer module.
//! \param clockSourceDivider is the desired divider for the clock source
//!
//! Modified registers are TAxEX0, TAxCTL
//!
//! \returns None
//
//*****************************************************************************
void privateTimerAProcessClockSourceDivider (unsigned int baseAddress,
    unsigned int clockSourceDivider)
{
    HWREG(baseAddress + OFS_TAxCTL) &= ~ID__8;
    HWREG(baseAddress + OFS_TAxEX0) &= ~TAIDEX_7;
    switch (clockSourceDivider){
        case TIMER_A_CLOCKSOURCE_DIVIDER_1:
        case TIMER_A_CLOCKSOURCE_DIVIDER_2:
        case TIMER_A_CLOCKSOURCE_DIVIDER_4:
        case TIMER_A_CLOCKSOURCE_DIVIDER_8:
            HWREG(baseAddress + OFS_TAxCTL) |= ((clockSourceDivider - 1) << 6);
            HWREG(baseAddress + OFS_TAxEX0) = TAIDEX_0;
            break;

        case TIMER_A_CLOCKSOURCE_DIVIDER_3:
        case TIMER_A_CLOCKSOURCE_DIVIDER_5:
        case TIMER_A_CLOCKSOURCE_DIVIDER_6:
        case TIMER_A_CLOCKSOURCE_DIVIDER_7:
            HWREG(baseAddress + OFS_TAxCTL) |= ID__1;
            HWREG(baseAddress + OFS_TAxEX0) = (clockSourceDivider - 1);
            break;

        case TIMER_A_CLOCKSOURCE_DIVIDER_10:
        case TIMER_A_CLOCKSOURCE_DIVIDER_12:
        case TIMER_A_CLOCKSOURCE_DIVIDER_14:
        case TIMER_A_CLOCKSOURCE_DIVIDER_16:
            HWREG(baseAddress + OFS_TAxCTL) |= ID__2;
            HWREG(baseAddress + OFS_TAxEX0) = (clockSourceDivider / 2 - 1 );
            break;

        case TIMER_A_CLOCKSOURCE_DIVIDER_20:
        case TIMER_A_CLOCKSOURCE_DIVIDER_24:
        case TIMER_A_CLOCKSOURCE_DIVIDER_28:
        case TIMER_A_CLOCKSOURCE_DIVIDER_32:
            HWREG(baseAddress + OFS_TAxCTL) |= ID__4;
            HWREG(baseAddress + OFS_TAxEX0) = (clockSourceDivider / 4 - 1);
            break;
        case TIMER_A_CLOCKSOURCE_DIVIDER_40:
        case TIMER_A_CLOCKSOURCE_DIVIDER_48:
        case TIMER_A_CLOCKSOURCE_DIVIDER_56:
        case TIMER_A_CLOCKSOURCE_DIVIDER_64:
            HWREG(baseAddress + OFS_TAxCTL) |= ID__8;
            HWREG(baseAddress + OFS_TAxEX0) = (clockSourceDivider / 8 - 1);
            break;
    }
}

//*****************************************************************************
//
//! Sets the value of the capture-compare register
//!
//! \param baseAddress is the base address of the Timer module.
//! \param compareRegister selects the Capture register being used. Valid values
//!     are
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    Refer datasheet to ensure the device has the capture compare register
//!    being used
//! \param compareValue is the count to be compared with in compare mode
//!
//! Modified register is \b TAxCCRn
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_setCompareValue (  unsigned int baseAddress,
    unsigned int compareRegister,
    unsigned int compareValue
    )
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_1 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_2 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_3 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_4 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_5 == compareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_6 == compareRegister)
        );

    HWREG(baseAddress + compareRegister + OFS_TAxR) = compareValue;
}

//*****************************************************************************
//
//! Clears the Timer TAIFG interrupt flag
//!
//! \param baseAddress is the base address of the Timer module.
//!
//! Modified bits are TAIFG of TAxCTL register
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_clearTimerInterruptFlag (unsigned int baseAddress)
{
    HWREG(baseAddress + OFS_TAxCTL) &= ~TAIFG;
}

//*****************************************************************************
//
//! Clears the capture-compare interrupt flag
//!
//! \param baseAddress is the base address of the Timer module.
//! \param captureCompareRegister selects the Capture-compare register being
//! used. Valid values are
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_0
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_1
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_2
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_3
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_4
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_5
//!     \b TIMER_A_CAPTURECOMPARE_REGISTER_6
//!    Refer datasheet to ensure the device has the capture compare register
//!    being used
//!
//! Modified bits are CCIFG of \b TAxCCTLn register
//!
//! \return None
//
//*****************************************************************************
void TIMER_A_clearCaptureCompareInterruptFlag (unsigned int baseAddress,
    unsigned int captureCompareRegister
    )
{
    ASSERT((TIMER_A_CAPTURECOMPARE_REGISTER_0 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_1 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_2 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_3 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_4 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_5 == captureCompareRegister) ||
        (TIMER_A_CAPTURECOMPARE_REGISTER_6 == captureCompareRegister)
        );

    HWREG(baseAddress + captureCompareRegister)  &= ~CCIFG;
}

/*********************************************************************//*******
* @brief        	Timer Initialization Fucntion For Selected Timer
* @param[in]        None
* @return           None
*******************************************************************************/
void Timer_Init(unsigned int timer)
{
	switch(timer)
	{
	case TIMER_0:
		TIMER_A_startCounter(__MSP430_BASEADDRESS_T0A3__,TIMER_A_UP_MODE);
		TIMER_A_configureUpMode
			(__MSP430_BASEADDRESS_T0A3__,
					TIMER_A_CLOCKSOURCE_SMCLK,
					TIMER_A_CLOCKSOURCE_DIVIDER_1,20000,
					TIMER_A_TAIE_INTERRUPT_ENABLE,
					TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,
					TIMER_A_SKIP_CLEAR
			);
		break;
	case TIMER_1:
		TIMER_B_startCounter(__MSP430_BASEADDRESS_T1A2__,TIMER_B_UP_MODE);
		TIMER_B_configureUpMode
			( __MSP430_BASEADDRESS_T1A2__,
				TIMER_B_CLOCKSOURCE_SMCLK,
				TIMER_B_CLOCKSOURCE_DIVIDER_1,20000,
				TIMER_B_TBIE_INTERRUPT_ENABLE,
				TIMER_B_CCIE_CCR0_INTERRUPT_ENABLE,
				TIMER_B_SKIP_CLEAR
			);
		break;
	case TIMER_2:
		TIMER_D_startCounter(__MSP430_BASEADDRESS_T3A2__,TIMER_D_UP_MODE);

		TIMER_D_configureUpMode
			( __MSP430_BASEADDRESS_T1A2__,
				TIMER_D_CLOCKSOURCE_SMCLK,
				TIMER_D_CLOCKSOURCE_DIVIDER_1,
				TIMER_D_CLOCKINGMODE_AUXILIARY_CLK,20000,
				TIMER_D_TDIE_INTERRUPT_ENABLE,
				TIMER_D_CCIE_CCR0_INTERRUPT_ENABLE,
				TIMER_D_SKIP_CLEAR
			);
		break;
	default:
		break;
	}

}

/*****************************************************************//**
* @brief        		Provides Time Delay in msec.
* @param[in]        	Delay in msec.
* @return           	None
**********************************************************************/
void Delay_Ms(unsigned int dly)
{
	unsigned int i;
	dly = 1000/dly;
	_BIS_SR(GIE);

	TIMER_A_clear(__MSP430_BASEADDRESS_T0A3__);
	TIMER_A_startUpMode(__MSP430_BASEADDRESS_T0A3__,TIMER_A_CLOCKSOURCE_SMCLK,
						TIMER_A_CLOCKSOURCE_DIVIDER_4,20000,
						TIMER_A_TAIE_INTERRUPT_ENABLE,TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,TIMER_A_SKIP_CLEAR
						);

	for(i = 0;i<(25/dly);i++)
	{}

}

/****************************************************************//**
* @brief        		Provides Time Delay in usec.
* @param[in]        	Delay in usec.
* @return           	None
**********************************************************************/
void Delay_Us(unsigned int dly)
{
	unsigned int i;
	dly = 1000000/dly;
	_BIS_SR(GIE);

	TIMER_A_clear(__MSP430_BASEADDRESS_T0A3__);
	TIMER_A_startUpMode(__MSP430_BASEADDRESS_T0A3__,TIMER_A_CLOCKSOURCE_SMCLK,
			TIMER_A_CLOCKSOURCE_DIVIDER_4,20000,
			TIMER_A_TAIE_INTERRUPT_ENABLE,TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE,TIMER_A_SKIP_CLEAR
			);

	for(i = 0;i<(25/dly);i++)
	{}
}

//*****************************************************************************
//
//Close the Doxygen group.
//! @}
//
//*****************************************************************************
