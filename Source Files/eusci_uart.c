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
//uart.c - Driver for the UART Module.
//
//*****************************************************************************
//#include "inc/hw_types.h"
#include "debug.h"
#include "eusci_uart.h"
#include "eusci_euartbaudrate.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "msp430xgeneric.h"
#endif

//*************************UART A0 ISR **************************************//
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR (void)
{
	unsigned int get_che_status;
    switch (__even_in_range(UCA0IV, 4))
	{
	case USCI_UART_UCTXIFG:
	{
		EUSCI_UART_enableInterrupt(EUSCI_A0_BASE,EUSCI_UART_RECEIVE_INTERRUPT);
		break;
	}
	case USCI_UART_UCRXIFG:
	{
		r_flag = 1;
		EUSCI_UART_enableInterrupt(EUSCI_A0_BASE,EUSCI_UART_TRANSMIT_INTERRUPT);
		break;
	}
	default: break;
	}
}

//*************************UART A1 ISR **************************************//
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR (void)
{
	unsigned int get_che_status;
    switch (__even_in_range(UCA1IV, 4))
	{
	case USCI_UART_UCTXIFG:
	{
		EUSCI_UART_enableInterrupt(EUSCI_A1_BASE,EUSCI_UART_RECEIVE_INTERRUPT);
		break;
	}
	case USCI_UART_UCRXIFG:
	{
		r_flag = 1;
		uarta1_buffer[index++] = UCA1RXBUF;
		EUSCI_UART_enableInterrupt(EUSCI_A1_BASE,EUSCI_UART_TRANSMIT_INTERRUPT);
		break;
	}
	default: break;
	}
}
//*****************************************************************************
//
//! Initializes the UART block.
//!
//! \param baseAddress is the base address of the UART module.
//! \param selectClockSource selects Clock source. Valid values are
//!         \b EUSCI_UART_CLOCKSOURCE_SMCLK
//!         \b EUSCI_UART_CLOCKSOURCE_ACLK
//! \param clockSourceFrequency is the frequency of the slected clock source
//! \param desiredUartClock is the desired clock rate for UART communication
//! \param parity is the desired parity. Valid values are
//!        \b EUSCI_UART_NO_PARITY  [Default Value],
//!        \b EUSCI_UART_ODD_PARITY,
//!        \b EUSCI_UART_EVEN_PARITY
//! \param msborLsbFirst controls direction of receive and transmit shift
//!     register. Valid values are
//!        \b EUSCI_UART_MSB_FIRST
//!        \b EUSCI_UART_LSB_FIRST [Default Value]
//! \param numberofStopBits indicates one/two STOP bits
//!      Valid values are
//!        \b EUSCI_UART_ONE_STOP_BIT [Default Value]
//!        \b EUSCI_UART_TWO_STOP_BITS
//! \param uartMode selects the mode of operation
//!      Valid values are
//!        \b EUSCI_UART_MODE  [Default Value],
//!        \b EUSCI_UART_IDLE_LINE_MULTI_PROCESSOR_MODE,
//!        \b EUSCI_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE,
//!        \b EUSCI_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE
//! \param overSampling indicates low frequency or oversampling baud generation
//!      Valid values are
//!        \b EUSCI_UART_OVERSAMPLING_BAUDRATE_GENERATION
//!        \b EUSCI_UART_LOW_FREQUENCY_BAUDRATE_GENERATION
//!
//! Upon successful initialization of the UART block, this function
//! will have initialized the module, but the UART block still remains
//! disabled and must be enabled with EUSCI_UART_enable()
//!
//! Modified bits are \b UCPEN, \b UCPAR, \b UCMSB, \b UC7BIT, \b UCSPB,
//! \b UCMODEx, \b UCSYNC bits of \b UCAxCTL0 and \b UCSSELx,
//! \b UCSWRST bits of \b UCAxCTL1
//!
//! \return STATUS_SUCCESS or
//!         STATUS_FAIL of the initialization process
//
//*****************************************************************************
unsigned short EUSCI_UART_init ( unsigned int baseAddress,
    unsigned char selectClockSource,
    unsigned long clockSourceFrequency,
    unsigned long desiredUartBaudRate,
    unsigned char parity,
    unsigned int msborLsbFirst,
    unsigned int numberofStopBits,
    unsigned int uartMode,
    unsigned short overSampling
    )
{
    ASSERT(
        (EUSCI_UART_MODE == uartMode) ||
        (EUSCI_UART_IDLE_LINE_MULTI_PROCESSOR_MODE == uartMode) ||
        (EUSCI_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE == uartMode) ||
        (EUSCI_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE == uartMode)
        );

    ASSERT(
        (EUSCI_UART_CLOCKSOURCE_ACLK == selectClockSource) ||
        (EUSCI_UART_CLOCKSOURCE_SMCLK == selectClockSource)
        );

    ASSERT(
        (EUSCI_UART_MSB_FIRST == msborLsbFirst) ||
        (EUSCI_UART_LSB_FIRST == msborLsbFirst)
        );

    ASSERT(
        (EUSCI_UART_ONE_STOP_BIT == numberofStopBits) ||
        (EUSCI_UART_TWO_STOP_BITS == numberofStopBits)
        );

    ASSERT(
        (EUSCI_UART_NO_PARITY == parity) ||
        (EUSCI_UART_ODD_PARITY == parity) ||
        (EUSCI_UART_EVEN_PARITY == parity)
        );


    unsigned char retVal = STATUS_SUCCESS;
    unsigned int UCAxBRW_value = 0x00;
    unsigned int UCAxMCTL_value = 0x00;

    //Disable the USCI Module
    HWREG(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;

    //Clock source select
    HWREG(baseAddress + OFS_UCAxCTLW0) &= ~UCSSEL_3;
    HWREG(baseAddress + OFS_UCAxCTLW0) |= selectClockSource;

    //MSB, LSB select
    HWREG(baseAddress + OFS_UCAxCTLW0) &= ~UCMSB;
    HWREG(baseAddress + OFS_UCAxCTLW0) |= msborLsbFirst;


    //UCSPB = 0(1 stop bit) OR 1(2 stop bits)
    HWREG(baseAddress + OFS_UCAxCTLW0) &= ~UCSPB;
    HWREG(baseAddress + OFS_UCAxCTLW0) |= numberofStopBits;


    //Parity
    switch (parity){
        case EUSCI_UART_NO_PARITY:
            //No Parity
            HWREG(baseAddress + OFS_UCAxCTLW0) &= ~UCPEN;
            break;
        case EUSCI_UART_ODD_PARITY:
            //Odd Parity
            HWREG(baseAddress + OFS_UCAxCTLW0) |= UCPEN;
            HWREG(baseAddress + OFS_UCAxCTLW0) &= ~UCPAR;
            break;
        case EUSCI_UART_EVEN_PARITY:
            //Even Parity
            HWREG(baseAddress + OFS_UCAxCTLW0) |= UCPEN;
            HWREG(baseAddress + OFS_UCAxCTLW0) |= UCPAR;
            break;
    }

    //Calculate Baud rate divider values for Modulation control registers
    if ( STATUS_FAIL == eUARTBAUDRATE_calculateBaudDividers(clockSourceFrequency,
             desiredUartBaudRate,
             &UCAxBRW_value,
             &UCAxMCTL_value,
             overSampling
             )){
        return ( STATUS_FAIL) ;
    }

    //Modulation Control Registers
    HWREG(baseAddress + OFS_UCAxBRW ) = UCAxBRW_value;
    HWREG(baseAddress + OFS_UCAxMCTLW) = UCAxMCTL_value;

    //Asynchronous mode & 8 bit character select & clear mode
    HWREG(baseAddress + OFS_UCAxCTLW0) &=  ~(UCSYNC +
                                             UC7BIT +
                                             UCMODE_3
                                             );

    //Configure  UART mode.
    HWREG(baseAddress + OFS_UCAxCTLW0) |= uartMode ;

    //Reset UCRXIE, UCBRKIE, UCDORM, UCTXADDR, UCTXBRK
    HWREG(baseAddress + OFS_UCAxCTLW0)  &= ~(UCRXEIE + UCBRKIE + UCDORM +
                                             UCTXADDR + UCTXBRK
                                             );

    return ( retVal) ;
}

//*****************************************************************************
//
//! Advanced initialization routine for the UART block. The values to be written
//! into the UCAxBRW and UCAxMCTLW registers should be pre-computed and passed
//! into the initialization function
//!
//! \param baseAddress is the base address of the UART module.
//! \param selectClockSource selects Clock source. Valid values are
//!         \b EUSCI_UART_CLOCKSOURCE_SMCLK
//!         \b EUSCI_UART_CLOCKSOURCE_ACLK
//! \param clockPrescalar is the value to be written into UCBRx bits
//! \param firstModReg  is First modulation stage register setting. This value 
//! 	is a pre-calculated value which can be obtained from the Device User�s 
//!		Guide.This value is written into UCBRFx bits of UCAxMCTLW.
//! \param secondModReg is Second modulation stage register setting. 
//! 	This value is a pre-calculated value which can be obtained from the Device 
//! 	User�s Guide. This value is written into UCBRSx bits of UCAxMCTLW.
//! \param parity is the desired parity. Valid values are
//!        \b EUSCI_UART_NO_PARITY  [Default Value],
//!        \b EUSCI_UART_ODD_PARITY,
//!        \b EUSCI_UART_EVEN_PARITY
//! \param msborLsbFirst controls direction of receive and transmit shift
//!     register. Valid values are
//!        \b EUSCI_UART_MSB_FIRST
//!        \b EUSCI_UART_LSB_FIRST [Default Value]
//! \param numberofStopBits indicates one/two STOP bits
//!      Valid values are
//!        \b EUSCI_UART_ONE_STOP_BIT [Default Value]
//!        \b EUSCI_UART_TWO_STOP_BITS
//! \param uartMode selects the mode of operation
//!      Valid values are
//!        \b EUSCI_UART_MODE  [Default Value],
//!        \b EUSCI_UART_IDLE_LINE_MULTI_PROCESSOR_MODE,
//!        \b EUSCI_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE,
//!        \b EUSCI_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE
//! \param overSampling indicates low frequency or oversampling baud generation
//!      Valid values are
//!        \b EUSCI_UART_OVERSAMPLING_BAUDRATE_GENERATION
//!        \b EUSCI_UART_LOW_FREQUENCY_BAUDRATE_GENERATION
//!
//! Upon successful initialization of the UART block, this function
//! will have initialized the module, but the UART block still remains
//! disabled and must be enabled with EUSCI_UART_enable()
//!
//! Modified bits are \b UCPEN, \b UCPAR, \b UCMSB, \b UC7BIT, \b UCSPB,
//! \b UCMODEx, \b UCSYNC bits of \b UCAxCTL0 and \b UCSSELx,
//! \b UCSWRST bits of \b UCAxCTL1
//!
//! \return STATUS_SUCCESS or
//!         STATUS_FAIL of the initialization process
//
//*****************************************************************************
unsigned short EUSCI_UART_initAdvance ( unsigned int baseAddress,
    unsigned char selectClockSource,
    unsigned int clockPrescalar,
    unsigned char firstModReg,
    unsigned char secondModReg,
    unsigned char parity,
    unsigned int msborLsbFirst,
    unsigned int numberofStopBits,
    unsigned int uartMode,
    unsigned short overSampling
    )
{
    ASSERT(
        (EUSCI_UART_MODE == uartMode) ||
        (EUSCI_UART_IDLE_LINE_MULTI_PROCESSOR_MODE == uartMode) ||
        (EUSCI_UART_ADDRESS_BIT_MULTI_PROCESSOR_MODE == uartMode) ||
        (EUSCI_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE == uartMode)
        );

    ASSERT(
        (EUSCI_UART_CLOCKSOURCE_ACLK == selectClockSource) ||
        (EUSCI_UART_CLOCKSOURCE_SMCLK == selectClockSource)
        );

    ASSERT(
        (EUSCI_UART_MSB_FIRST == msborLsbFirst) ||
        (EUSCI_UART_LSB_FIRST == msborLsbFirst)
        );

    ASSERT(
        (EUSCI_UART_ONE_STOP_BIT == numberofStopBits) ||
        (EUSCI_UART_TWO_STOP_BITS == numberofStopBits)
        );

    ASSERT(
        (EUSCI_UART_NO_PARITY == parity) ||
        (EUSCI_UART_ODD_PARITY == parity) ||
        (EUSCI_UART_EVEN_PARITY == parity)
        );


    unsigned char retVal = STATUS_SUCCESS;

    //Disable the USCI Module
    HWREG(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;

    //Clock source select
    HWREG(baseAddress + OFS_UCAxCTLW0) &= ~UCSSEL_3;
    HWREG(baseAddress + OFS_UCAxCTLW0) |= selectClockSource;

    //MSB, LSB select
    HWREG(baseAddress + OFS_UCAxCTLW0) &= ~UCMSB;
    HWREG(baseAddress + OFS_UCAxCTLW0) |= msborLsbFirst;


    //UCSPB = 0(1 stop bit) OR 1(2 stop bits)
    HWREG(baseAddress + OFS_UCAxCTLW0) &= ~UCSPB;
    HWREG(baseAddress + OFS_UCAxCTLW0) |= numberofStopBits;


    //Parity
    switch (parity){
        case EUSCI_UART_NO_PARITY:
            //No Parity
            HWREG(baseAddress + OFS_UCAxCTLW0) &= ~UCPEN;
            break;
        case EUSCI_UART_ODD_PARITY:
            //Odd Parity
            HWREG(baseAddress + OFS_UCAxCTLW0) |= UCPEN;
            HWREG(baseAddress + OFS_UCAxCTLW0) &= ~UCPAR;
            break;
        case EUSCI_UART_EVEN_PARITY:
            //Even Parity
            HWREG(baseAddress + OFS_UCAxCTLW0) |= UCPEN;
            HWREG(baseAddress + OFS_UCAxCTLW0) |= UCPAR;
            break;
    }

    //BaudRate Control Register
    HWREG(baseAddress + OFS_UCAxBRW ) = clockPrescalar;
    //Modulation Control Register
    HWREG(baseAddress + OFS_UCAxMCTLW) = ((secondModReg <<8) + (firstModReg <<4) + overSampling );

    //Asynchronous mode & 8 bit character select & clear mode
    HWREG(baseAddress + OFS_UCAxCTLW0) &=  ~(UCSYNC +
                                             UC7BIT +
                                             UCMODE_3
                                             );

    //Configure  UART mode.
    HWREG(baseAddress + OFS_UCAxCTLW0) |= uartMode ;

    //Reset UCRXIE, UCBRKIE, UCDORM, UCTXADDR, UCTXBRK
    HWREG(baseAddress + OFS_UCAxCTLW0)  &= ~(UCRXEIE + UCBRKIE + UCDORM +
                                             UCTXADDR + UCTXBRK
                                             );

    return ( retVal) ;
}
//*****************************************************************************
//
//! Transmits a byte from the UART Module.
//!
//! \param baseAddress is the base address of the UART module.
//! \param transmitData data to be transmitted from the UART module
//!
//! This function will place the supplied data into UART trasmit data register
//! to start transmission
//!
//! Modified register is \b UCAxTXBUF
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_transmitData ( unsigned int baseAddress,
    unsigned char transmitData
    )
{
    HWREG(baseAddress + OFS_UCAxTXBUF) = transmitData;
}

//*****************************************************************************
//
//! Receives a byte that has been sent to the UART Module.
//!
//! \param baseAddress is the base address of the UART module.
//!
//! This function reads a byte of data from the UART receive data Register.
//!
//! Modified register is \b UCAxRXBUF
//!
//! \return Returns the byte received from by the UART module, cast as an
//! unsigned char.
//
//*****************************************************************************
unsigned char EUSCI_UART_receiveData (unsigned int baseAddress)
{
    return ( HWREG(baseAddress + OFS_UCAxRXBUF)) ;
}

//*****************************************************************************
//
//! Enables individual UART interrupt sources.
//!
//! \param baseAddress is the base address of the UART module.
//! \param mask is the bit mask of the interrupt sources to be enabled.
//!
//! Enables the indicated UART interrupt sources.  The interrupt flag is first
//! and then the corresponfing interrupt is enabled. Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The mask parameter is the logical OR of any of the following:
//! - \b EUSCI_UART_RECEIVE_INTERRUPT -Receive interrupt
//! - \b EUSCI_UART_TRANSMIT_INTERRUPT - Transmit interrupt
//! - \b EUSCI_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT - Receive erroneous-character
//!                             interrupt enable
//! - \b EUSCI_UART_BREAKCHAR_INTERRUPT - Receive break character interrupt enable
//!
//! Modified register is \b UCAxIFG, \b UCAxIE and \b UCAxCTL1
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_enableInterrupt (unsigned int baseAddress,
    unsigned char mask
    )
{
    ASSERT((EUSCI_UART_RECEIVE_INTERRUPT == mask) ||
        (EUSCI_UART_TRANSMIT_INTERRUPT == mask) ||
        (EUSCI_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT == mask) ||
        (EUSCI_UART_BREAKCHAR_INTERRUPT == mask)
        );
    switch (mask){
        case EUSCI_UART_RECEIVE_INTERRUPT:
        case EUSCI_UART_TRANSMIT_INTERRUPT:
            //Clear interrupt flag
            HWREG(baseAddress + OFS_UCAxIFG) &= ~(mask);
            //Enable Interrupt
            HWREG(baseAddress + OFS_UCAxIE) |= mask;
            break;
        case EUSCI_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT:
        case EUSCI_UART_BREAKCHAR_INTERRUPT:
            //Enable Interrupt
            HWREG(baseAddress + OFS_UCAxCTLW0) |= mask;
            break;
    }
}

//*****************************************************************************
//
//! Disables individual UART interrupt sources.
//!
//! \param baseAddress is the base address of the UART module.
//! \param mask is the bit mask of the interrupt sources to be
//! disabled.
//!
//! Disables the indicated UART interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The mask parameter is the logical OR of any of the following:
//! - \b EUSCI_UART_RECEIVE_INTERRUPT -Receive interrupt
//! - \b EUSCI_UART_TRANSMIT_INTERRUPT - Transmit interrupt
//! - \b EUSCI_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT - Receive erroneous-character
//!                             interrupt enable
//! - \b EUSCI_UART_BREAKCHAR_INTERRUPT - Receive break character interrupt enable
//!
//! Modified register is \b UCAxIFG, \b UCAxIE and \b UCAxCTL1
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_disableInterrupt (unsigned int baseAddress,
    unsigned char mask
    )
{
    ASSERT((EUSCI_UART_RECEIVE_INTERRUPT == mask) ||
        (EUSCI_UART_TRANSMIT_INTERRUPT == mask) ||
        (EUSCI_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT == mask) ||
        (EUSCI_UART_BREAKCHAR_INTERRUPT == mask)
        );

    switch (mask){
        case EUSCI_UART_RECEIVE_INTERRUPT:
        case EUSCI_UART_TRANSMIT_INTERRUPT:
            //Disable Interrupt
            HWREG(baseAddress + OFS_UCAxIE) &= ~mask;
            break;
        case EUSCI_UART_RECEIVE_ERRONEOUSCHAR_INTERRUPT:
        case EUSCI_UART_BREAKCHAR_INTERRUPT:
            //Disable Interrupt
            HWREG(baseAddress + OFS_UCAxCTLW0) &= ~mask;
            break;
    }
}

//*****************************************************************************
//
//! Gets the current UART interrupt status.
//!
//! \param baseAddress is the base address of the UART module.
//! \param mask is the masked interrupt flag status to be returned.
//!
//! This returns the interrupt status for the UART  module based on which
//! flag is passed. mask parameter can be either any of the following
//! selection.
//! - \b EUSCI_UART_RECEIVE_INTERRUPT_FLAG -Receive interrupt flag
//! - \b EUSCI_UART_TRANSMIT_INTERRUPT_FLAG - Transmit interrupt flag
//!
//! Modified register is \b UCAxIFG.
//!
//! \return The current interrupt status, returned as with the respective bits
//! set if the corresponding interrupt flag is set
//
//*****************************************************************************
unsigned char EUSCI_UART_getInterruptStatus (unsigned int baseAddress,
    unsigned char mask)
{
    ASSERT(  (EUSCI_UART_RECEIVE_INTERRUPT_FLAG == mask) ||
        (EUSCI_UART_TRANSMIT_INTERRUPT_FLAG == mask)
        );

    return ( HWREG(baseAddress + OFS_UCAxIFG) & mask );
}

//*****************************************************************************
//
//! Clears UART interrupt sources.
//!
//! \param baseAddress is the base address of the UART module.
//! \param mask is a bit mask of the interrupt sources to be cleared.
//!
//! The UART interrupt source is cleared, so that it no longer asserts.
//! The highest interrupt flag is automatically cleared when an interrupt vector
//! generator is used.
//!
//! The mask parameter has the same definition as the mask parameter to
//! EUSCI_UART_enableInterrupt().
//!
//! Modified register is \b UCAxIFG
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_clearInterruptFlag (unsigned int baseAddress, unsigned char mask)
{
    //Clear the UART interrupt source.
    HWREG(baseAddress + OFS_UCAxIFG) &= ~(mask);
}

//*****************************************************************************
//
//! Enables the UART block.
//!
//! \param baseAddress is the base address of the USCI UART module.
//!
//! This will enable operation of the UART block.
//!
//! Modified register is \b UCAxCTL1
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_enable (unsigned int baseAddress)
{
    //Reset the UCSWRST bit to enable the USCI Module
    HWREG(baseAddress + OFS_UCAxCTLW0) &= ~(UCSWRST);
}

//*****************************************************************************
//
//! Disables the UART block.
//!
//! \param baseAddress is the base address of the USCI UART module.
//!
//! This will disable operation of the UART block.
//!
//! Modified register is \b UCAxCTL1
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_disable (unsigned int baseAddress)
{
    //Set the UCSWRST bit to disable the USCI Module
    HWREG(baseAddress + OFS_UCAxCTLW0) |= UCSWRST;
}

//*****************************************************************************
//
//! Gets the current UART status flags.
//!
//! \param baseAddress is the base address of the UART module.
//! \param mask is the masked interrupt flag status to be returned.
//!
//! This returns the status for the UART  module based on which
//! flag is passed. mask parameter can be either any of the following
//! selection.
//! - \b EUSCI_UART_LISTEN_ENABLE
//! - \b EUSCI_UART_FRAMING_ERROR
//! - \b EUSCI_UART_OVERRUN_ERROR
//! - \b EUSCI_UART_PARITY_ERROR
//! - \b eUARTBREAK_DETECT
//! - \b EUSCI_UART_RECEIVE_ERROR
//! - \b EUSCI_UART_ADDRESS_RECEIVED
//! - \b EUSCI_UART_IDLELINE
//! - \b EUSCI_UART_BUSY
//!
//! Modified register is \b UCAxSTAT
//!
//! \return the masked status flag
//
//*****************************************************************************
unsigned char EUSCI_UART_queryStatusFlags (unsigned int baseAddress,
    unsigned char mask)
{
    ASSERT((EUSCI_UART_LISTEN_ENABLE == mask) ||
        (EUSCI_UART_FRAMING_ERROR == mask) ||
        (EUSCI_UART_OVERRUN_ERROR == mask) ||
        (EUSCI_UART_PARITY_ERROR == mask) ||
        (eUARTBREAK_DETECT == mask) ||
        (EUSCI_UART_RECEIVE_ERROR == mask) ||
        (EUSCI_UART_ADDRESS_RECEIVED == mask) ||
        (EUSCI_UART_IDLELINE == mask) ||
        (EUSCI_UART_BUSY == mask)
        );
    return ( HWREG(baseAddress + OFS_UCAxSTATW) & mask );
}

//*****************************************************************************
//
//! Sets the UART module in dormant mode
//!
//! \param baseAddress is the base address of the UART module.
//!
//! Puts USCI in sleep mode
//! Only characters that are preceded by an idle-line or with address bit set
//! UCRXIFG. In UART mode with automatic baud-rate detection, only the
//! combination of a break and synch field sets UCRXIFG.
//!
//! Modified register is \b UCAxCTL1
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_setDormant (unsigned int baseAddress)
{
    HWREG(baseAddress + OFS_UCAxCTLW0) |= UCDORM;
}

//*****************************************************************************
//
//! Re-enables UART module from dormant mode
//!
//! \param baseAddress is the base address of the UART module.
//!
//! Not dormant. All received characters set UCRXIFG.
//!
//! Modified bits are \b UCDORM of \b UCAxCTL1 register.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_resetDormant (unsigned int baseAddress)
{
    HWREG(baseAddress + OFS_UCAxCTLW0) &= ~UCDORM;
}

//*****************************************************************************
//
//! Transmits the next byte to be transmitted marked as address depending on
//! selected multiprocessor mode
//!
//! \param baseAddress is the base address of the UART module.
//! \param transmitAddress is the next byte to be transmitted
//!
//! Modified register is \b UCAxCTL1, \b UCAxTXBUF
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_transmitAddress (unsigned int baseAddress,
    unsigned char transmitAddress)
{
    //Set UCTXADDR bit
    HWREG(baseAddress + OFS_UCAxCTLW0) |= UCTXADDR;

    //Place next byte to be sent into the transmit buffer
    HWREG(baseAddress + OFS_UCAxTXBUF) = transmitAddress;
}

//*****************************************************************************
//
//! Transmit break. Transmits a break with the next write to the transmit
//! buffer. In UART mode with automatic baud-rate detection,
//! EUSCI_UART_AUTOMATICBAUDRATE_SYNC(0x55) must be written into UCAxTXBUF to
//! generate the required break/synch fields.
//! Otherwise, DEFAULT_SYNC(0x00) must be written into the transmit buffer.
//! Also ensures module is ready for transmitting the next data
//!
//! \param baseAddress is the base address of the UART module.
//!
//! Modified register is \b UCAxCTL1, \b UCAxTXBUF
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_UART_transmitBreak (unsigned int baseAddress)
{
    //Set UCTXADDR bit
    HWREG(baseAddress + OFS_UCAxCTLW0) |= UCTXBRK;

    //If current mode is automatic baud-rate detection
    if (EUSCI_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE ==
        (HWREG(baseAddress + OFS_UCAxCTLW0) &
         EUSCI_UART_AUTOMATIC_BAUDRATE_DETECTION_MODE)){
        HWREG(baseAddress + OFS_UCAxTXBUF) = EUSCI_UART_AUTOMATICBAUDRATE_SYNC;
    } else   {
        HWREG(baseAddress + OFS_UCAxTXBUF) = DEFAULT_SYNC;
    }

    //USCI TX buffer ready?
    while (!EUSCI_UART_getInterruptStatus(baseAddress, UCTXIFG)) ;
}

//*****************************************************************************
//
//! Returns the address of the RX Buffer of the UART for the DMA module.
//!
//! \param baseAddress is the base address of the UART module.
//!
//! Returns the address of the UART RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \return None
//
//*****************************************************************************
unsigned long EUSCI_UART_getReceiveBufferAddressForDMA (unsigned int baseAddress)
{
    return ( baseAddress + OFS_UCAxRXBUF );
}

//*****************************************************************************
//
//! Returns the address of the TX Buffer of the UART for the DMA module.
//!
//! \param baseAddress is the base address of the UART module.
//!
//! Returns the address of the UART TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \return None
//
//*****************************************************************************
unsigned long EUSCI_UART_getTransmitBufferAddressForDMA (unsigned int baseAddress)
{
    return ( baseAddress + OFS_UCAxTXBUF );
}

//*****************************************************************************
//
//! Sets the deglitch time
//!
//! \param baseAddress is the base address of the UART module.
//! \param deglitchTime is the selected deglitch time
//! 	Valid values are 
//! 		- \b EUSCI_UART_DEGLITCH_TIME_2ns
//! 		- \b EUSCI_UART_DEGLITCH_TIME_50ns
//! 		- \b EUSCI_UART_DEGLITCH_TIME_100ns
//! 		- \b EUSCI_UART_DEGLITCH_TIME_200ns
//!		
//!
//! Returns the address of the UART TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \return None
//
//*****************************************************************************
void EUSCI_UART_selectDeglitchTime(unsigned int baseAddress,
			unsigned long deglitchTime
			)
{
	ASSERT((EUSCI_UART_DEGLITCH_TIME_2ns == deglitchTime) ||
        (EUSCI_UART_DEGLITCH_TIME_50ns == deglitchTime) ||
        (EUSCI_UART_DEGLITCH_TIME_100ns == deglitchTime) ||
        (EUSCI_UART_DEGLITCH_TIME_200ns == deglitchTime)
        );
    
    HWREG(baseAddress + OFS_UCAxCTLW1) &= ~(UCGLIT1 + UCGLIT0);
    
    HWREG(baseAddress + OFS_UCAxCTLW1) = deglitchTime;
}

/*****************************************************************//**
* @brief        		Initialize All UART Module.
* @param[in]        	UART Module like uartA0,uartA1 AND Baud Rate.
* @return           	None
**********************************************************************/
void Uart_Init(UartId_e uartx,long int BAUD_RATE)
{
	if(uartx==UARTA0)
	{

	//P1.2,3 = USCI_A0 TXD/RXD
	GPIO_setAsPeripheralModuleFunctionOutputPin(
			GPIO_PORT_P1,
			GPIO_PIN2
			);

	GPIO_setAsPeripheralModuleFunctionInputPin(
			GPIO_PORT_P1,
			GPIO_PIN3
			);

	//Initialize USCI UART module
	if ( STATUS_FAIL == EUSCI_UART_init(EUSCI_A0_BASE,
				 EUSCI_UART_CLOCKSOURCE_SMCLK,
				 UCS_getSMCLK(UCS_BASE),
				 BAUD_RATE,
				 EUSCI_UART_NO_PARITY,
				 EUSCI_UART_LSB_FIRST,
				 EUSCI_UART_ONE_STOP_BIT,
				 EUSCI_UART_MODE,
				 EUSCI_UART_LOW_FREQUENCY_BAUDRATE_GENERATION ))
	{
		return;
	}
	//Enable UART module for operation
	EUSCI_UART_enable(EUSCI_A0_BASE);
#ifdef INTERRUPT_MODE
	EUSCI_UART_enableInterrupt(EUSCI_A0_BASE,EUSCI_UART_TRANSMIT_INTERRUPT);	// Enable UART Interrupts.
	EUSCI_UART_enableInterrupt(EUSCI_A0_BASE,EUSCI_UART_RECEIVE_INTERRUPT);
#endif
	}
	else if(uartx==UARTA1)
	{

		//P1.4,5 = USCI_A1 TXD/RXD
		GPIO_setAsPeripheralModuleFunctionOutputPin(
				GPIO_PORT_P1,
				GPIO_PIN4
				);

		GPIO_setAsPeripheralModuleFunctionInputPin(
				GPIO_PORT_P1,
				GPIO_PIN5
				);

		//Initialize USCI UART module
		if ( STATUS_FAIL == EUSCI_UART_init(EUSCI_A1_BASE,
							 EUSCI_UART_CLOCKSOURCE_SMCLK,
							 UCS_getSMCLK(UCS_BASE),
							 BAUD_RATE,
							 EUSCI_UART_NO_PARITY,
							 EUSCI_UART_LSB_FIRST,
							 EUSCI_UART_ONE_STOP_BIT,
							 EUSCI_UART_MODE,
							 EUSCI_UART_LOW_FREQUENCY_BAUDRATE_GENERATION ))
		{
			return;
		}
		//Enable UART module for operation
		EUSCI_UART_enable(EUSCI_A1_BASE);
#ifdef INTERRUPT_MODE
	EUSCI_UART_enableInterrupt(EUSCI_A1_BASE,EUSCI_UART_TRANSMIT_INTERRUPT);	// Enable UART Interrupts.
	EUSCI_UART_enableInterrupt(EUSCI_A1_BASE,EUSCI_UART_RECEIVE_INTERRUPT);
#endif
	}
}
#ifdef POLLING_MODE
/*****************************************************************//**
* @brief        		Write The Data to UART Port Selected.
* @param[in]        	UART Module like uartA0,uartA1 AND Data.
* @return           	None
**********************************************************************/
void UART_Send(UartId_e uartx,char Data)
{
	if(uartx==UARTA0)
	{
		while (!EUSCI_UART_getInterruptStatus(EUSCI_A0_BASE,
						        EUSCI_UART_TRANSMIT_INTERRUPT_FLAG));
		EUSCI_UART_transmitData(EUSCI_A0_BASE,Data);
	}
	else if(uartx==UARTA1)
	{
		while (!EUSCI_UART_getInterruptStatus(EUSCI_A1_BASE,
				                EUSCI_UART_TRANSMIT_INTERRUPT_FLAG)) ;
		EUSCI_UART_transmitData(EUSCI_A1_BASE,Data);
	}
	else
	{
		while (!EUSCI_UART_getInterruptStatus(EUSCI_A2_BASE,
						        EUSCI_UART_TRANSMIT_INTERRUPT_FLAG)) ;
		EUSCI_UART_transmitData(EUSCI_A2_BASE,Data);
	}
}

unsigned int UART_Recieve(UartId_e uartx,char *data,int length,TRANSFER_BLOCK_Type uartm)
{
	unsigned int i;
	if(uartx==UARTA0)
	{
		for(i=0;i<length;i++)
		{
			while (!EUSCI_UART_getInterruptStatus(EUSCI_A0_BASE,
										   EUSCI_UART_RECEIVE_INTERRUPT_FLAG)) ;
			data[i] = UCA0RXBUF;
		}
	}
	else if(uartx==UARTA1)
	{
		if(uartm == BLOCKING)
		{
			for(i=0;i<length;i++)
			{
				while (!EUSCI_UART_getInterruptStatus(EUSCI_A1_BASE,
											   EUSCI_UART_RECEIVE_INTERRUPT_FLAG)) ;
				data[i] = UCA1RXBUF;
			}
		}
		else if(uartm == NONE_BLOCKING)
		{
			for(i=0;i<length;i++)
			{
				if(!EUSCI_UART_getInterruptStatus(EUSCI_A1_BASE,
											   EUSCI_UART_RECEIVE_INTERRUPT_FLAG))
				{
					data[i] = 0x00;
				}
				else
				{
					data[i] = UCA0RXBUF;
				}
			}
		}
		else if(uartm == TIME_BLOCKING)
		{
			unsigned long int time = 1250000;
			for(i=0;i<length;i++)
			{
				while(!EUSCI_UART_getInterruptStatus(EUSCI_A1_BASE,
											   EUSCI_UART_RECEIVE_INTERRUPT_FLAG))
				{
					time--;
					if(time == 0)
					{
						data[i] = 0x00;
					}
				}
			}
			data[i] = UCA0RXBUF;
		}
	}
}

/*****************************************************************//**
* @brief        		Write The String of Data to UART Port Selected.
* @param[in]        	UART Module like uartA0,uartA1 AND String of Data.
* @return           	None
**********************************************************************/
void WriteDataStringUart(UartId_e uartx,char *String)
{
    int i = 0;
	while (String[i] != 0)
	{
		UART_Send(uartx,String[i]);
		i++;
	}
}
#endif

#ifdef INTERRUPT_MODE
/*****************************************************************//**
* @brief        		Write The Data to UART Port Selected.
* @param[in]        	UART Module like uartA0,uartA1 AND Data.
* @return           	None
**********************************************************************/
void UART_Send(UartId_e uartx,char Data)
{
	if(uartx==UARTA0)
	{
		EUSCI_UART_transmitData(EUSCI_A0_BASE,Data);
	}
	else if(uartx==UARTA1)
	{
		EUSCI_UART_transmitData(EUSCI_A1_BASE,Data);
	}
}

unsigned int UART_Recieve(UartId_e uartx,unsigned char *data,int length,TRANSFER_BLOCK_Type uartm)
{
	unsigned int i;
	if(uartx == UARTA0)
	{
		while(r_flag == 0);

			unsigned int i;
			for(i=0;i<index;i++)
			{
				data[i] = uarta1_buffer[i];
				if(length == (index))
				{
					index = 0;
				}
			}
			r_flag = 0;
			return(i);


	}
	else
	{
		if(uartm == BLOCKING)
		{
			while(r_flag == 0);

			unsigned int i;
			for(i=0;i<index;i++)
			{
				data[i] = uarta1_buffer[i];
				if(length == (index))
				{
					index = 0;
				}
			}
			r_flag = 0;
			return(i);
		}
		else if(uartm == NONE_BLOCKING)
		{
			if(r_flag == 0)
			{
				data[i] = 0x00;
			}
			else
			{
				unsigned int i;
				for(i=0;i<index;i++)
				{
					data[i] = uarta1_buffer[i];
					if(length == (index))
					{
						index = 0;
					}
				}
				r_flag = 0;
			}
			return(i);
		}
		else if(uartm == TIME_BLOCKING)
		{
			unsigned int time = 50000;
			if(r_flag == 0)
			{
				time--;
				if(time == 0)
				{
					data[i] = 0x00;
				}
			}
			else
			{
				unsigned int i;
				for(i=0;i<index;i++)
				{
					data[i] = uarta1_buffer[i];
					if(length == (index))
					{
						index = 0;
					}
				}
				r_flag = 0;
			}
			return(i);
		}
	}

}

/*****************************************************************//**
* @brief        		Write The String of Data to UART Port Selected.
* @param[in]        	UART Module like uartA0,uartA1 AND String of Data.
* @return           	None
**********************************************************************/
void WriteDataStringUart(UartId_e uartx,char *String)
{
    int i = 0;
	while (String[i] != 0)
	{
		UART_Send(uartx,String[i]);
		Delay_Ms(2);
		i++;
	}
}
#endif

/**********************************************************************//**
* @brief        		Read the single byte of Data from UART Port Selected.
* @param[in]        	UART Module like uartA0,uartA1.
* @return           	Character Received from UART.
***************************************************************************/
char ReadDataUart(UartId_e uartx)
{
	if(uartx==UARTA0)
	{
		while (!EUSCI_UART_getInterruptStatus(EUSCI_A0_BASE,
		                       EUSCI_UART_RECEIVE_INTERRUPT_FLAG)) ;
		return(EUSCI_UART_receiveData(EUSCI_A0_BASE));
	}
	else if(uartx==UARTA1)
	{
		while (!EUSCI_UART_getInterruptStatus(EUSCI_A1_BASE,
				               EUSCI_UART_RECEIVE_INTERRUPT_FLAG)) ;
		return(EUSCI_UART_receiveData(EUSCI_A1_BASE));
	}
	else
	{
		while (!EUSCI_UART_getInterruptStatus(EUSCI_A2_BASE,
				               EUSCI_UART_RECEIVE_INTERRUPT_FLAG)) ;
	    return(EUSCI_UART_receiveData(EUSCI_A2_BASE));
	}
}

/******************************************************************************//**
* @brief        		Display the data of any format on UART Port Selected.
* @param[in]        	UART Module (uartA0,uartA1) , Data you want to Display.
* @return           	Data will be Displayed on Serial PORT TERMINAL .
**********************************************************************************/
char printf(UartId_e uartx,char *format, ...)
{
	char hex[]= "0123456789ABCDEF";
	unsigned int width_dec[10] = { 1, 10, 100, 1000,10000};
	unsigned int width_hex[10] = { 0x1, 0x10, 0x100, 0x1000};
	unsigned int temp;

	char format_flag, fill_char,temp_data;
	unsigned long u_val, div_val;
	unsigned int base;

	char *ptr;
	va_list ap;
	va_start(ap, format);

	for(;;)
	{
		while((format_flag = *format++) != '%')      /* until full format string read */
		{
			if(!format_flag)
			{                        /* until '%' or '\0' */
				return(0);
			}
			UART_Send(uartx,format_flag);
			Delay_Ms(2);
		}
		switch(format_flag = *format++)
		{
			case 'c':
				format_flag = va_arg(ap,int);
				UART_Send(uartx,format_flag);
				Delay_Ms(2);

				continue;

			default:
				UART_Send(uartx,format_flag);
				Delay_Ms(2);

				continue;

			case 'b':
				format_flag = va_arg(ap,int);
				temp_data = (hex[(unsigned int)format_flag >> 4]);
				UART_Send(uartx,temp_data);
				Delay_Ms(2);
				temp_data = (hex[(unsigned int)format_flag & 0x0F]);
				UART_Send(uartx,temp_data);
				Delay_Ms(2);

				continue;

			case 's':
				ptr = va_arg(ap,char *);
				while(*ptr)
				{
					temp_data = (*ptr++);
					UART_Send(uartx,temp_data);
					Delay_Ms(2);
				}

				continue;

			case 'd':
				base = 10;
				if(*format == ' ')
				{
					format_flag = 0;
					//*format++;
				}
				else
				{
					fill_char = *format++;
					format_flag = ( *format++) - '1';

				}
				div_val = width_dec[format_flag];
				u_val = va_arg(ap,int);
				if(((int)u_val) < 0)
				{
					u_val = - u_val;    /* applied to unsigned type, result still unsigned */
					temp = '-';
					UART_Send(uartx,temp);
					Delay_Ms(2);
				}

				goto CONVER_LOOP;

			case 'x':
				base = 16;
				if(*format == ' ')
				{
					format_flag = 0;
					//*format++;
				}
				else
				{
					fill_char = *format++;
					format_flag = ( *format) - '1';
					*format++;
				}
				div_val = width_hex[format_flag];
				u_val = va_arg(ap, int);

CONVER_LOOP:
				while(div_val > 1 && div_val > u_val)
				{
					div_val /= base;
					UART_Send(uartx,fill_char);
					Delay_Ms(2);
				}

				do
				{
					temp_data = (hex[u_val/div_val]);
					UART_Send(uartx,temp_data);
					Delay_Ms(2);
					u_val %= div_val;
					div_val /= base;
				}while(div_val);
			}/* end of switch statement */
		}
		return(0);
}

/******************************************************************************//**
* @brief        		Take a Single character of data from UART Port Selected.
* @param[in]        	UART Module(UARTA0,UARTA1) , TRANSFER BLOCK TYPE.
* @return           	Received Data will be returned to an INT variable.
**********************************************************************************/
int getche(UartId_e uartx,TRANSFER_BLOCK_Type uartm)
{
	unsigned char len[1]={0};
	while(1)
	{
		UART_Recieve(uartx,len,1,uartm);
		while(len[0]>0)
		{
			if (len[0] == ENTER_KEY)
			{
				return(ENTER_KEY);
			}
			else if (len[0] == BACKSPACE_KEY)
			{
				return(BACKSPACE_KEY);
			}
			else if(len[0] == ESC_KEY)
			{
				Esc_Flag = 1;
				return(ESC_KEY);
			}
			else if ((len[0] >= 0x20) && (len[0] <= 0x7F))
			{
				return(len[0]);
			}
		}
	}
}

/***********************************************************************************************//**
* @brief        		Take data of variable length from UART Port Selected.
* @param[in]        	UART Module(UARTA0,UARTA1) , Character Array , LENGTH , TRANSFER BLOCK TYPE.
* @return           	Received Data will be returned to character array.
***************************************************************************************************/
char getline(UartId_e uartx,char *s,unsigned int length,TRANSFER_BLOCK_Type uartm)
{
	unsigned int j = 0,temp;
	unsigned int getline_data;
	while(1)
	{
		getline_data = getche(uartx,uartm);
		if(j < length)
		{
			if(getline_data == BACKSPACE_KEY)
			{
				temp = BACKSPACE_KEY;
				printf(UARTA1,"%c",temp);
				temp = ' ';
				printf(UARTA1,"%c",temp);
				temp = BACKSPACE_KEY;
				printf(UARTA1,"%c",temp);
				j--;
			}
			else if(getline_data == ENTER_KEY)
			{
				return j;
			}
			else if((getline_data >= ' ') && (getline_data <= 0x7F))
			{
				s[j] = getline_data;
				UART_Send(UARTA1,s[j]);
				j++;
			}
		}
		else
		{
			UART_Send(uartx,BELL_KEY);
			if(getline_data == ENTER_KEY)
			{
				return j;
			}
			j++;
		}
	}
}

/***********************************************************************************************//**
* @brief        		Take data of variable length of HEX format from UART Port Selected.
* @param[in]        	UART Module(UARTA0,UARTA1) , Character Array , LENGTH , TRANSFER BLOCK TYPE.
* @return           	Received Data will be returned in HEX format to character array.
***************************************************************************************************/
unsigned int getline_hex(UartId_e uartx,char s[10],unsigned int length,TRANSFER_BLOCK_Type uartm)
{
	unsigned char dummy[10]={0};
	unsigned int j = 0,temp,str_ad=0,i,b=0,c=0,k;
	unsigned int getline_data;
	while(1)
	{
		getline_data = getche(uartx,uartm);
		if(getline_data == BACKSPACE_KEY)
		{
			if(j>0)
			{
				temp = BACKSPACE_KEY;
				printf(UARTA1,"%c",temp);
				temp = ' ';
				printf(UARTA1,"%c",temp);
				temp = BACKSPACE_KEY;
				printf(UARTA1,"%c",temp);
				j--;
			}
			else
			{
				UART_Send(uartx,BELL_KEY);
			}
		}
		else if(getline_data == ENTER_KEY)
		{
			if(j>(length-1))
			{
				for(i=0;i<length;i++)
				{
					dummy[i] = dummy[i] - 0x30;
					if((dummy[i] >= 0x10) && (dummy[i] <= 0x15))
					{
						dummy[i] = dummy[i] - 6;
					}
				}
				for(i=length;i>0;i--)
				{
					c = 1;
					for(k=0;k<(i-1);k++)
					{
						b = 16;
						c = c*b;
					}
					str_ad += (dummy[length-i]*c);
				}
				return str_ad;
			}
		}
		else if((getline_data >= 0x30) && (getline_data <= 0x46))
		{
			if(j<length)
			{
				s[j] = getline_data;
				UART_Send(UARTA1,s[j]);
				if((s[j] >= 0x41) && (s[j] <= 0x46))
				{
					s[j] = s[j]-1;
				}
				dummy[j] = s[j];
				j++;
			}
		}
		else if(j>length)
		{
			UART_Send(uartx,BELL_KEY);
			if(getline_data == ENTER_KEY)
			{
				return j;
			}
		}
		else
		{
			UART_Send(uartx,BELL_KEY);
		}
	}
}

/***********************************************************************************************//**
* @brief        		Take data of variable length of DECIMAL format from UART Port Selected.
* @param[in]        	UART Module(UARTA0,UARTA1) , Character Array , LENGTH , TRANSFER BLOCK TYPE.
* @return           	Received Data will be returned in DECIMAL format to character array.
***************************************************************************************************/
unsigned int getline_dec(UartId_e uartx,char s[10],unsigned int length,TRANSFER_BLOCK_Type uartm)
{
	unsigned char dummy[10]={0};
	unsigned int j = 0,temp,str_ad=0,i,b=0,c=0,k;
	unsigned int getline_data;
	while(1)
	{
		getline_data = getche(uartx,uartm);
		if(getline_data == BACKSPACE_KEY)
		{
			if(j>0)
			{
				temp = BACKSPACE_KEY;
				printf(UARTA1,"%c",temp);
				temp = ' ';
				printf(UARTA1,"%c",temp);
				temp = BACKSPACE_KEY;
				printf(UARTA1,"%c",temp);
				j--;
			}
			else
			{
				UART_Send(uartx,BELL_KEY);
			}
		}
		else if(getline_data == ENTER_KEY)
		{
			if(j>(length-1))
			{
				for(i=0;i<length;i++)
				{
					dummy[i] = dummy[i] - 0x30;
				}

				for(i=length;i>0;i--)
				{
					c = 1;
					for(k=0;k<(i-1);k++)
					{
						c = c*10;
					}
					b = (dummy[length-i]*c);
					str_ad = str_ad + b;
				}
				return str_ad;
			}
		}
		else if((getline_data >= 0x30) && (getline_data <= 0x46))
		{
			if(j<length)
			{
				s[j] = getline_data;
				UART_Send(UARTA1,s[j]);
				dummy[j] = s[j];
				j++;
			}
		}
		else if(j>length)
		{
			UART_Send(uartx,BELL_KEY);
			if(getline_data == ENTER_KEY)
			{
				return j;
			}
		}
		else
		{
			UART_Send(uartx,BELL_KEY);
		}
	}
}
//*****************************************************************************
//
//Close the Doxygen group.
//! @}
//
//*****************************************************************************
