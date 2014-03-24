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
//EUSCI_I2C_.c - Driver for the I2C Module.
//
//*****************************************************************************
#include "hw_types.h"
#include "debug.h"
#include "eusci_i2c.h"
#ifdef  __IAR_SYSTEMS_ICC__
#include "deprecated/IAR/msp430xgeneric.h"
#else
#include "msp430xgeneric.h"
#endif

//*****************************************************************************
//
//! Initializes the I2C Master block.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param selectClockSource is the clocksource.
//!         Valid values are
//!         \b EUSCI_I2C_CLOCKSOURCE_ACLK
//!         \b EUSCI_I2C_CLOCKSOURCE_SMCLK
//! \param i2cClk is the rate of the clock supplied to the I2C module.
//! \param dataRate set up for selecting data transfer rate.
//!         Valid values are
//!         \b EUSCI_I2C_SET_DATA_RATE_400KBPS
//!         \b EUSCI_I2C_SET_DATA_RATE_100KBPS
//! \param byteCounterThreshold sets threshold for automatic STOP or UCSTPIFG
//! \param autoSTOPGeneration sets up the STOP condition generation.
//!         Valid values are
//!         \b EUSCI_I2C_NO_AUTO_STOP
//!         \b EUSCI_I2C_SET_BYTECOUNT_THRESHOLD_FLAG
//!         \b EUSCI_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD
//!
//! This function initializes operation of the I2C Master block.  Upon
//! successful initialization of the I2C block, this function will have set the
//! bus speed for the master; however I2C module is still disabled till
//! EUSCI_I2C_enable is invoked
//!
//! If the parameter \e dataRate is EUSCI_I2C_SET_DATA_RATE_400KBPS, then the master
//! block will be set up to transfer data at 400 kbps; otherwise, it will be
//! set up to transfer data at 100 kbps.
//!
//! Modified bits are \b UCMST,UCMODE_3,\b UCSYNC of \b UCBxCTL0 register
//!                   \b UCSSELx, \b UCSWRST, of \b UCBxCTL1 register
//!                   \b UCBxBR0 and \b UCBxBR1 regsiters
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_masterInit (unsigned int baseAddress,
    unsigned char selectClockSource,
    unsigned long i2cClk,
    unsigned long dataRate,
    unsigned char byteCounterThreshold,
    unsigned char autoSTOPGeneration
    )
{
    unsigned int preScalarValue;

    ASSERT((EUSCI_I2C_CLOCKSOURCE_ACLK == selectClockSource) ||
        (EUSCI_I2C_CLOCKSOURCE_SMCLK == selectClockSource)
        );

    ASSERT((EUSCI_I2C_SET_DATA_RATE_400KBPS == dataRate) ||
        (EUSCI_I2C_SET_DATA_RATE_100KBPS == dataRate)
        );

    ASSERT((EUSCI_I2C_NO_AUTO_STOP == autoSTOPGeneration) ||
            (EUSCI_I2C_SET_BYTECOUNT_THRESHOLD_FLAG == autoSTOPGeneration) ||
            (EUSCI_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD == autoSTOPGeneration)
            );


    //Disable the USCI module and clears the other bits of control register
    HWREG(baseAddress + OFS_UCBxCTLW0) = UCSWRST;


    //Configure Automatic STOP condition generation
    HWREG(baseAddress + OFS_UCBxCTLW1) &= ~UCASTP_3;
    HWREG(baseAddress + OFS_UCBxCTLW1) |= autoSTOPGeneration;

    //Byte Count Threshold
    HWREG(baseAddress + OFS_UCBxTBCNT) = byteCounterThreshold;
    /*
     * Configure as I2C master mode.
     * UCMST = Master mode
     * UCMODE_3 = I2C mode
     * UCSYNC = Synchronous mode
     */
    HWREG(baseAddress + OFS_UCBxCTLW0) = UCMST + UCMODE_3 + UCSYNC;

    //Configure I2C clock source
    HWREG(baseAddress + OFS_UCBxCTLW0) |= (selectClockSource + UCSWRST );

    /*
     * Compute the clock divider that achieves the fastest speed less than or
     * equal to the desired speed.  The numerator is biased to favor a larger
     * clock divider so that the resulting clock is always less than or equal
     * to the desired clock, never greater.
     */
    preScalarValue = (unsigned short)(i2cClk / dataRate);
    HWREG(baseAddress + OFS_UCBxBRW) = preScalarValue;
}

//*****************************************************************************
//
//! Initializes the I2C Slave block.
//!
//! \param baseAddress is the base address of the I2C Slave module.
//! \param slaveAddress 7-bit slave address
//! \param slaveAddressOffset Own address Offset referred to- 'x' value of
//!		UCBxI2COAx. Valid values are \b EUSCI_I2C_OWN_ADDRESS_OFFSET0,
//!									 \b EUSCI_I2C_OWN_ADDRESS_OFFSET1,
//!     							 \b EUSCI_I2C_OWN_ADDRESS_OFFSET2,
//!     							 \b EUSCI_I2C_OWN_ADDRESS_OFFSET3
//! \param slaveOwnAddressEnable selects if the specified address is enabled
//!		or disabled. Valid values are \b EUSCI_I2C_OWN_ADDRESS_DISABLE,
//!									  \b EUSCI_I2C_OWN_ADDRESS_ENABLE
//!
//! This function initializes operation of the I2C as a Slave mode.  Upon
//! successful initialization of the I2C blocks, this function will have set
//! the slave address but the I2C module is still disabled till EUSCI_I2C_enable
//! is invoked.
//!
//! The parameter slaveAddress is the value that will be compared against the
//! slave address sent by an I2C master.
//! Modified bits are \b UCMODE_3, \b UCSYNC of \b UCBxCTL0 register
//!                   \b UCSWRST of \b UCBxCTL1 register
//!                   \b UCBxI2COA register
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_slaveInit (unsigned int baseAddress,
    unsigned char slaveAddress,
    unsigned char slaveAddressOffset,
    unsigned long slaveOwnAddressEnable
    )
{
	ASSERT((EUSCI_I2C_OWN_ADDRESS_OFFSET0 == slaveAddressOffset) ||
	            (EUSCI_I2C_OWN_ADDRESS_OFFSET1 == slaveAddressOffset) ||
	            (EUSCI_I2C_OWN_ADDRESS_OFFSET2 == slaveAddressOffset) ||
	            (EUSCI_I2C_OWN_ADDRESS_OFFSET3 == slaveAddressOffset)
	            );

    //Disable the USCI module
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCSWRST;

    //Configure I2C as Slave and Synchronous mode
    HWREG(baseAddress + OFS_UCBxCTLW0) = UCMODE_3 + UCSYNC;

    //Set up the slave address.
    HWREG(baseAddress + OFS_UCBxI2COA0 + slaveAddressOffset)
    								= slaveAddress + slaveOwnAddressEnable;
}

//*****************************************************************************
//
//! Enables the I2C block.
//!
//! \param baseAddress is the base address of the USCI I2C module.
//!
//! This will enable operation of the I2C block.
//! Modified bits are \b UCSWRST of \b UCBxCTL1 register.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_enable (unsigned int baseAddress)
{
    //Reset the UCSWRST bit to enable the USCI Module
    HWREG(baseAddress + OFS_UCBxCTLW0) &= ~(UCSWRST);
}

//*****************************************************************************
//
//! Disables the I2C block.
//!
//! \param baseAddress is the base address of the USCI I2C module.
//!
//! This will disable operation of the I2C block.
//! Modified bits are \b UCSWRST of \b UCBxCTL1 register.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_disable (unsigned int baseAddress)
{
    //Set the UCSWRST bit to disable the USCI Module
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCSWRST;
}

//*****************************************************************************
//
//! Sets the address that the I2C Master will place on the bus.
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param slaveAddress 7-bit slave address
//!
//! This function will set the address that the I2C Master will place on the
//! bus when initiating a transaction.
//! Modified register is  \b UCBxI2CSA register
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_setSlaveAddress (unsigned int baseAddress,
    unsigned char slaveAddress
    )
{
    //Set the address of the slave with which the master will communicate.
    HWREG(baseAddress + OFS_UCBxI2CSA) = (slaveAddress);
}

//*****************************************************************************
//
//! Sets the mode of the I2C device
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param receive indicates whether module is in transmit/receive mode
//!
//! When the receive parameter is set to EUSCI_I2C_TRANSMIT_MODE, the address will
//! indicate that the I2C module is in receive mode; otherwise, the I2C module
//! is in send mode. Valid values are
//!     \b EUSCI_I2C_TRANSMIT_MODE
//!     \b EUSCI_I2C_RECEIVE_MODE [Default value]
//! Modified bits are \b UCTR of \b UCBxCTL1 register
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_setMode (unsigned int baseAddress,
    unsigned char mode
    )
{
    ASSERT((EUSCI_I2C_TRANSMIT_MODE == mode) ||
        (EUSCI_I2C_RECEIVE_MODE == mode)
        );

    HWREG(baseAddress + OFS_UCBxCTLW0) &= ~EUSCI_I2C_TRANSMIT_MODE;
    HWREG(baseAddress + OFS_UCBxCTLW0) |= mode;
}

//*****************************************************************************
//
//! Transmits a byte from the I2C Module.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param transmitData data to be transmitted from the I2C module
//!
//! This function will place the supplied data into I2C trasmit data register
//! to start transmission
//! Modified bit is \b UCBxTXBUF register
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_slaveDataPut (unsigned int baseAddress,
    unsigned char transmitData
    )
{
    //Send single byte data.
    HWREG(baseAddress + OFS_UCBxTXBUF) = transmitData;
}

//*****************************************************************************
//
//! Receives a byte that has been sent to the I2C Module.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function reads a byte of data from the I2C receive data Register.
//!
//! \return Returns the byte received from by the I2C module, cast as an
//! unsigned char.
//! Modified bit is \b UCBxRXBUF register
//
//*****************************************************************************
unsigned char EUSCI_I2C_slaveDataGet (unsigned int baseAddress)
{
    //Read a byte.
    return (HWREG(baseAddress + OFS_UCBxRXBUF));
}

//*****************************************************************************
//
//! Indicates whether or not the I2C bus is busy.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function returns an indication of whether or not the I2C bus is
//! busy.This function checks the status of the bus via UCBBUSY bit in
//! UCBxSTAT register.
//!
//! \return Returns EUSCI_I2C_BUS_BUSY if the I2C Master is busy; otherwise, returns
//! EUSCI_I2C_BUS_NOT_BUSY.
//
//*****************************************************************************
unsigned char EUSCI_I2C_isBusBusy (unsigned int baseAddress)
{
    //Return the bus busy status.
    return (HWREG(baseAddress + OFS_UCBxSTATW) & UCBBUSY);
}


//*****************************************************************************
//
//! Indicates whether STOP got sent.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function returns an indication of whether or not STOP got sent
//! This function checks the status of the bus via UCTXSTP bit in
//! UCBxCTL1 register.
//!
//! \return Returns EUSCI_I2C_STOP_SEND_COMPLETE if the I2C Master is busy; otherwise, returns
//! EUSCI_I2C_SENDING_STOP.
//
//*****************************************************************************
unsigned char EUSCI_I2C_masterIsSTOPSent (unsigned int baseAddress)
{
    //Return the bus busy status.
    return (HWREG(baseAddress + OFS_UCBxCTL0) & UCTXSTP);
}
//*****************************************************************************
//
//! Enables individual I2C interrupt sources.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param interruptFlags is the bit mask of the interrupt sources to be enabled.
//!
//! Enables the indicated I2C interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The mask parameter is the logical OR of any of the following:
//!
//! - \b EUSCI_I2C_STOP_INTERRUPT - STOP condition interrupt
//! - \b EUSCI_I2C_START_INTERRUPT - START condition interrupt
//! - \b EUSCI_I2C_TRANSMIT_INTERRUPT0 - Transmit interrupt0
//! - \b EUSCI_I2C_TRANSMIT_INTERRUPT1 - Transmit interrupt1
//! - \b EUSCI_I2C_TRANSMIT_INTERRUPT2 - Transmit interrupt2
//! - \b EUSCI_I2C_TRANSMIT_INTERRUPT3 - Transmit interrupt3
//! - \b EUSCI_I2C_RECEIVE_INTERRUPT0 - Receive interrupt0
//! - \b EUSCI_I2C_RECEIVE_INTERRUPT1 - Receive interrupt1
//! - \b EUSCI_I2C_RECEIVE_INTERRUPT2 - Receive interrupt2
//! - \b EUSCI_I2C_RECEIVE_INTERRUPT3 - Receive interrupt3
//! - \b EUSCI_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//! - \b EUSCI_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost interrupt
//! - \b EUSCI_I2C_BIT9_POSITION_INTERRUPT - Bit position 9 interrupt enable
//! - \b EUSCI_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT - Clock low timeout interrupt enable
//! - \b EUSCI_I2C_BYTE_COUNTER_INTERRUPT - Byte counter interrupt enable
//!
//! Modified registers are UCBxIFG and OFS_UCBxIE.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_enableInterrupt (unsigned int baseAddress,
    unsigned int mask
    )
{
    ASSERT( 0x00 == ( mask & ~(EUSCI_I2C_STOP_INTERRUPT +
                               EUSCI_I2C_START_INTERRUPT +
                               EUSCI_I2C_NAK_INTERRUPT +
                               EUSCI_I2C_ARBITRATIONLOST_INTERRUPT +
                               EUSCI_I2C_BIT9_POSITION_INTERRUPT +
                               EUSCI_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT +
                               EUSCI_I2C_BYTE_COUNTER_INTERRUPT +
                               EUSCI_I2C_TRANSMIT_INTERRUPT0 +
                               EUSCI_I2C_TRANSMIT_INTERRUPT1 +
                               EUSCI_I2C_TRANSMIT_INTERRUPT2 +
                               EUSCI_I2C_TRANSMIT_INTERRUPT3 +
                               EUSCI_I2C_RECEIVE_INTERRUPT0 +
                               EUSCI_I2C_RECEIVE_INTERRUPT1 +
                               EUSCI_I2C_RECEIVE_INTERRUPT2 +
                               EUSCI_I2C_RECEIVE_INTERRUPT3
                               ))
                               );

    HWREG(baseAddress + OFS_UCBxIFG) &= ~(mask);

    //Enable the interrupt masked bit
    HWREG(baseAddress + OFS_UCBxIE) |= mask;
}

//*****************************************************************************
//
//! Disables individual I2C interrupt sources.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param mask is the bit mask of the interrupt sources to be
//! disabled.
//!
//! Disables the indicated I2C interrupt sources.  Only the sources that
//! are enabled can be reflected to the processor interrupt; disabled sources
//! have no effect on the processor.
//!
//! The mask parameter is the logical OR of any of the following:
//!
//! - \b EUSCI_I2C_STOP_INTERRUPT - STOP condition interrupt
//! - \b EUSCI_I2C_START_INTERRUPT - START condition interrupt
//! - \b EUSCI_I2C_TRANSMIT_INTERRUPT0 - Transmit interrupt0
//! - \b EUSCI_I2C_TRANSMIT_INTERRUPT1 - Transmit interrupt1
//! - \b EUSCI_I2C_TRANSMIT_INTERRUPT2 - Transmit interrupt2
//! - \b EUSCI_I2C_TRANSMIT_INTERRUPT3 - Transmit interrupt3
//! - \b EUSCI_I2C_RECEIVE_INTERRUPT0 - Receive interrupt0
//! - \b EUSCI_I2C_RECEIVE_INTERRUPT1 - Receive interrupt1
//! - \b EUSCI_I2C_RECEIVE_INTERRUPT2 - Receive interrupt2
//! - \b EUSCI_I2C_RECEIVE_INTERRUPT3 - Receive interrupt3
//! - \b EUSCI_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//! - \b EUSCI_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost interrupt
//! - \b EUSCI_I2C_BIT9_POSITION_INTERRUPT - Bit position 9 interrupt enable
//! - \b EUSCI_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT - Clock low timeout interrupt enable
//! - \b EUSCI_I2C_BYTE_COUNTER_INTERRUPT - Byte counter interrupt enable
//!
//! Modified register is \b UCBxIE.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_disableInterrupt (unsigned int baseAddress,
    unsigned int mask
    )
{
	ASSERT( 0x00 == ( mask & ~(EUSCI_I2C_STOP_INTERRUPT +
	                           EUSCI_I2C_START_INTERRUPT +
	                           EUSCI_I2C_NAK_INTERRUPT +
	                           EUSCI_I2C_ARBITRATIONLOST_INTERRUPT +
	                           EUSCI_I2C_BIT9_POSITION_INTERRUPT +
	                           EUSCI_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT +
	                           EUSCI_I2C_BYTE_COUNTER_INTERRUPT +
	                           EUSCI_I2C_TRANSMIT_INTERRUPT0 +
	                           EUSCI_I2C_TRANSMIT_INTERRUPT1 +
	                           EUSCI_I2C_TRANSMIT_INTERRUPT2 +
	                           EUSCI_I2C_TRANSMIT_INTERRUPT3 +
	                           EUSCI_I2C_RECEIVE_INTERRUPT0 +
	                           EUSCI_I2C_RECEIVE_INTERRUPT1 +
	                           EUSCI_I2C_RECEIVE_INTERRUPT2 +
	                           EUSCI_I2C_RECEIVE_INTERRUPT3
	                               ))
	                               );

    //Disable the interrupt masked bit
    HWREG(baseAddress + OFS_UCBxIE) &= ~(mask);
}

//*****************************************************************************
//
//! Clears I2C interrupt sources.
//!
//! \param baseAddress is the base address of the I2C Slave module.
//! \param mask is a bit mask of the interrupt sources to be cleared.
//!
//! The I2C interrupt source is cleared, so that it no longer asserts.
//! The highest interrupt flag is automatically cleared when an interrupt vector
//! generator is used.
//!
//! The mask parameter has the same definition as the mask
//! parameter to EUSCI_I2C_enableInterrupt().
//!
//! Modified register is \b UCBxIFG.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_clearInterruptFlag (unsigned int baseAddress,
    unsigned int mask
    )
{
	ASSERT( 0x00 == ( mask & ~(EUSCI_I2C_STOP_INTERRUPT +
	                           EUSCI_I2C_START_INTERRUPT +
	                           EUSCI_I2C_NAK_INTERRUPT +
	                           EUSCI_I2C_ARBITRATIONLOST_INTERRUPT +
	                           EUSCI_I2C_BIT9_POSITION_INTERRUPT +
	                           EUSCI_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT +
	                           EUSCI_I2C_BYTE_COUNTER_INTERRUPT +
	                           EUSCI_I2C_TRANSMIT_INTERRUPT0 +
	                           EUSCI_I2C_TRANSMIT_INTERRUPT1 +
	                           EUSCI_I2C_TRANSMIT_INTERRUPT2 +
	                           EUSCI_I2C_TRANSMIT_INTERRUPT3 +
	                           EUSCI_I2C_RECEIVE_INTERRUPT0 +
	                           EUSCI_I2C_RECEIVE_INTERRUPT1 +
	                           EUSCI_I2C_RECEIVE_INTERRUPT2 +
	                           EUSCI_I2C_RECEIVE_INTERRUPT3
	                               ))
	                               );
    //Clear the I2C interrupt source.
    HWREG(baseAddress + OFS_UCBxIFG) &= ~(mask);
}

//*****************************************************************************
//
//! Gets the current I2C interrupt status.
//!
//! \param baseAddress is the base address of the I2C module.
//! \param mask is the masked interrupt flag status to be returned.
//!
//! This returns the interrupt status for the I2C  module based on which
//! flag is passed. mask parameter can be either any of the following
//! selection.
//!
//! - \b EUSCI_I2C_STOP_INTERRUPT - STOP condition interrupt
//! - \b EUSCI_I2C_START_INTERRUPT - START condition interrupt
//! - \b EUSCI_I2C_TRANSMIT_INTERRUPT0 - Transmit interrupt0
//! - \b EUSCI_I2C_TRANSMIT_INTERRUPT1 - Transmit interrupt1
//! - \b EUSCI_I2C_TRANSMIT_INTERRUPT2 - Transmit interrupt2
//! - \b EUSCI_I2C_TRANSMIT_INTERRUPT3 - Transmit interrupt3
//! - \b EUSCI_I2C_RECEIVE_INTERRUPT0 - Receive interrupt0
//! - \b EUSCI_I2C_RECEIVE_INTERRUPT1 - Receive interrupt1
//! - \b EUSCI_I2C_RECEIVE_INTERRUPT2 - Receive interrupt2
//! - \b EUSCI_I2C_RECEIVE_INTERRUPT3 - Receive interrupt3
//! - \b EUSCI_I2C_NAK_INTERRUPT - Not-acknowledge interrupt
//! - \b EUSCI_I2C_ARBITRATIONLOST_INTERRUPT - Arbitration lost interrupt
//! - \b EUSCI_I2C_BIT9_POSITION_INTERRUPT - Bit position 9 interrupt enable
//! - \b EUSCI_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT - Clock low timeout interrupt enable
//! - \b EUSCI_I2C_BYTE_COUNTER_INTERRUPT - Byte counter interrupt enable
//!
//! Modified register is \b UCBxIFG.
//!
//! \returns the masked status of the interrupt flag
//
//*****************************************************************************
unsigned char EUSCI_I2C_getInterruptStatus (unsigned int baseAddress,
    unsigned int mask
    )
{
	ASSERT( 0x00 == ( mask & ~(EUSCI_I2C_STOP_INTERRUPT +
		                           EUSCI_I2C_START_INTERRUPT +
		                           EUSCI_I2C_NAK_INTERRUPT +
		                           EUSCI_I2C_ARBITRATIONLOST_INTERRUPT +
		                           EUSCI_I2C_BIT9_POSITION_INTERRUPT +
		                           EUSCI_I2C_CLOCK_LOW_TIMEOUT_INTERRUPT +
		                           EUSCI_I2C_BYTE_COUNTER_INTERRUPT +
		                           EUSCI_I2C_TRANSMIT_INTERRUPT0 +
		                           EUSCI_I2C_TRANSMIT_INTERRUPT1 +
		                           EUSCI_I2C_TRANSMIT_INTERRUPT2 +
		                           EUSCI_I2C_TRANSMIT_INTERRUPT3 +
		                           EUSCI_I2C_RECEIVE_INTERRUPT0 +
		                           EUSCI_I2C_RECEIVE_INTERRUPT1 +
		                           EUSCI_I2C_RECEIVE_INTERRUPT2 +
		                           EUSCI_I2C_RECEIVE_INTERRUPT3
		                               ))
		                               );
    //Return the interrupt status of the request masked bit.
    return (HWREG(baseAddress + OFS_UCBxIFG) & mask);
}

//*****************************************************************************
//
//! Does single byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the data byte to be transmitted
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the byte to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCBxIE, \b UCBxCTL1, \b UCBxIFG, \b UCBxTXBUF,
//! \b UCBxIE
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
void EUSCI_I2C_masterSendSingleByte (unsigned int baseAddress,
    unsigned char txData
    )
{
    //Store current TXIE status
    unsigned int txieStatus = HWREG(baseAddress + OFS_UCBxIE) & UCTXIE;

    //Disable transmit interrupt enable
    HWREG(baseAddress + OFS_UCBxIE) &= ~(UCTXIE);

    //Send start condition.
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTR + UCTXSTT;

    //Poll for transmit interrupt flag.
    while (!(HWREG(baseAddress + OFS_UCBxIFG) & UCTXIFG)) ;

    //Send single byte data.
    HWREG(baseAddress + OFS_UCBxTXBUF) = txData;

    //Poll for transmit interrupt flag.
    while (!(HWREG(baseAddress + OFS_UCBxIFG) & UCTXIFG)) ;

    //Send stop condition.
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;

    //Clear transmit interrupt flag before enabling interrupt again
    HWREG(baseAddress + OFS_UCBxIFG) &= ~(UCTXIFG);

    //Reinstate transmit interrupt enable
    HWREG(baseAddress + OFS_UCBxIE) |= txieStatus;
}

//*****************************************************************************
//
//! Does single byte transmission from Master to Slave with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the data byte to be transmitted
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the byte to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCBxIE, \b UCBxCTL1, \b UCBxIFG, \b UCBxTXBUF,
//! \b UCBxIE
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short EUSCI_I2C_masterSendSingleByteWithTimeout (unsigned int baseAddress,
    unsigned char txData,
    unsigned long timeout
    )
{
	// Creating variable for second timeout scenario
	unsigned long timeout2 = timeout;
	
    //Store current TXIE status
    unsigned int txieStatus = HWREG(baseAddress + OFS_UCBxIE) & UCTXIE;

    //Disable transmit interrupt enable
    HWREG(baseAddress + OFS_UCBxIE) &= ~(UCTXIE);

    //Send start condition.
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTR + UCTXSTT;

    //Poll for transmit interrupt flag.
    while ((!(HWREG(baseAddress + OFS_UCBxIFG) & UCTXIFG)) & timeout--);

 	//Check if transfer timed out
    if (timeout == 0){
    	return (STATUS_FAIL);
    }

    //Send single byte data.
    HWREG(baseAddress + OFS_UCBxTXBUF) = txData;

    //Poll for transmit interrupt flag.
    while ((!(HWREG(baseAddress + OFS_UCBxIFG) & UCTXIFG)) & timeout2--);

    //Check if transfer timed out
    if (timeout2 == 0){
    	return (STATUS_FAIL);
    }
    
    //Send stop condition.
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;

    //Clear transmit interrupt flag before enabling interrupt again
    HWREG(baseAddress + OFS_UCBxIFG) &= ~(UCTXIFG);

    //Reinstate transmit interrupt enable
    HWREG(baseAddress + OFS_UCBxIE) |= txieStatus;
    
    return (STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Starts multi-byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the first data byte to be transmitted
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the first data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCBxIE, \b UCBxCTL1, \b UCBxIFG, \b UCBxTXBUF,
//! \b UCBxIE
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_masterMultiByteSendStart (unsigned int baseAddress,
    unsigned char txData
    )
{
    //Store current transmit interrupt enable
    unsigned int txieStatus = HWREG(baseAddress + OFS_UCBxIE) & UCTXIE;


    //Disable transmit interrupt enable
    HWREG(baseAddress + OFS_UCBxIE) &= ~(UCTXIE);

    //Send start condition.
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTR +  UCTXSTT;

    //Poll for transmit interrupt flag.
    while (!(HWREG(baseAddress + OFS_UCBxIFG) & UCTXIFG)) ;

    //Send single byte data.
    HWREG(baseAddress + OFS_UCBxTXBUF) = txData;

    //Reinstate transmit interrupt enable
    HWREG(baseAddress + OFS_UCBxIE) |= txieStatus;
}

//*****************************************************************************
//
//! Starts multi-byte transmission from Master to Slave with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the first data byte to be transmitted
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to send a single byte.
//! This function
//! - Sends START
//! - Transmits the first data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCBxIE, \b UCBxCTL1, \b UCBxIFG, \b UCBxTXBUF,
//! \b UCBxIE
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short EUSCI_I2C_masterMultiByteSendStartWithTimeout (unsigned int baseAddress,
    unsigned char txData,
    unsigned long timeout
    )
{
    //Store current transmit interrupt enable
    unsigned int txieStatus = HWREG(baseAddress + OFS_UCBxIE) & UCTXIE;

    //Disable transmit interrupt enable
    HWREG(baseAddress + OFS_UCBxIE) &= ~(UCTXIE);

    //Send start condition.
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTR +  UCTXSTT;

    //Poll for transmit interrupt flag.
    while ((!(HWREG(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && timeout--);
    
    //Check if transfer timed out
    if (timeout == 0){
    	return (STATUS_FAIL);
    }

    //Send single byte data.
    HWREG(baseAddress + OFS_UCBxTXBUF) = txData;

    //Reinstate transmit interrupt enable
    HWREG(baseAddress + OFS_UCBxIE) |= txieStatus;
    
    return(STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Continues multi-byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the next data byte to be transmitted
//!
//! This function is used by the Master module continue each byte of a
//! multi-byte trasmission. This function
//! - Transmits each data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCBxTXBUF
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_masterMultiByteSendNext (unsigned int baseAddress,
    unsigned char txData
    )
{
    //If interrupts are not used, poll for flags
    if (!(HWREG(baseAddress + OFS_UCBxIE) & UCTXIE)){
        //Poll for transmit interrupt flag.
        while (!(HWREG(baseAddress + OFS_UCBxIFG) & UCTXIFG)) ;
    }

    //Send single byte data.
    HWREG(baseAddress + OFS_UCBxTXBUF) = txData;
}

//*****************************************************************************
//
//! Continues multi-byte transmission from Master to Slave with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the next data byte to be transmitted
//!
//! This function is used by the Master module continue each byte of a
//! multi-byte trasmission. This function
//! - Transmits each data byte of a multi-byte transmission to the Slave
//!
//! Modified registers are \b UCBxTXBUF
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short EUSCI_I2C_masterMultiByteSendNextWithTimeout (unsigned int baseAddress,
    unsigned char txData,
    unsigned long timeout
    )
{
    //If interrupts are not used, poll for flags
    if (!(HWREG(baseAddress + OFS_UCBxIE) & UCTXIE)){
        //Poll for transmit interrupt flag.
        while ((!(HWREG(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && timeout--);
    
    	//Check if transfer timed out
        if (timeout == 0){
        	return (STATUS_FAIL);
        }
    }

    //Send single byte data.
    HWREG(baseAddress + OFS_UCBxTXBUF) = txData;
    
    return(STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Finishes multi-byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the last data byte to be transmitted in a multi-byte
//! tramsission
//!
//! This function is used by the Master module to send the last byte and STOP.
//! This function
//! - Transmits the last data byte of a multi-byte transmission to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCBxTXBUF and \b UCBxCTL1.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_masterMultiByteSendFinish (unsigned int baseAddress,
    unsigned char txData
    )
{
    //If interrupts are not used, poll for flags
    if (!(HWREG(baseAddress + OFS_UCBxIE) & UCTXIE)){
        //Poll for transmit interrupt flag.
        while (!(HWREG(baseAddress + OFS_UCBxIFG) & UCTXIFG)) ;
    }

    //Send single byte data.
    HWREG(baseAddress + OFS_UCBxTXBUF) = txData;

    //Poll for transmit interrupt flag.
    while (!(HWREG(baseAddress + OFS_UCBxIFG) & UCTXIFG)) ;

    //Send stop condition.
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;
}

//*****************************************************************************
//
//! Finishes multi-byte transmission from Master to Slave with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is the last data byte to be transmitted in a multi-byte
//! tramsission
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to send the last byte and STOP.
//! This function
//! - Transmits the last data byte of a multi-byte transmission to the Slave
//! - Sends STOP
//!
//! Modified registers are \b UCBxTXBUF and \b UCBxCTL1.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short EUSCI_I2C_masterMultiByteSendFinishWithTimeout (unsigned int baseAddress,
    unsigned char txData,
    unsigned long timeout
    )
{
	unsigned long timeout2 = timeout;
	
	ASSERT(timeout == 0);
	
    //If interrupts are not used, poll for flags
    if (!(HWREG(baseAddress + OFS_UCBxIE) & UCTXIE)){
        //Poll for transmit interrupt flag.
        while ((!(HWREG(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && timeout--) ;
        
        //Check if transfer timed out
        if (timeout == 0){
        	return (STATUS_FAIL);
        }
    }

    //Send single byte data.
    HWREG(baseAddress + OFS_UCBxTXBUF) = txData;

    //Poll for transmit interrupt flag.
    while ((!(HWREG(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && timeout2--) ;

	//Check if transfer timed out
    if (timeout2 == 0){
    	return (STATUS_FAIL);
    }
    
    //Send stop condition.
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;
    
    return(STATUS_SUCCESS);
}

//*****************************************************************************
//
//! This function is used by the Master module to initiate START
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module to initiate STOP
//!
//! Modified bits are UCTXSTT bit of UCBxCTLW0.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_masterSendStart (unsigned int baseAddress)
{
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTXSTT;
}

//*****************************************************************************
//
//! Send STOP byte at the end of a multi-byte transmission from Master to Slave
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module send STOP at the end of a
//! multi-byte trasmission
//!
//! This function
//! - Send a STOP after current transmission is complete
//!
//! Modified bits are \b UCTXSTP bit of \b UCBxCTL1.
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_masterMultiByteSendStop (unsigned int baseAddress)
{
    //If interrupts are not used, poll for flags
    if (!(HWREG(baseAddress + OFS_UCBxIE) & UCTXIE)){
        //Poll for transmit interrupt flag.
        while (!(HWREG(baseAddress + OFS_UCBxIFG) & UCTXIFG)) ;
    }

    //Send stop condition.
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;
}

//*****************************************************************************
//
//! Send STOP byte at the end of a multi-byte transmission from Master to Slave 
//! with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module send STOP at the end of a
//! multi-byte trasmission
//!
//! This function
//! - Send a STOP after current transmission is complete
//!
//! Modified bits are \b UCTXSTP bit of \b UCBxCTL1.
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short EUSCI_I2C_masterMultiByteSendStopWithTimeout (unsigned int baseAddress,
	unsigned long timeout)
{
    //If interrupts are not used, poll for flags
    if (!(HWREG(baseAddress + OFS_UCBxIE) & UCTXIE)){
        //Poll for transmit interrupt flag.
        while ((!(HWREG(baseAddress + OFS_UCBxIFG) & UCTXIFG)) && timeout--) ;
        
		//Check if transfer timed out
        if (timeout == 0){
        	return (STATUS_FAIL);
        }
    }

    //Send stop condition.
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;
    
    return (STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Starts reception at the Master end
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module initiate reception of a single
//! byte. This function
//! - Sends START
//!
//! Modified bits are \b UCTXSTT bit of \b UCBxCTL1.
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_masterReceiveStart (unsigned int baseAddress)
{
    //Set USCI in Receive mode
    HWREG(baseAddress + OFS_UCBxCTLW0) &= ~UCTR;
    //Send start
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTXSTT;
}

//*****************************************************************************
//
//! Starts multi-byte reception at the Master end one byte at a time
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module to receive each byte of a
//! multi-byte reception
//! This function reads currently received byte
//!
//! Modified register is \b UCBxRXBUF.
//! \return Received byte at Master end.
//
//*****************************************************************************
unsigned char EUSCI_I2C_masterMultiByteReceiveNext (unsigned int baseAddress)
{
    return (HWREG(baseAddress + OFS_UCBxRXBUF));
}

//*****************************************************************************
//
//! Finishes multi-byte reception at the Master end
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module to initiate completion of a
//! multi-byte reception
//! This function
//! - Receives the current byte and initiates the STOP from Master to Slave
//!
//! Modified bits are \b UCTXSTP bit of \b UCBxCTL1.
//!
//! \return Received byte at Master end.
//
//*****************************************************************************
unsigned char EUSCI_I2C_masterMultiByteReceiveFinish (unsigned int baseAddress)
{
    //Send stop condition.
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;

    //Wait for Stop to finish
    while (HWREG(baseAddress + OFS_UCBxCTLW0) & UCTXSTP)

    // Wait for RX buffer
    while (!(HWREG(baseAddress + OFS_UCBxIFG) & UCRXIFG)) ;

    //Capture data from receive buffer after setting stop bit due to
    //MSP430 I2C critical timing.
    return (HWREG(baseAddress + OFS_UCBxRXBUF));
}

//*****************************************************************************
//
//! Finishes multi-byte reception at the Master end with timeout
//!
//! \param baseAddress is the base address of the I2C Master module.
//! \param txData is a pointer to the location to store the recieved byte at 
//!     master end
//! \param timeout is the amount of time to wait until giving up
//!
//! This function is used by the Master module to initiate completion of a
//! multi-byte reception
//! This function
//! - Receives the current byte and initiates the STOP from Master to Slave
//!
//! Modified bits are \b UCTXSTP bit of \b UCBxCTL1.
//!
//! \return STATUS_SUCCESS or STATUS_FAILURE of the transmission process.
//
//*****************************************************************************
unsigned short EUSCI_I2C_masterMultiByteReceiveFinishWithTimeout (unsigned int baseAddress,
	unsigned char *txData,
	unsigned long timeout
	)
{
	unsigned long timeout2 = timeout;
		
    //Send stop condition.
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;

    //Wait for Stop to finish
    while ((HWREG(baseAddress + OFS_UCBxCTLW0) & UCTXSTP) && timeout--);
    
    //Check if transfer timed out
    if (timeout == 0){
    	return (STATUS_FAIL);
    }

    // Wait for RX buffer
    while ((!(HWREG(baseAddress + OFS_UCBxIFG) & UCRXIFG)) && timeout2--);
    
    //Check if transfer timed out
    if (timeout2 == 0){
    	return (STATUS_FAIL);
    }

    //Capture data from receive buffer after setting stop bit due to
    //MSP430 I2C critical timing.
    *txData = (HWREGB(baseAddress + OFS_UCBxRXBUF));
     
    return (STATUS_SUCCESS);
}

//*****************************************************************************
//
//! Sends the STOP at the end of a multi-byte reception at the Master end
//!
//! \param baseAddress is the base address of the I2C Master module.
//!
//! This function is used by the Master module to initiate STOP
//!
//! Modified bits are UCTXSTP bit of UCBxCTL1.
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_masterMultiByteReceiveStop (unsigned int baseAddress)
{
    //Send stop condition.
    HWREG(baseAddress + OFS_UCBxCTLW0) |= UCTXSTP;
}

//*****************************************************************************
//
//! Enables Multi Master Mode
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! At the end of this function, the I2C module is still disabled till
//! EUSCI_I2C_enable is invoked
//!
//! Modified bits are \b UCSWRST of \b OFS_UCBxCTLW0, \b UCMM bit of
//!	\b UCBxCTLW0
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_enableMultiMasterMode(unsigned int baseAddress)
{
	HWREG(baseAddress + OFS_UCBxCTLW0) |= UCSWRST;
	HWREG(baseAddress + OFS_UCBxCTLW0) |= UCMM;
}

//*****************************************************************************
//
//! Disables Multi Master Mode
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! At the end of this function, the I2C module is still disabled till
//! EUSCI_I2C_enable is invoked
//!
//! Modified bits are \b UCSWRST of \b OFS_UCBxCTLW0, \b UCMM bit of
//!	\b UCBxCTLW0
//!
//! \return None.
//
//*****************************************************************************
void EUSCI_I2C_disableMultiMasterMode(unsigned int baseAddress)
{
	
	HWREG(baseAddress + OFS_UCBxCTLW0) |= UCSWRST;
	HWREG(baseAddress + OFS_UCBxCTLW0) &= ~UCMM;
}

//*****************************************************************************
//
//! Receives a byte that has been sent to the I2C Master Module.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! This function reads a byte of data from the I2C receive data Register.
//!
//! \return Returns the byte received from by the I2C module, cast as an
//! unsigned char.
//
//*****************************************************************************
unsigned char EUSCI_I2C_masterSingleReceive (unsigned int baseAddress)
{
    //Read a byte.
    return (HWREG(baseAddress + OFS_UCBxRXBUF));
}

//*****************************************************************************
//
//! Returns the address of the RX Buffer of the I2C for the DMA module.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! Returns the address of the I2C RX Buffer. This can be used in conjunction
//! with the DMA to store the received data directly to memory.
//!
//! \return NONE
//
//*****************************************************************************
unsigned long EUSCI_I2C_getReceiveBufferAddressForDMA (unsigned int baseAddress)
{
    return ( baseAddress + OFS_UCBxRXBUF );
}

//*****************************************************************************
//
//! Returns the address of the TX Buffer of the I2C for the DMA module.
//!
//! \param baseAddress is the base address of the I2C module.
//!
//! Returns the address of the I2C TX Buffer. This can be used in conjunction
//! with the DMA to obtain transmitted data directly from memory.
//!
//! \return NONE
//
//*****************************************************************************
unsigned long EUSCI_I2C_getTransmitBufferAddressForDMA (unsigned int baseAddress)
{
    return ( baseAddress + OFS_UCBxTXBUF );
}

/*****************************************************************//**
* @brief        		Initialize SPI Module.
* @param[in]        	NONE.
* @return           	None
**********************************************************************/
void I2C_Init()
{
	// Set P2.0 and P2.1 as I2C Peripheral pins
	GPIO_setAsPeripheralModuleFunctionInputPin(
					GPIO_PORT_P2,
					GPIO_PIN0 + GPIO_PIN1
					);

	// Initialize Master
   EUSCI_I2C_masterInit(EUSCI_B0_BASE,
				   EUSCI_I2C_CLOCKSOURCE_SMCLK,
				   UCS_getSMCLK(UCS_BASE),
				   EUSCI_I2C_SET_DATA_RATE_100KBPS,
				   1,
				   EUSCI_I2C_NO_AUTO_STOP
				   );

   // Specify slave address
   EUSCI_I2C_setSlaveAddress(EUSCI_B0_BASE,
	   0x50
	   );

   // Enabling I2C Mode
   EUSCI_I2C_enable(EUSCI_B0_BASE);
}

/*******************************************************************************//**
* @brief        		Write single byte Data to I2C EEPROM at definite address.
* @param[in]        	Start Address , Data.
* @return           	NONE.
***********************************************************************************/
void WriteDataI2C(unsigned int address, unsigned char data)
{
	 EUSCI_I2C_setMode(EUSCI_B0_BASE, EUSCI_I2C_TRANSMIT_MODE);
	 EUSCI_I2C_masterMultiByteSendStart(EUSCI_B0_BASE,(address>>8));
	 Delay_Ms(50);
	 EUSCI_I2C_masterMultiByteSendNext(EUSCI_B0_BASE,(address & 0xFF));
	 Delay_Ms(50);
	 EUSCI_I2C_masterMultiByteSendFinish(EUSCI_B0_BASE,data);
	 Delay_Ms(100);

}

/*******************************************************************************//**
* @brief        		Read single byte Data from I2C EEPROM from definite address.
* @param[in]        	Start Address.
* @return           	Read character will be returned to char variable.
***********************************************************************************/
unsigned char ReadDataI2C(unsigned int address)
{

	EUSCI_I2C_masterMultiByteSendStart(EUSCI_B0_BASE,(address>>8));
	Delay_Ms(50);
	EUSCI_I2C_masterMultiByteSendNext(EUSCI_B0_BASE,(address>>8));
	Delay_Ms(50);
	EUSCI_I2C_masterMultiByteSendFinish(EUSCI_B0_BASE,address);
	Delay_Ms(100);
	EUSCI_I2C_setMode(EUSCI_B0_BASE, EUSCI_I2C_RECEIVE_MODE);

	EUSCI_I2C_masterReceiveStart(EUSCI_B0_BASE);
	Delay_Ms(50);
  	EUSCI_I2C_masterMultiByteReceiveStop(EUSCI_B0_BASE);
  	while(!EUSCI_I2C_getInterruptStatus(EUSCI_B0_BASE,EUSCI_I2C_RECEIVE_INTERRUPT0));
    return EUSCI_I2C_masterSingleReceive(EUSCI_B0_BASE);
}

/*******************************************************************************//**
* @brief        		Write String of Data from I2C EEPROM to definite address.
* @param[in]        	Start Address , Char Array of data.
* @return           	NONE.
***********************************************************************************/
void WriteDataStringI2C(unsigned int address, unsigned char data[])
{
				unsigned int i = 0;
				EUSCI_I2C_setMode(EUSCI_B0_BASE, EUSCI_I2C_TRANSMIT_MODE);
				EUSCI_I2C_masterMultiByteSendStart(EUSCI_B0_BASE,(address>>8));

				EUSCI_I2C_masterMultiByteSendNext(EUSCI_B0_BASE,(address & 0xFF));

				do
				{
				EUSCI_I2C_masterMultiByteSendNext(EUSCI_B0_BASE,data[pankil]);

				i++;
				pankil++;
				i2c_eeprom_add++;
				if((i > 62) || (i2c_eeprom_add%64 ==0 && i!=0))
				{
					goto PAGE_END;
				}
				}while (data[pankil] != '\0');
				pankil = 0;
PAGE_END:		EUSCI_I2C_masterMultiByteSendFinish(EUSCI_B0_BASE,data[pankil]);
}

/*******************************************************************************//**
* @brief        		Read String of Data from I2C EEPROM from definite address.
* @param[in]        	Start Address , Char Array , Length.
* @return           	NONE.
***********************************************************************************/
void ReadDataStringI2C(unsigned int address, unsigned char*rxd, unsigned int length)
{

    EUSCI_I2C_masterMultiByteSendStart(EUSCI_B0_BASE,(address>>8));
    Delay_Ms(1);
    EUSCI_I2C_masterMultiByteSendNext(EUSCI_B0_BASE,(address>>8));
    Delay_Ms(1);
    EUSCI_I2C_masterMultiByteSendFinish(EUSCI_B0_BASE,address);
    Delay_Ms(1);
    EUSCI_I2C_setMode(EUSCI_B0_BASE, EUSCI_I2C_RECEIVE_MODE);
    EUSCI_I2C_masterReceiveStart(EUSCI_B0_BASE);
    while(length--)
    {
    while(!EUSCI_I2C_getInterruptStatus(EUSCI_B0_BASE,EUSCI_I2C_RECEIVE_INTERRUPT0));
    Delay_Ms(1);
    *rxd=EUSCI_I2C_masterMultiByteReceiveNext (EUSCI_B0_BASE);
     rxd++;
    }
    *rxd='\0';
    EUSCI_I2C_masterMultiByteReceiveStop(EUSCI_B0_BASE);
}

/************************************************************************//**
* @brief        		Display I2C EEPROM Data on UART Port Selected.
* @param[in]        	Start Address , End Address.
* @return           	NONE.
****************************************************************************/
void Display_I2C_Data(unsigned int start_add,unsigned int end_add)
{
	unsigned char RXData[270];
	unsigned int i;
	unsigned int page;
	unsigned int page_req = 'N';

	while(end_add >= start_add)
	{
		while(page_req != 'N');
		{
			printf(UARTA1,"\x1b[2J");
			ReadDataStringI2C(start_add,RXData,256);						// Reading Data From EEPROM
			for(i=0;i<256;i++)
			{
				if(end_add >= start_add)
				{
					if(i%16 == 0)
					{
						printf(UARTA1,"\n\r");
						printf(UARTA1,"%x04\t",start_add);
					}
				}
				else
				{
					Print_Esc_Menu();
					return;
				}
				printf(UARTA1,"%x02  ",RXData[i]);
				start_add++;
			}
			page_req = 0;
			if(end_add >= start_add)
			{
				printf(UARTA1,"\n\rCurrent Page Displayed Correctly\n\rPress N for Next Page\n\rPRESS ESC. KEY TO BACK TO PREVIOUS MENU");
again:			page_req = getche(UARTA1,BLOCKING);
				if(page_req == ESC_KEY)
				{
					return;
				}
				else if(page_req != 'N' && page_req != ESC_KEY)
				{
					printf(UARTA1,"\n\rPlease Enter Valid Input\n\rPress N for Next Page\n\rPRESS ESC. KEY TO BACK TO PREVIOUS MENU");
					goto again;
				}

				start_add++;
			}
		}
	}
}

/**********************************************************************//**
* @brief        		Compare String given as Input.
* @param[in]        	Two Strings that needs to be compared.
* @return           	Return 1 if string matched and 0 if not matched.
**************************************************************************/
int my_strCmp (unsigned char *s,unsigned char *t)
{
	while (*t!='\0')
	{
		if (*t==*s )
		{
			s++;
			t++;
		}
		else
		{
			return(0);
		}
	}
	return(1);
}

//*****************************************************************************
//
//Close the Doxygen group.
//! @}
//
//*****************************************************************************
