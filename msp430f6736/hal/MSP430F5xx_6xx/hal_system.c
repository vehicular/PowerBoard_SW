/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
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

//#############################################################################
//
//! \file   hal_system.c
//!
//! \brief  Hardware Abstraction Layer for MSP system including clocks,
//!         watchdog, and GPIOs
//!
//!  Group:          MSP Software Development
//!  Target Device:  MSP430F6736
//!
//!  (C) Copyright 2017, Texas Instruments, Inc.
//#############################################################################

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "hal.h"

//
// Local function prototypes
//
static void hal_system_ClockInit(uint32_t mclkFreq);
static void hal_system_GPIOInit(void);

//
// Function declarations
//
void hal_system_Init(void)
{
    uint8_t i;

    // Configure all the phase GPIOs
    for(i=0;i<gEmSWConfig.systemConfig->pulseArrayLength;i++)
    {
        GPIO_setAsOutputPin(
                gEmSWConfig.systemConfig->pulseArray[i].activePulseGpioPort,
                gEmSWConfig.systemConfig->pulseArray[i].activePulseGpioPin
        );

        GPIO_setAsOutputPin(
                gEmSWConfig.systemConfig->pulseArray[i].reactivePulseGpioPort,
                gEmSWConfig.systemConfig->pulseArray[i].reactivePulseGpioPin
        );

        GPIO_setOutputHighOnPin(gEmSWConfig.systemConfig->pulseArray[i].activePulseGpioPort,
                                gEmSWConfig.systemConfig->pulseArray[i].activePulseGpioPin);

        GPIO_setOutputHighOnPin(gEmSWConfig.systemConfig->pulseArray[i].reactivePulseGpioPort,
                                gEmSWConfig.systemConfig->pulseArray[i].reactivePulseGpioPin);
    }

    hal_system_ClockInit(EM_SMCLK_FREQ_IN_HZ);
    hal_system_GPIOInit();
}

//*****************************************************************************
//
//! Initializes the system clocks
//!
//! This function initializes crystals, DCO and ACLK,SMCLK,MCLK
//!
//! \return none
//
// *****************************************************************************
static void hal_system_ClockInit(uint32_t mclkFreq)
{
    PMM_setVCore(PMM_CORE_LEVEL_3);

    UCS_turnOnLFXT1(UCS_XT1_DRIVE_0, UCS_XCAP_0);

    UCS_initClockSignal(
       UCS_FLLREF,
       UCS_XT1CLK_SELECT,
       UCS_CLOCK_DIVIDER_1);

    UCS_initClockSignal(
       UCS_ACLK,
       UCS_XT1CLK_SELECT,
       UCS_CLOCK_DIVIDER_1);

    UCS_initFLLSettle(
        mclkFreq/1000,
        mclkFreq/32768);
}

//
// Local Function declarations
//
//*****************************************************************************
//
//! Initializes GPIOs
//!
//! \return none
//
// *****************************************************************************
static void hal_system_GPIOInit(void)
{
    /* P1.2: UCA0 UART RX & P1.3: UCA0 UART TX  */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                                               GPIO_PIN2 | GPIO_PIN3);
}
