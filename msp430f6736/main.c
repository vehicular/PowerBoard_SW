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
//! \file   main.c
//!
//! \brief  Energy Measurement demo
//!
//!  Group:          MSP Embedded Software Development
//!  Target Device:  MSP430F6736
//!
//!  (C) Copyright 2017, Texas Instruments, Inc.
//#############################################################################
// TI Release: MSP430_ENERGY_MEASUREMENT 
// Release Date: 10Jul2018
//#############################################################################

#include <stdint.h>
#include "hal.h"
#include "hmi.h"
#include "EMLibGUIApp.h"
#include "EM_userConfig.h"

/*** FUNCTIONS ***/
static void MSP_GUI_System_setup(void);

//*****************************************************************************
//
//! In this software main will call a few initialization routines and then it
//!  will jump to the main loop for the Energy Measurement Library.
//!
//! \return none
//
// *****************************************************************************
void main(void)
{
	MSP_GUI_System_setup();
	
	// Initialize the EM Lib to default settings and start conversions
	EMLibGUIApp_Init();
	
	// This is the Design Centers Main Loop
	EMLibGUIApp_Engine();
}

//*****************************************************************************
//
//! This Function Initializes all the ports on the MCU 
//!
//! \return none
//
// *****************************************************************************
static void MSP_GUI_System_setup(void)
{
	// Stop WDT
    WDTCTL = WDTPW + WDTHOLD;
	
    // Initializes the basic functionality of the system
    hal_system_Init();

    // Initialize interfaces to user including GUI
    HMI_Init();
}
