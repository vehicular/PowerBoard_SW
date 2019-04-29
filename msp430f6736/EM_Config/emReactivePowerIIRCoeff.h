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
//! \file   emReactivePowerIIRCoeff.h
//!
//! \brief  This file contains EM Reactive Power Coefficients
//!         
//
//!  Group:          MSP Embedded Software Development
//!  Target Device:  MSP430F6736
//
//  (C) Copyright 2017, Texas Instruments, Inc.
//#############################################################################
// TI Release: MSP430_ENERGY_MEASUREMENT 
// Release Date: 10Jul2018
//#############################################################################

#ifndef _EMREACTIVEPOWERIIRCOEFF_H_
#define _EMREACTIVEPOWERIIRCOEFF_H_
// IIR G Constant Table
// Samp. Freq=4096Hz, Min Freq=49Hz, Max Freq=51Hz, Resolution=0.02Hz
#define	IIR_COEFF_MIN_FREQ		4900
#define	IIR_COEFF_MAX_FREQ		5100
#define	IIR_COEFF_TABLE_LEN		101
#define	IIR_COEFF_SHIFT			1

const _iq30 G_CONSTANT[IIR_COEFF_TABLE_LEN] ={
	0xC4A36FC3, /* Freq 49.00 */
	0xC4A3E774, /* Freq 49.02 */
	0xC4A45F24, /* Freq 49.04 */
	0xC4A4D6D2, /* Freq 49.06 */
	0xC4A54E80, /* Freq 49.08 */
	0xC4A5C62E, /* Freq 49.10 */
	0xC4A63DDA, /* Freq 49.12 */
	0xC4A6B585, /* Freq 49.14 */
	0xC4A72D30, /* Freq 49.16 */
	0xC4A7A4D9, /* Freq 49.18 */
	0xC4A81C82, /* Freq 49.20 */
	0xC4A8942A, /* Freq 49.22 */
	0xC4A90BD1, /* Freq 49.24 */
	0xC4A98377, /* Freq 49.26 */
	0xC4A9FB1C, /* Freq 49.28 */
	0xC4AA72C1, /* Freq 49.30 */
	0xC4AAEA64, /* Freq 49.32 */
	0xC4AB6207, /* Freq 49.34 */
	0xC4ABD9A9, /* Freq 49.36 */
	0xC4AC5149, /* Freq 49.38 */
	0xC4ACC8E9, /* Freq 49.40 */
	0xC4AD4089, /* Freq 49.42 */
	0xC4ADB827, /* Freq 49.44 */
	0xC4AE2FC4, /* Freq 49.46 */
	0xC4AEA761, /* Freq 49.48 */
	0xC4AF1EFD, /* Freq 49.50 */
	0xC4AF9697, /* Freq 49.52 */
	0xC4B00E31, /* Freq 49.54 */
	0xC4B085CA, /* Freq 49.56 */
	0xC4B0FD63, /* Freq 49.58 */
	0xC4B174FA, /* Freq 49.60 */
	0xC4B1EC90, /* Freq 49.62 */
	0xC4B26426, /* Freq 49.64 */
	0xC4B2DBBB, /* Freq 49.66 */
	0xC4B3534E, /* Freq 49.68 */
	0xC4B3CAE1, /* Freq 49.70 */
	0xC4B44273, /* Freq 49.72 */
	0xC4B4BA05, /* Freq 49.74 */
	0xC4B53195, /* Freq 49.76 */
	0xC4B5A925, /* Freq 49.78 */
	0xC4B620B3, /* Freq 49.80 */
	0xC4B69841, /* Freq 49.82 */
	0xC4B70FCE, /* Freq 49.84 */
	0xC4B7875A, /* Freq 49.86 */
	0xC4B7FEE5, /* Freq 49.88 */
	0xC4B8766F, /* Freq 49.90 */
	0xC4B8EDF9, /* Freq 49.92 */
	0xC4B96581, /* Freq 49.94 */
	0xC4B9DD09, /* Freq 49.96 */
	0xC4BA5490, /* Freq 49.98 */
	0xC4BACC16, /* Freq 50.00 */
	0xC4BB439B, /* Freq 50.02 */
	0xC4BBBB1F, /* Freq 50.04 */
	0xC4BC32A2, /* Freq 50.06 */
	0xC4BCAA25, /* Freq 50.08 */
	0xC4BD21A6, /* Freq 50.10 */
	0xC4BD9927, /* Freq 50.12 */
	0xC4BE10A7, /* Freq 50.14 */
	0xC4BE8826, /* Freq 50.16 */
	0xC4BEFFA4, /* Freq 50.18 */
	0xC4BF7721, /* Freq 50.20 */
	0xC4BFEE9D, /* Freq 50.22 */
	0xC4C06619, /* Freq 50.24 */
	0xC4C0DD93, /* Freq 50.26 */
	0xC4C1550D, /* Freq 50.28 */
	0xC4C1CC86, /* Freq 50.30 */
	0xC4C243FE, /* Freq 50.32 */
	0xC4C2BB75, /* Freq 50.34 */
	0xC4C332EC, /* Freq 50.36 */
	0xC4C3AA61, /* Freq 50.38 */
	0xC4C421D6, /* Freq 50.40 */
	0xC4C49949, /* Freq 50.42 */
	0xC4C510BC, /* Freq 50.44 */
	0xC4C5882E, /* Freq 50.46 */
	0xC4C5FF9F, /* Freq 50.48 */
	0xC4C6770F, /* Freq 50.50 */
	0xC4C6EE7F, /* Freq 50.52 */
	0xC4C765ED, /* Freq 50.54 */
	0xC4C7DD5B, /* Freq 50.56 */
	0xC4C854C7, /* Freq 50.58 */
	0xC4C8CC33, /* Freq 50.60 */
	0xC4C9439E, /* Freq 50.62 */
	0xC4C9BB08, /* Freq 50.64 */
	0xC4CA3272, /* Freq 50.66 */
	0xC4CAA9DA, /* Freq 50.68 */
	0xC4CB2142, /* Freq 50.70 */
	0xC4CB98A8, /* Freq 50.72 */
	0xC4CC100E, /* Freq 50.74 */
	0xC4CC8773, /* Freq 50.76 */
	0xC4CCFED7, /* Freq 50.78 */
	0xC4CD763A, /* Freq 50.80 */
	0xC4CDED9C, /* Freq 50.82 */
	0xC4CE64FE, /* Freq 50.84 */
	0xC4CEDC5F, /* Freq 50.86 */
	0xC4CF53BE, /* Freq 50.88 */
	0xC4CFCB1D, /* Freq 50.90 */
	0xC4D0427B, /* Freq 50.92 */
	0xC4D0B9D8, /* Freq 50.94 */
	0xC4D13134, /* Freq 50.96 */
	0xC4D1A890, /* Freq 50.98 */
	0xC4D21FEA, /* Freq 51.00 */
};
#endif
