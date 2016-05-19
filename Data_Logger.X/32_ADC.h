/*
 * File:   .h
 * Author: Armstrong Subero
 * PIC: 32MX230F064B @ 40MHz, 3.3v
 * Program: Header file Nokia 5110 (PCD8544)
 * Compiler: XC32 (v1.33, MPLAX X v2.30)
 * Program Version 1.0
 * Program Description: This header file provides routines for using the
 *                      Nokia 5110 LCD
 *
 * Created on February 13, 2015, 4:59 PM
 ******************************************************************************/

#include <plib.h>

void adcON(){
	// configure and enable the ADC
	CloseADC10();	// ensure the ADC is off before setting the configuration

	// define setup parameters for OpenADC10
				// Turn module on | output in integer | trigger mode auto | enable  autosample
	#define PARAM1  ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON

	// define setup parameters for OpenADC10
			    // ADC ref external    | disable offset test    | enable scan mode | perform 2 samples | use one buffer | use MUXA mode
       // note: to read X number of pins you must set ADC_SAMPLES_PER_INT_X
	#define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF

	// define setup parameters for OpenADC10
	// 				  use ADC internal clock | set sample time
	#define PARAM3  ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_15

	// define setup parameters for OpenADC10
        // set AN1
	#define PARAM4 ENABLE_AN3_ANA

	// define setup parameters for OpenADC10
	// do not assign channels to scan
	#define PARAM5	SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN4 | SKIP_SCAN_AN5 | SKIP_SCAN_AN9 | SKIP_SCAN_AN11

	// use ground as neg ref for A
	SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF); // use ground as the negative reference
	OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using parameter define above

	EnableADC10(); // Enable the ADC

	while ( ! mAD1GetIntFlag() ) { } // wait for the first conversion to complete so there will be valid data in ADC result registers
}
