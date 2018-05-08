/*------------------------------------------------------------------------------------------------*/
/* ___::((odmkZedSystem1_5.c))::___

   ___::((JIROBATA Programming Industries))::___
   ___::((ODMK:2018))::___
   ___::((created by eschei))___

	Purpose: ARM software for odmkZyncSystem1
	Device: zedboard - xc7z020clg484-1
	Revision History: May 16, 2017 - initial
	Revision History: Feb 03, 2018 - version 1_5
*/

/*------------------------------------------------------------------------------------------------*/

/*---%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%---*

ARM software for driving peripherals, etc. used for Zedboard

*---%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%---*/

/*------------------------------------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "platform.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xscugic.h"
#include "xtmrctr.h"
#include "xil_exception.h"
#include "xstatus.h"
#include "sleep.h"
#include "xil_types.h"
#include "xil_cache.h"
#include "xparameters.h"
//#include "xil_printf.h"

#include "oledrgb_pattern1.h"
#include "PmodOLEDrgb.h"


/*------------------------------------------------------------------------------------------------*/
/* (((Definitions))) */

#define bool u8
#define true 1
#define false 0

//#define printf xil_printf							/* smaller, optimised printf !!?doesn't allow for %d variables?!?*/

#define BTN_CH_MASK		XGPIO_IR_CH1_MASK
#define SWS_CH_MASK		XGPIO_IR_CH2_MASK
#define TIMER_INT		XGPIO_IR_CH1_MASK
#define ROTARY1_INT		XGPIO_IR_CH1_MASK
#define ROTARY2_INT		XGPIO_IR_CH1_MASK

//#define LED 0xC3									/* Initial LED value - XX0000XX */
#define LED 0x01									/* Initial LED value - XX0000XX */
#define LED_DELAY		15600000					/* Software delay length */
#define LED_CHANNEL		1							/* GPIO port for LEDs */
#define SW_CHANNEL		2							/* GPIO port for SWs */

/*------------------------------------------------------------------------------------------------*/
/* Device IDs */

//#define Snoop Control Unit (interrupt controller)
#define INTC_DEVICE_ID 				XPAR_PS7_SCUGIC_0_DEVICE_ID

//#define Timer
#define TMRCTR_TIMER_ID				XPAR_AXI_TIMER_0_DEVICE_ID
#define INTC_TIMER_INTERRUPT_ID		XPAR_FABRIC_AXI_TIMER_0_INTERRUPT_INTR

//#define BTNS_DEVICE_ID
#define GPIO_BTNS_ID  				XPAR_AXIGPIO0_BTNS_DEVICE_ID
#define INTC_BTNS_INTERRUPT_ID 		XPAR_FABRIC_AXIGPIO0_BTNS_IP2INTC_IRPT_INTR

//#define ROTARY1_DEVICE_ID
#define GPIO_ROTARY1_ID				XPAR_AXIGPIO1_ROT1_DEVICE_ID
#define INTC_ROTARY1_INTERRUPT_ID 	XPAR_FABRIC_AXIGPIO1_ROT1_IP2INTC_IRPT_INTR

//#define LEDS_SWS_DEVICE_ID
// DUAL: ?LSB[7:0]=LED, MSB[7:0]=SW
#define GPIO_LED_SW_ID  			XPAR_AXIGPIO2_LEDSW_DEVICE_ID

//#define OSC4T Freq1 DEVICE_ID (single channel)
#define GPIO_OSCF1_SSBF1_ID 		XPAR_AXIGPIO3_FREQ1SSB_DEVICE_ID

//#define OSC4T Freq2 & PWM DEVICE_ID (dual channel)
// DUAL: ?LSB[27:0]=DDS FREQ2, MSB[27:0]=PWM FREQ
#define GPIO_OSCF2_PWM_ID 			XPAR_AXIGPIO4_FREQ2PWM_DEVICE_ID

//#define AUDIOMUX_DEVICE_ID
#define GPIO_AUDIOMUX_ID 			XPAR_AXIGPIO5_MUXCTRL_DEVICE_ID

//#define MHL_ALPHA_K_DEVICE_ID
// DUAL: ?LSB[27:0]=ALPHA (G - cutoff), MSB[27:0]=K (resonance)
#define GPIO_MHL_ALPHA_K_ID 		XPAR_AXIGPIO6_ALPHAK_DEVICE_ID

//#define VOLCTRL_DEVICE_ID
#define GPIO_VOLCTRL_ID 			XPAR_AXIGPIO7_VOL_DEVICE_ID

//#define DDL3CTRL_DEVICE_ID
#define GPIO_DDL3CTRL_ID 			XPAR_AXIGPIO8_DDLCTRL_DEVICE_ID

//#define MHL_PARAM1_DEVICE_ID
// DUAL: ?LSB[23:0]=ALPHA0, MSB[23:0]=Beta1
#define GPIO_MHL_PARAM1_ID 			XPAR_AXIGPIO9_FPARAMS1_DEVICE_ID

//#define MHL_PARAM2_DEVICE_ID
// DUAL: ?LSB[23:0]=Beta2, MSB[23:0]=Beta3
#define GPIO_MHL_PARAM2_ID 			XPAR_AXIGPIO10_FPARAMS2_DEVICE_ID

//#define ROTARY2_DEVICE_ID
#define GPIO_ROTARY2_ID				XPAR_AXIGPIO11_ROT2_DEVICE_ID
#define INTC_ROTARY2_INTERRUPT_ID 	XPAR_FABRIC_AXIGPIO11_ROT2_IP2INTC_IRPT_INTR

//#define ROTARY3_DEVICE_ID
#define GPIO_ROTARY3_ID				XPAR_AXIGPIO12_ROT3_DEVICE_ID
#define INTC_ROTARY3_INTERRUPT_ID 	XPAR_FABRIC_AXIGPIO12_ROT3_IP2INTC_IRPT_INTR


/*------------------------------------------------------------------------------------------------*/
/* instantiate objects */
XScuGic INTCInst;
XTmrCtr TIMERInst;
PmodOLEDrgb oledrgb;
XGpio LEDSWSInst;
XGpio BTNInst;
XGpio AUDIOMUXInst;
XGpio ROTARY1Inst;
XGpio ROTARY2Inst;
XGpio VOLCTRLInst;
XGpio OSCF1SSBStepInst;
XGpio OSCF2PWMStepInst;
XGpio DDL3CTRLInst;
XGpio MHLALPHAKInst;
XGpio MHLCTRL1Inst;
XGpio MHLCTRL2Inst;


/*------------------------------------------------------------------------------------------------*/
/* (((Constant Variables))) */

const float pi = 3.14159265359;

const float clkFs = 100000000.0;
const float fs = 48000.0;

// OLEDrgb_DrawBitmap function delay (approximate! - needs accurate tuning)
// *empirically measured tempo lag + included usleep(5000) func call
const unsigned long oledTempoLag = 50000 + 5000;

// Tunings:
const float mstrVolScale = 0.23;
const u16 maxVolVal = 32767; 				// typedef ap_uint<15> delay_t;

const int oscFreqScale = 560;
const int lfoFreqScale = 1;
const int ssbFreqScale = 756000;

const int mooghlCutoffScale = 111;
const float filtVolScale = 2.7;
const float mooghlResScale = 0.007;

const float ddlWetDryScale = 0.005;
const u16 ddlLengthScale = 56;
const float ddlFbGainScale = 0.005;
const unsigned int ddlCtrlBwidth = 14;			// ap_fixed<16,2>
const u16 maxDlyLength = 32767; 				// typedef ap_uint<15> delay_t;

// led patterns
const int ledFlash1[] = {24, 36, 66, 129};		// center out LED strobe pattern

/*------------------------------------------------------------------------------------------------*/
/* (((Static Variables))) */

static int INT_CH_MASK = XGPIO_IR_CH1_MASK;
static int INTC_X_INTERRUPT_ID = INTC_BTNS_INTERRUPT_ID;	// arbitrarily defaults to button interrupt ID

static float BPM = 133.0;	// Default BPM
static unsigned long uSECPERBEAT = 451128;		// 1000000*60/BPM - default initial value assuming 133 bpm

static u8 audioMux3_prev = 0;

static float mooghlCutoff = 24860.00;	// modify to Hz values (currently raw value)
static float mooghlRes = 0.0;

//static int ledData = 0;
static int ledState = 0;	// used for LED manipulation function
static bool trigOnce = 1;

static int btnValue;
static bool btnDirection = 1;

// fader Mux select
static u8 faderMuxSel = 0;
static u8 fader2MuxSel = 0;
static int sysmode = 0;	// 0=>OLED info ; 1=>OLED imgRotate ; 2=>?

//static int rotary_AB;
static u8 rotary_AB = 0;
static u8 rotary2_AB = 0;
//static u8 rotary3_AB = 0;

static double volCtrlDB = 25.2;		// Attenuation amount: 0 DB = full volume, -140 DB ~ -inf
static u32 volCtrlVal = 3601;		// 16b exponential audio out volume control val { 0 - 32767 }, default =16422 (-6 DB)

/*------------------------------------------------------------------------------------------------*/
// define types for switches
struct sws_type {
	bool sw1: 1;
	bool sw2: 1;
	bool sw3: 1;
	bool sw4: 1;
	bool sw5: 1;
	bool sw6: 1;
	bool sw7: 1;
	bool sw8: 1;
};

union sws_u {
	u32 swValue;
	struct sws_type switches;
};

static union sws_u swsUnion;

// audioMux5_0: {4,2} = input audio MUX sel
// 0 = sin, 1 = saw, 2 = sqr, 3 = pwm, 4 = ext

// audioMux3_0: {1,0} = output audio MUX sel
// 0 = Dry, 1 = DDL, 2 = SSB, 3 = Filter

// filterInMuxSel: {1,0} = Filter Input Mux sel
// 0 = Dry, 1 = DDL, 2 = SSB

// ddlInMuxSel: = DDL Input Mux sel
// 0 = Dry, 1 = SSB

struct auMux_type {
	u8 audioMux5_0: 3;			// Audio Source Mux (audioSrcMux5Stereo) - right button
	u8 audioMux3_0: 2;			// Audio Output Mux (audioOutMux4Stereo) - sw1&2
	u8 filterInMuxSel: 2;		// Filter Input Mux (filterSelMux2Stereo) - sw3&4
	bool ddlInMuxSel: 1;		// Delay Input Mux (ddlFBMux2Stereo) - sw5
};

union auMux_u {
	u16 audioMuxSel;
	struct auMux_type audioSelect;
};

static union auMux_u auMuxUnion;



/*------------------------------------------------------------------------------------------------*/
//  __OSC4T__
//
// oscOutFreq = clkFs * oscStep / pow(2, hirezPhaseAccWidth)
// oscOutFreq = 100000000 * oscStep / pow(2, 36);
//
// osc1Step = osc1OutFreq * pow(2, hirezPhaseAccWidth) / clkFs;
// osc1Step = (int)osc1OutFreq * pow(2, 36) / 100000000;
//
// 560 Hz osc output 384829 (hex: 0x00005DF3D)
// 560 / 2 : 192414 (hex: 0x00002EF9E)
//static int oscFreq1StepVal = 192414;
static int oscFreq1StepVal = 60836;		// ~ 88.53 Hz - (??check calc??)
static int oscFreq2StepVal = 317;		// ~ 118.09 Hz - (??check calc??)
static int oscPWMStepVal = 3848;		// ~ 5.6 Hz

const int hirezPhaseAccWidth = 36;					// high resolution phase accumulator width
const int lorezPhaseAccWidth = 28;					// low resolution phase accumulator width

static float oscFreq1;
static float oscFreq2;

/* OSC4T USER INPUTS

ctrlBus_lfoType_V	[1:0]
ctrlBus_syncSw_V	[0:0]
ctrlBus_fMuxSel_V	[1:0]
ctrlBus_pwMuxSel_V	[1:0]
ctrlBus_upDn1_V		[0:0]
ctrlBus_upDn2_V		[0:0]

freq1_V 			[35:0]
freq2_V				[27:0]
fm_V				[27:0]
pwm_V				[27:0]
fmVol_V				[11:0]

*/

/*------------------------------------------------------------------------------------------------*/
//  __SSB - Single Side-Band Modulator__

static int ssbSetStepVal = 11000093;	// SSM Modulation frequency


/*------------------------------------------------------------------------------------------------*/
// define struct for Moog Half-Ladder Filter parameters

// ap_fixed<24,3>

struct moogHLParam_type {
	int K;
	int alpha0;
	int alpha;
	int beta1;
	int beta2;
	int beta3;
};

struct moogHLParam_type mooghlParam;


/*------------------------------------------------------------------------------------------------*/
// define struct for DDL3 CTRL parameters

//u32 DDL3CTRL_CH1Val;	// odmkDDL3_0 delayLength ctrl - 32 bit bus {wetMix[15:0], dLengthFlag[0:0], delayLength[14:0]}
//u32 DDL3CTRL_CH2Val;	// odmkDDL3_0 feedback ctrl - 18 bit bus {fbMuxSel[0:0], selectExtFB[0:0], feedbackGain[15:0]}

static float wetMixFloat = 0.5;
static float feedbackGainFloat = 0;

static u16 delayLengthTmp = 0;

// DDL3 CH1
struct DDL3CTRL1_type {
	u32 delayLength: 15;		// range = 0 - ap_fixed<16,2> (*** re-gen IP to change dataType from ap_fixed to ap_ufixed..)
	bool dLengthFlag: 1;
	u16 wetMix: 16;				// range = 0.0:1.0 <-> ap_fixed<16,2> (*** re-gen IP to change dataType from ap_fixed to ap_ufixed..)
};

union DDL3CTRL1_u {
	u32 DDL3CTRL_CH1Val;
	struct DDL3CTRL1_type DDL3CTRL1;
};

static union DDL3CTRL1_u DDL3CTRL1Union;


// DDL3 CH2
struct DDL3CTRL2_type {
	u16 feedbackGain: 16;
	bool selectExtFB: 1;
	bool fbMuxSel: 1;
};

union DDL3CTRL2_u {
	u32 DDL3CTRL_CH2Val;
	struct DDL3CTRL2_type DDL3CTRL2;
};

static union DDL3CTRL2_u DDL3CTRL2Union;


/*------------------------------------------------------------------------------------------------*/
// oledRGB

uint8_t rgbUserFont[] = {
	0x00, 0x04, 0x02, 0x1F, 0x02, 0x04, 0x00, 0x00,	// 0x00
	0x0E, 0x1F, 0x15, 0x1F, 0x17, 0x10, 0x1F, 0x0E,	// 0x01
	0x00, 0x1F, 0x11, 0x00, 0x00, 0x11, 0x1F, 0x00,	// 0x02
	0x00, 0x0A, 0x15, 0x11, 0x0A, 0x04, 0x00, 0x00,	// 0x03
    0x07, 0x0C, 0xFA, 0x2F, 0x2F, 0xFA, 0x0C, 0x07  // 0x04
}; // this table defines 5 user characters, although only one is used


/*------------------------------------------------------------------------------------------------*/
// PROTOTYPE FUNCTIONS

static void BTN_Intr_Handler(void *baseaddr_p);
static void ROTARY1_Intr_Handler(void *InstancePtr);
static void ROTARY2_Intr_Handler(void *InstancePtr);
static int InterruptSystemSetup(XScuGic *XScuGicInstancePtr, XGpio *GpioInstancePtr, int INT_CH_MASK);
int IntcGpioInitFunction(u16 DeviceId, XGpio *GpioInstancePtr, Xil_ExceptionHandler X_Intr_Handler, int INT_X_ID, int INT_CH_MASK);
//void audioMux3_switch(void *audioMuxGpioInstPtr, u8 auMux3sel);
void odmkZynqLED1(unsigned long uSECPERBEAT);
void ftoa(char *buffer, float d, int precision);
void odmkInfoScreen(void *oledrgbPtr, float bpm);
void odmkAudioMuxScreen(void *oledrgbPtr, int muxSel);
void odmkFaderMuxScreen(void *oledrgbPtr, int faderSel);
void odmkFader2MuxScreen(void *oledrgbPtr, int fader2Sel);
void setFcAndRes_ARM(float sampleRate, float cutoff, float resonance, struct moogHLParam_type *fParamSW);


/*------------------------------------------------------------------------------------------------*/
// INTERRUPT HANDLER FUNCTIONS


//----------------------------------------------------
// Button INTERRUPT HANDLER
// - button interrupt, performs
// -
// - Button Values returned when pressed:
// - center: 1, bottom: 2, left: 4, right: 8, top: 16
//----------------------------------------------------
void BTN_Intr_Handler(void *InstancePtr)
{

	// Disable GPIO interrupts
	XGpio_InterruptDisable(&BTNInst, BTN_CH_MASK);

	// Ignore additional button presses
	if ((XGpio_InterruptGetStatus(&BTNInst) & BTN_CH_MASK) != BTN_CH_MASK) {
		return;
	}

	usleep(50000);	//Wait 1/1000 seconds - button debounce

	btnValue = XGpio_DiscreteRead(&BTNInst, 1);

	// Reset if centre button pressed
	if(btnValue == 1) {
		// Reset to Top - print initial info / tempo - LED BPM flash
		printf("BTN[%d] Top State\n", btnValue);
		sysmode = 0;
		trigOnce = 1;
	}
	// Lower Button pressed
	else if(btnValue == 2) {
		// Fader Mux Sel - selects the target for fader control
		printf("BTN[%d] - Fader 2 Target Select\n", btnValue);
		sysmode = 1;
		trigOnce = 1;

		if (fader2MuxSel == 2) {
			fader2MuxSel = 0;
		} else {
			fader2MuxSel++;
		}
	}
	// Left Button pressed
	else if(btnValue == 4) {
		printf("BTN[%d] Update SW State\n", btnValue);
		sysmode = 2;
		trigOnce = 0;
		swsUnion.swValue = XGpio_DiscreteRead(&LEDSWSInst, 2);

		// update Audio Mux Select
		auMuxUnion.audioSelect.audioMux3_0 = (u8)(3&swsUnion.swValue);
		XGpio_DiscreteWrite(&AUDIOMUXInst, 1, auMuxUnion.audioMuxSel);
		if (audioMux3_prev != 3 && auMuxUnion.audioSelect.audioMux3_0 == 3) {
			printf("Enabled Filter Gain Compensation\n");
			printf("volCtrlDB = %f,   volCtrlVal = %d\n", volCtrlDB, (int)volCtrlVal);
			//volCtrlDB+=filtVolScale;	// ?doesn't work?
			volCtrlDB-=4.6;
			if (volCtrlDB <= 0.0) {
				volCtrlDB = 0.0;		// 0 DB, Max Volume
			}
			volCtrlVal = (u32)maxVolVal*(pow((float)10.0, -volCtrlDB/20));
			XGpio_DiscreteWrite(&VOLCTRLInst, 1, volCtrlVal);
			audioMux3_prev = auMuxUnion.audioSelect.audioMux3_0;
		} else if(audioMux3_prev == 3 && auMuxUnion.audioSelect.audioMux3_0 != 3) {
			printf("Disabled Filter Gain Compensation\n");
			printf("volCtrlDB = %f,   volCtrlVal = %d\n", volCtrlDB, (int)volCtrlVal);
			//volCtrlDB-=filtVolScale;
			volCtrlDB+=4.6;
			if (volCtrlDB >= 140.0) {
				volCtrlDB = 140.0;		// -140 DB, Min Volume
			}
			volCtrlVal = (u32)maxVolVal*(pow((float)10.0, -volCtrlDB/20));
			XGpio_DiscreteWrite(&VOLCTRLInst, 1, volCtrlVal);
			audioMux3_prev = auMuxUnion.audioSelect.audioMux3_0;
		}

		// update filter Data In Mux (filterInMux3Stereo) - sw3&sw4
		auMuxUnion.audioSelect.filterInMuxSel = (u8)(12&swsUnion.swValue)>>2;
		printf("filterInMuxSel = %d\n", (int)auMuxUnion.audioSelect.filterInMuxSel);
		if (auMuxUnion.audioSelect.filterInMuxSel == 0) {
			printf("Filter Input: Source Direct\n");
			XGpio_DiscreteWrite(&AUDIOMUXInst, 1, auMuxUnion.audioMuxSel);
		} else if (auMuxUnion.audioSelect.filterInMuxSel == 1) {
			printf("Filter Input: DDL\n");
			XGpio_DiscreteWrite(&AUDIOMUXInst, 1, auMuxUnion.audioMuxSel);
		} else {
			printf("Filter Input: SSB Modulation\n");
			XGpio_DiscreteWrite(&AUDIOMUXInst, 1, auMuxUnion.audioMuxSel);
		}

		// update DDL Data In Mux (ddlInMux2Stereo) - sw5
		auMuxUnion.audioSelect.ddlInMuxSel = swsUnion.switches.sw5;
		if (auMuxUnion.audioSelect.ddlInMuxSel == 0) {
			printf("Filter Input: Source Direct\n");
			XGpio_DiscreteWrite(&AUDIOMUXInst, 1, auMuxUnion.audioMuxSel);
		} else {
			printf("Filter Input: SSB Modulation\n");
			XGpio_DiscreteWrite(&AUDIOMUXInst, 1, auMuxUnion.audioMuxSel);
		}

		// update DDL selectExtFB
		if (swsUnion.switches.sw6 == 1) {
			printf("DDL selectExtFB = %d\n", swsUnion.switches.sw6);
			DDL3CTRL2Union.DDL3CTRL2.selectExtFB = 1;
		} else {
			printf("DDL selectExtFB = %d\n", swsUnion.switches.sw6);
			DDL3CTRL2Union.DDL3CTRL2.selectExtFB = 0;
		}

		// update DDL feedbackIn Mux Select {CH1: SSB, CH2: Filter}
		if (swsUnion.switches.sw7 == 1) {
			printf("DDL feedbackIn Mux Sel = %d\n", swsUnion.switches.sw7);
			DDL3CTRL2Union.DDL3CTRL2.fbMuxSel = 1;
		} else {
			printf("DDL feedbackIn Mux Sel = %d\n", swsUnion.switches.sw7);
			DDL3CTRL2Union.DDL3CTRL2.fbMuxSel = 0;
		}
		XGpio_DiscreteWrite(&DDL3CTRLInst, 2, DDL3CTRL2Union.DDL3CTRL_CH2Val);


	}
	// Right Button pressed
	else if(btnValue == 8) {
		// Audio Mux Select - btn increments mux sel
		printf("BTN[%d] Audio Source Select\n", btnValue);
		sysmode = 3;
		trigOnce = 1;

		if (btnDirection == 0 && auMuxUnion.audioSelect.audioMux5_0 == 4) {
			auMuxUnion.audioSelect.audioMux5_0 = 0;
		} else if (btnDirection == 1 && auMuxUnion.audioSelect.audioMux5_0 == 0) {
			auMuxUnion.audioSelect.audioMux5_0 = 4;
		} else {
			if (btnDirection == 0) {
				auMuxUnion.audioSelect.audioMux5_0++;
			} else {
				auMuxUnion.audioSelect.audioMux5_0--;
			}
		}

		XGpio_DiscreteWrite(&AUDIOMUXInst, 1, auMuxUnion.audioMuxSel);

	}
	// Top Button pressed
	else if(btnValue == 16) {
		// Fader Mux Sel - selects the target for fader control
		printf("BTN[%d] - Fader Target Select\n", btnValue);
		sysmode = 4;
		trigOnce = 1;

		if (btnDirection == 0 && faderMuxSel == 6) {
			faderMuxSel = 0;
		} else if (btnDirection == 1 && faderMuxSel == 0) {
			faderMuxSel = 6;
		} else {
			if (btnDirection == 0) {
				faderMuxSel++;
			} else {
				faderMuxSel--;
			}
		}

	}

	//printf("TEMP: BTN Intr Handler - exited case - faderMuxSel = %i\n", faderMuxSel);

	usleep(300000);	//Wait 1/1000 seconds - button debounce

    (void)XGpio_InterruptClear(&BTNInst, BTN_CH_MASK);

    // Enable GPIO interrupts
    XGpio_InterruptEnable(&BTNInst, BTN_CH_MASK);

}


//----------------------------------------------------
// Rotary Encoder INTERRUPT HANDLER
// - called by the rotary encoder interrupt, performs
// - increment / decrement of phaseAcc step value
//----------------------------------------------------
void ROTARY1_Intr_Handler(void *InstancePtr)
{

	// Disable GPIO interrupts
	XGpio_InterruptDisable(&ROTARY1Inst, ROTARY1_INT);

	// Ignore additional button presses
	if ((XGpio_InterruptGetStatus(&ROTARY1Inst) & ROTARY1_INT) != ROTARY1_INT) {
		return;
	}

	rotary_AB = XGpio_DiscreteRead(&ROTARY1Inst, 1);

	if (faderMuxSel == 0) {

		//printf("Master Vol Ctrl: Fader MuxSel = %d\n\r",faderMuxSel);

		// Audio Output volume control {0 dB MAX : -140 dB MIN}
		// convert dB to 16 bit unsigned integer values {0:65536}
		if(rotary_AB == 0) {
			//printf("Rotary Enc -Left- rotary_AB = %d\n",rotary_AB);
			volCtrlDB+=mstrVolScale;
			if (volCtrlDB >= 140.0) {
				volCtrlDB = 140.0;		// -140 DB, Min Volume
			}
			//volCtrlVal = (u32)32767*(pow((float)10, -volCtrlDB/20));
			volCtrlVal = (u32)maxVolVal*(pow((double)10.0, -volCtrlDB/20));
			XGpio_DiscreteWrite(&VOLCTRLInst, 1, volCtrlVal);
		}
		else if(rotary_AB == 2) {
			//printf("Rotary Enc -Right- rotary_AB = %d\n",rotary_AB);
			volCtrlDB-=mstrVolScale;
			if (volCtrlDB <= 0.0) {
				volCtrlDB = 0.0;		// 0 DB, Max Volume
			}
			volCtrlVal = (u32)maxVolVal*(pow((double)10.0, -volCtrlDB/20));
			XGpio_DiscreteWrite(&VOLCTRLInst, 1, volCtrlVal);
		}
		else {

		}

	} else if (faderMuxSel == 1) {

		//printf("OSC Freq Ctrl: Fader MuxSel = %d\n\r",faderMuxSel);

		// Update DDS phase accumulator value (OSC output Frequency)
		if(rotary_AB == 0) {
			//printf("Rotary Enc -Left- rotary_AB = %d\n\r",rotary_AB);
			oscFreq1StepVal-=oscFreqScale;
			XGpio_DiscreteWrite(&OSCF1SSBStepInst, 1, oscFreq1StepVal);
		}
		else if(rotary_AB == 2) {
			//printf("Rotary Enc -Right- rotary_AB = %d\n\r",rotary_AB);
			oscFreq1StepVal+=oscFreqScale;
			XGpio_DiscreteWrite(&OSCF1SSBStepInst, 1, oscFreq1StepVal);
		}
		else {

		}

	} else if (faderMuxSel == 2) {

		//printf("LFO Freq Ctrl: Fader MuxSel = %d\n\r",faderMuxSel);

		// Update OSC4T freq2 phase accumulator value (LFO output Frequency)
		if(rotary_AB == 0) {
			//printf("Rotary Enc -Left- rotary_AB = %d\n\r",rotary_AB);
			oscFreq2StepVal-=lfoFreqScale;
			XGpio_DiscreteWrite(&OSCF2PWMStepInst, 1, oscFreq2StepVal);
		}
		else if(rotary_AB == 2) {
			//printf("Rotary Enc -Right- rotary_AB = %d\n\r",rotary_AB);
			oscFreq2StepVal+=lfoFreqScale;
			XGpio_DiscreteWrite(&OSCF2PWMStepInst, 1, oscFreq2StepVal);
		}
		else {

		}

	} else if (faderMuxSel == 3) {

		//printf("PWM Freq Ctrl: Fader MuxSel = %d\n\r",faderMuxSel);

		// Update OSC4T PWM phase accumulator value (SSB modulation Frequency)
		if(rotary_AB == 0) {
			//printf("Rotary Enc -Left- (NoEvent) - rotary_AB = %d\n\r",rotary_AB);
			oscPWMStepVal-=lfoFreqScale;
			if (ssbSetStepVal < 0) ssbSetStepVal = 0;
			XGpio_DiscreteWrite(&OSCF2PWMStepInst, 2, oscPWMStepVal);
		}
		else if(rotary_AB == 2) {
			//printf("Rotary Enc -Right- (NoEvent) - rotary_AB = %d\n\r",rotary_AB);
			oscPWMStepVal+=lfoFreqScale;
			XGpio_DiscreteWrite(&OSCF2PWMStepInst, 2, oscPWMStepVal);
		}
		else {

		}

	} else if (faderMuxSel == 4) {

		//printf("SSB Freq Ctrl: Fader MuxSel = %d\n\r",faderMuxSel);

		// Update SSB Modulator phase accumulator value (SSB modulation Frequency)
		if(rotary_AB == 0) {
			//printf("Rotary Enc -Left- rotary_AB = %d\n\r",rotary_AB);
			ssbSetStepVal-=ssbFreqScale;
			if (ssbSetStepVal < 0) ssbSetStepVal = 0;
			XGpio_DiscreteWrite(&OSCF1SSBStepInst, 2, ssbSetStepVal);
		}
		else if(rotary_AB == 2) {
			//printf("Rotary Enc -Right- rotary_AB = %d\n\r",rotary_AB);
			ssbSetStepVal+=ssbFreqScale;
			XGpio_DiscreteWrite(&OSCF1SSBStepInst, 2, ssbSetStepVal);
		}
		else {

		}

	} else if (faderMuxSel == 5) {

		//printf("FLT Cutoff Freq Ctrl: Fader MuxSel = %d\n\r",faderMuxSel);

		// Update FLT Cutoff Freq & re-calculate parameters (FLT Cutoff Frequency)
		if(rotary_AB == 0) {
			//printf("Rotary Enc -Left- rotary_AB = %d\n",rotary_AB);
			mooghlCutoff-=mooghlCutoffScale;
			if (mooghlCutoff <= 0) {
				mooghlCutoff = 0;
			}
			setFcAndRes_ARM(fs, mooghlCutoff, mooghlRes, &mooghlParam);
			XGpio_DiscreteWrite(&MHLALPHAKInst, 1, mooghlParam.alpha);
			XGpio_DiscreteWrite(&MHLALPHAKInst, 2, mooghlParam.K);
			XGpio_DiscreteWrite(&MHLCTRL1Inst, 1, mooghlParam.alpha0);
			XGpio_DiscreteWrite(&MHLCTRL1Inst, 2, mooghlParam.beta1);
			XGpio_DiscreteWrite(&MHLCTRL2Inst, 1, mooghlParam.beta2);
			XGpio_DiscreteWrite(&MHLCTRL2Inst, 2, mooghlParam.beta3);
		}
		else if(rotary_AB == 2) {
			//printf("Rotary Enc -Right- rotary_AB = %d\n",rotary_AB);
			mooghlCutoff+=mooghlCutoffScale;
			if (mooghlCutoff >= 30860) {
				mooghlCutoff = 30860;
			}
			setFcAndRes_ARM(fs, mooghlCutoff, mooghlRes, &mooghlParam);
			XGpio_DiscreteWrite(&MHLALPHAKInst, 1, mooghlParam.alpha);
			XGpio_DiscreteWrite(&MHLALPHAKInst, 2, mooghlParam.K);
			XGpio_DiscreteWrite(&MHLCTRL1Inst, 1, mooghlParam.alpha0);
			XGpio_DiscreteWrite(&MHLCTRL1Inst, 2, mooghlParam.beta1);
			XGpio_DiscreteWrite(&MHLCTRL2Inst, 1, mooghlParam.beta2);
			XGpio_DiscreteWrite(&MHLCTRL2Inst, 2, mooghlParam.beta3);
		}
		else {

		}

	} else if (faderMuxSel == 6) {

		//printf("FLT Cutoff Freq Ctrl: Fader MuxSel = %d\n\r",faderMuxSel);

		// Update FLT Cutoff Freq & re-calculate parameters (FLT Cutoff Frequency)
		if(rotary_AB == 0) {
			//printf("Rotary Enc -Left- rotary_AB = %d\n",rotary_AB);
			mooghlRes-=mooghlResScale;
			//mooghlRes-=0.003;
			if (mooghlRes <= 0.0) {
				mooghlRes = 0.0;
			}
			setFcAndRes_ARM(fs, mooghlCutoff, mooghlRes, &mooghlParam);
			XGpio_DiscreteWrite(&MHLALPHAKInst, 1, mooghlParam.alpha);
			XGpio_DiscreteWrite(&MHLALPHAKInst, 2, mooghlParam.K);
			XGpio_DiscreteWrite(&MHLCTRL1Inst, 1, mooghlParam.alpha0);
			XGpio_DiscreteWrite(&MHLCTRL1Inst, 2, mooghlParam.beta1);
			XGpio_DiscreteWrite(&MHLCTRL2Inst, 1, mooghlParam.beta2);
			XGpio_DiscreteWrite(&MHLCTRL2Inst, 2, mooghlParam.beta3);
		}
		else if(rotary_AB == 2) {
			//printf("Rotary Enc -Right- rotary_AB = %d\n",rotary_AB);
			mooghlRes+=mooghlResScale;
			//mooghlRes+=0.003;
			if (mooghlRes >= 2.0) {
				mooghlRes = 2.0;
			}
			setFcAndRes_ARM(fs, mooghlCutoff, mooghlRes, &mooghlParam);
			XGpio_DiscreteWrite(&MHLALPHAKInst, 1, mooghlParam.alpha);
			XGpio_DiscreteWrite(&MHLALPHAKInst, 2, mooghlParam.K);
			XGpio_DiscreteWrite(&MHLCTRL1Inst, 1, mooghlParam.alpha0);
			XGpio_DiscreteWrite(&MHLCTRL1Inst, 2, mooghlParam.beta1);
			XGpio_DiscreteWrite(&MHLCTRL2Inst, 1, mooghlParam.beta2);
			XGpio_DiscreteWrite(&MHLCTRL2Inst, 2, mooghlParam.beta3);
		}
		else {

		}

	}

    (void)XGpio_InterruptClear(&ROTARY1Inst, ROTARY1_INT);

    // Enable GPIO interrupts
    XGpio_InterruptEnable(&ROTARY1Inst, ROTARY1_INT);

}


//----------------------------------------------------
// Rotary Encoder INTERRUPT HANDLER CH2
// - called by the rotary encoder interrupt, performs
// - increment / decrement of phaseAcc step value
//----------------------------------------------------
void ROTARY2_Intr_Handler(void *InstancePtr)
{

	// Disable GPIO interrupts
	XGpio_InterruptDisable(InstancePtr, ROTARY2_INT);

	// Ignore additional button presses
	if ((XGpio_InterruptGetStatus(InstancePtr) & ROTARY2_INT) != ROTARY2_INT) {
		return;
	}

	rotary2_AB = XGpio_DiscreteRead(InstancePtr, 1);

	if (fader2MuxSel == 0) {

		//printf("DDL Wet/Dry Ctrl: Fader2 MuxSel = %d\n\r",fader2MuxSel);
		// DDL Wet/Dry 15bit {0 = 100% Dry : 32767 100% Wet}
		if(rotary2_AB == 0) {
			//printf("Rotary Enc -Left- rotary2_AB = %d\n", rotary2_AB);
			wetMixFloat-=ddlWetDryScale;
			if (wetMixFloat <= 0.0) wetMixFloat = 0.0;		// -140 DB, Min Volume
			DDL3CTRL1Union.DDL3CTRL1.wetMix = (u16)(wetMixFloat*(1<<ddlCtrlBwidth));
			//printf("wetMixFloat = %f,  wetMix = %d\n", wetMixFloat, (int)DDL3CTRL1Union.DDL3CTRL1.wetMix);
			XGpio_DiscreteWrite(&DDL3CTRLInst, 1, DDL3CTRL1Union.DDL3CTRL_CH1Val);
		}
		else if(rotary2_AB == 2) {
			//printf("Rotary Enc -Right- rotary2_AB = %d\n", rotary2_AB);
			wetMixFloat+=ddlWetDryScale;
			if (wetMixFloat >= 1.0) wetMixFloat = 1.0;
			DDL3CTRL1Union.DDL3CTRL1.wetMix = (u16)(wetMixFloat*(1<<ddlCtrlBwidth));
			//printf("wetMixFloat = %f,  wetMix = %d\n", wetMixFloat, (int)DDL3CTRL1Union.DDL3CTRL1.wetMix);
			XGpio_DiscreteWrite(&DDL3CTRLInst, 1, DDL3CTRL1Union.DDL3CTRL_CH1Val);
		}
		else {

		}

	} else if (fader2MuxSel == 1) {

		//printf("DDL Length Ctrl: Fader2 MuxSel = %d\n\r",fader2MuxSel);
		// DDL Length control {0 samples MIN : ? samples MAX}
		if(rotary2_AB == 0) {
			//printf("Rotary Enc -Left- rotary_AB = %d\n\r",rotary2_AB);
			delayLengthTmp-=ddlLengthScale;
			if (delayLengthTmp < 0 || delayLengthTmp > maxDlyLength) delayLengthTmp = 0;
			DDL3CTRL1Union.DDL3CTRL1.delayLength = delayLengthTmp;
			//printf("delayLength = %d\n", (int)DDL3CTRL1Union.DDL3CTRL1.delayLength);
			DDL3CTRL1Union.DDL3CTRL1.dLengthFlag = 1;
			XGpio_DiscreteWrite(&DDL3CTRLInst, 1, DDL3CTRL1Union.DDL3CTRL_CH1Val);
			usleep(8);	// wait small amount of time to latch in new delayLength
			DDL3CTRL1Union.DDL3CTRL1.dLengthFlag = 0;
			XGpio_DiscreteWrite(&DDL3CTRLInst, 1, DDL3CTRL1Union.DDL3CTRL_CH1Val);
		}
		else if(rotary2_AB == 2) {
			//printf("Rotary Enc -Right- rotary_AB = %d\n\r",rotary2_AB);
			delayLengthTmp+=ddlLengthScale;
			if (delayLengthTmp > maxDlyLength) delayLengthTmp = maxDlyLength;
			DDL3CTRL1Union.DDL3CTRL1.delayLength = delayLengthTmp;
			//printf("delayLength = %d\n", (int)DDL3CTRL1Union.DDL3CTRL1.delayLength);
			DDL3CTRL1Union.DDL3CTRL1.dLengthFlag = 1;
			XGpio_DiscreteWrite(&DDL3CTRLInst, 1, DDL3CTRL1Union.DDL3CTRL_CH1Val);
			usleep(8);	// wait small amount of time to latch in new delayLength
			DDL3CTRL1Union.DDL3CTRL1.dLengthFlag = 0;
			XGpio_DiscreteWrite(&DDL3CTRLInst, 1, DDL3CTRL1Union.DDL3CTRL_CH1Val);
		}
		else {

		}

	} else if (fader2MuxSel == 2) {

		//printf("DDL feedback Gain Ctrl: Fader2 MuxSel = %d\n\r",fader2MuxSel);
		// DDL feedback Gain control {0 samples MIN : ? samples MAX}
		if(rotary2_AB == 0) {
			//printf("Rotary Enc -Left- rotary_AB = %d\n\r", rotary2_AB);
			feedbackGainFloat-=ddlFbGainScale;
			if (feedbackGainFloat < 0) feedbackGainFloat = 0;
			DDL3CTRL2Union.DDL3CTRL2.feedbackGain = (u16)(feedbackGainFloat*(1<<ddlCtrlBwidth));
			//printf("feedbackGainFloat = %f,   feedbackGain = %d\n", feedbackGainFloat, (int)DDL3CTRL2Union.DDL3CTRL2.feedbackGain);
			XGpio_DiscreteWrite(&DDL3CTRLInst, 2, DDL3CTRL2Union.DDL3CTRL_CH2Val);
		}
		else if(rotary2_AB == 2) {
			//printf("Rotary Enc -Right- rotary_AB = %d\n\r", rotary2_AB);
			feedbackGainFloat+=ddlFbGainScale;
			if (feedbackGainFloat >= 1.0) {
				feedbackGainFloat = 1.0;
			}
			DDL3CTRL2Union.DDL3CTRL2.feedbackGain = (u16)(feedbackGainFloat*(1<<ddlCtrlBwidth));
			//printf("feedbackGainFloat = %f,   feedbackGain = %d\n", feedbackGainFloat, (int)DDL3CTRL2Union.DDL3CTRL2.feedbackGain);
			XGpio_DiscreteWrite(&DDL3CTRLInst, 2, DDL3CTRL2Union.DDL3CTRL_CH2Val);
		}
		else {

		}

	}

    (void)XGpio_InterruptClear(InstancePtr, ROTARY2_INT);

    // Enable GPIO interrupts
    XGpio_InterruptEnable(InstancePtr, ROTARY2_INT);

}


/*------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

//----------------------------------------------------
// AudioMux3 update function - Selects main Audio out
//----------------------------------------------------
//void SWS_State(void *audioMuxGpioInstPtr, u8 auMux3sel)
//void audioMux3_switch(void *audioMuxGpioInstPtr, u8 auMux3sel)
//{
//
//	swsUnion.swValue = XGpio_DiscreteRead(&LEDSWSInst, 2);
//
//	// Handel Audio Mux 2 - selects Dry/SSBMod/Filter
//	if(auMux3sel == 0) {
//		// Bypass EFFX - Dry Output
//		printf("Audio Mux 3 {00}: No EFFX\n");
//		auMuxUnion.audioSelect.audioMux3_0 = 0;
//		XGpio_DiscreteWrite(audioMuxGpioInstPtr, 2, auMuxUnion.audioMuxSel);
//		sysmode = 3;
//	}
//	else if(auMux3sel == 1) {
//		// SSB Modulation EFFX
//		printf("Audio Mux 3 {01}: SSB Modulation EFFX\n");
//		auMuxUnion.audioSelect.audioMux3_0 = 1;
//		XGpio_DiscreteWrite(audioMuxGpioInstPtr, 2, auMuxUnion.audioMuxSel);
//		sysmode = 3;
//	}
//	else if(auMux3sel == 2) {
//		// Lowpass Filter
//		printf("Audio Mux 3 {10}: Lowpass Filter\n");
//		auMuxUnion.audioSelect.audioMux3_0 = 2;
//		XGpio_DiscreteWrite(audioMuxGpioInstPtr, 2, auMuxUnion.audioMuxSel);
//		sysmode = 3;
//	}
//	else if(auMux3sel == 3) {
//		// Bypass EFFX - Dry Output
//		printf("Audio Mux 3 {11}: No EFFX\n");
//		auMuxUnion.audioSelect.audioMux3_0 = 0;
//		XGpio_DiscreteWrite(audioMuxGpioInstPtr, 2, auMuxUnion.audioMuxSel);
//		sysmode = 3;
//	}
//
//}


//----------------------------------------------------
// - ODMK LED Manipulation
//----------------------------------------------------
void odmkZynqLED1(unsigned long uSECPERBEAT)
{

	//int Delay;
	static int ledD;
	ledD = ledFlash1[ledState];

	/* Write output to the LEDs. */
	XGpio_DiscreteWrite(&LEDSWSInst, LED_CHANNEL, ledD);

	/* Wait a small amount of time so that the LED blinking is visible. */
	//for (Delay = 0; Delay < CYCLESPERBEAT; Delay++);
	usleep(uSECPERBEAT);	// Wait 1 beat

	if (ledState == 3) {
		ledState = 0;
	} else {
		ledState++;
	}

}


/*------------------------------------------------------------------------------------------------*/
// - OLED Screen Functions


//----------------------------------------------------
// - convert float to character array
//----------------------------------------------------
//char *ftoa(char *buffer, float d, int precision) {
void ftoa(char *buffer, float d, int precision) {

	long wholePart = (long) d;

	// Deposit the whole part of the number.
	itoa(wholePart,buffer,10);

	// Now work on the fraction if we need one.
	if (precision > 0) {

		// locate the end of the string and insert a decimal point.
		char *endOfString = buffer;
		while (*endOfString != '\0') endOfString++;
		*endOfString++ = '.';

		// work on the fraction, be sure to turn any negative values positive.
		if (d < 0) {
			d *= -1;
			wholePart *= -1;
		}

		float fraction = d - wholePart;
		while (precision > 0) {

			// Multiple by ten and pull out the digit.
			fraction *= 10;
			wholePart = (long) fraction;
			*endOfString++ = '0' + wholePart;

			// Update the fraction and move on to the next digit.
			fraction -= wholePart;
			precision--;
		}

		// Terminate the string.
		*endOfString = '\0';
	}

   //return buffer;
}


//----------------------------------------------------
// - convert float to character array
//----------------------------------------------------
void odmkInfoScreen(void *oledrgbPtr, float bpm)
{
	char ch;
	char str2[5] = " bpm";
	char bpmStr[8];

	ftoa(bpmStr, bpm, 1);

	strcat(bpmStr, str2);

	/* output to screen */
	for (ch = 0; ch < 5; ch++) {
		OLEDrgb_DefUserChar(oledrgbPtr, ch, &rgbUserFont[ch*8]);
	}

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(0, 255, 0));	// red
	OLEDrgb_SetCursor(oledrgbPtr, 1, 1);
	OLEDrgb_PutString(oledrgbPtr, "::((o))::");

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(255,  0, 0)); // blue font
	OLEDrgb_SetCursor(oledrgbPtr, 2, 3);
	OLEDrgb_PutString(oledrgbPtr, "*ODMK*");

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12, 44)); // blue font
	OLEDrgb_SetCursor(oledrgbPtr, 1, 5);
	OLEDrgb_PutString(oledrgbPtr, bpmStr);

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12, 44));	//captain cobra grey
	OLEDrgb_SetCursor(oledrgbPtr, 1, 7);
	OLEDrgb_PutString(oledrgbPtr, "GORGULAN93");

}


//----------------------------------------------------
// - display Audio mux state on OLEDRGB
//----------------------------------------------------
// ***** pass pointer to oledrgb in arguments *****
void odmkAudioMuxScreen(void *oledrgbPtr, int muxSel)
{
	char ch;

	/* output to screen */
	for (ch = 0; ch < 5; ch++) {
		OLEDrgb_DefUserChar(oledrgbPtr, ch, &rgbUserFont[ch*8]);
	}

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB( 0, 255, 0));	// red
	OLEDrgb_SetCursor(oledrgbPtr, 1, 1);
	OLEDrgb_PutString(oledrgbPtr, "::((o))::");
	OLEDrgb_SetCursor(oledrgbPtr, 2, 4);
	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB( 255,  0, 0)); // blue font
	OLEDrgb_PutString(oledrgbPtr, "*ODMK*");

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB( 200, 200, 44));	//captain cobra grey
	OLEDrgb_SetCursor(oledrgbPtr, 1, 6);

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
	OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
	switch(muxSel) {
		case 0:
			printf("AudioMux screen: <<OSC SIN>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<OSC SIN>>");
			break;

		case 1:
			printf("AudioMux screen: <<OSC SAW>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<OSC SAW>>");
			break;

		case 2:
			printf("AudioMux screen: <<OSC SQR>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<OSC SQR>>");
			break;

		case 3:
			printf("AudioMux screen: <<OSC PWM>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<OSC PWM>>");
			break;

		case 4:
			printf("AudioMux screen: <<EXTERN>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<EXTERN>>");
			break;

		case 8:
			printf("AudioMux screen: <<DDL SIN>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL SIN>>");
			break;

		case 9:
			printf("AudioMux screen: <<DDL SAW>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL SAW>>");
			break;

		case 10:
			printf("AudioMux screen: <<DDL SQR>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL SQR>>");
			break;

		case 11:
			printf("AudioMux screen: <<DDL PWM>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL PWM>>");
			break;

		case 12:
			printf("AudioMux screen: <<DDL EXT>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL EXT>>");
			break;

		case 16:
			printf("AudioMux screen: <<SSB SIN>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB SIN>>");
			break;

		case 17:
			printf("AudioMux screen: <<SSB SAW>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB SAW>>");
			break;

		case 18:
			printf("AudioMux screen: <<SSB SQR>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB SQR>>");
			break;

		case 19:
			printf("AudioMux screen: <<SSB PWM>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB PWM>>");
			break;

		case 20:
			printf("AudioMux screen: <<SSB EXT>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB EXT>>");
			break;

		case 24:
			printf("AudioMux screen: <<FLT SAW>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT SAW>>");
			break;

		case 25:
			printf("AudioMux screen: <<FLT SQR>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT SQR>>");
			break;

		case 26:
			printf("AudioMux screen: <<FLT PWM>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT PWM>>");
			break;

		case 27:
			printf("AudioMux screen: <<FLT EXT>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT EXT>>");
			break;

		case 28:
			printf("AudioMux screen: <<FLT EXT>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT EXT>>");
			break;

		default:
			printf("AudioMux screen: <DEFAULT> MuxSel = %d\n\r",muxSel);
			OLEDrgb_PutString(oledrgbPtr, "<DEFAULT>");
			break;

	}
}


void odmkFaderMuxScreen(void *oledrgbPtr, int faderSel)
{
	char ch;

	/* output to screen */

	for (ch = 0; ch < 5; ch++) {
		OLEDrgb_DefUserChar(oledrgbPtr, ch, &rgbUserFont[ch*8]);
	}

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB( 0, 255, 0));	// red
	OLEDrgb_SetCursor(oledrgbPtr, 1, 1);
	OLEDrgb_PutString(oledrgbPtr, "::((o))::");
	OLEDrgb_SetCursor(oledrgbPtr, 2, 4);
	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB( 255,  0, 0)); // blue font
	OLEDrgb_PutString(oledrgbPtr, "*ODMK*");

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB( 200, 200, 44));	//captain cobra grey
	OLEDrgb_SetCursor(oledrgbPtr, 1, 6);

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
	OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
	switch(faderSel) {
		case 0:
			printf("FaderMux screen: <<VOL CTR>> MuxSel = %d\n\r",faderSel);
			OLEDrgb_PutString(oledrgbPtr, "<<VOL CTRL>>");
			break;

		case 1:
			printf("FaderMux screen: <<OSC FREQ>> MuxSel = %d\n\r",faderSel);
			OLEDrgb_PutString(oledrgbPtr, "<<OSC FREQ>>");
			break;

		case 2:
			printf("FaderMux screen: <<LFO FREQ>> MuxSel = %d\n\r",faderSel);
			OLEDrgb_PutString(oledrgbPtr, "<<LFO FREQ>>");
			break;

		case 3:
			printf("FaderMux screen: <<PWM FREQ>> MuxSel = %d\n\r",faderSel);
			OLEDrgb_PutString(oledrgbPtr, "<<PWM FREQ>>");
			break;

		case 4:
			printf("FaderMux screen: <<SSB FREQ>> MuxSel = %d\n\r",faderSel);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB FREQ>>");
			break;

		case 5:
			printf("FaderMux screen: <<FLT CUT>> MuxSel = %d\n\r",faderSel);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT CUT>>");
			break;

		case 6:
			printf("FaderMux screen: <<FLT RES>> MuxSel = %d\n\r",faderSel);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT RES>>");
			break;

	}

}


void odmkFader2MuxScreen(void *oledrgbPtr, int fader2Sel)
{
	char ch;

	/* output to screen */

	for (ch = 0; ch < 5; ch++) {
		OLEDrgb_DefUserChar(oledrgbPtr, ch, &rgbUserFont[ch*8]);
	}

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB( 0, 255, 0));	// red
	OLEDrgb_SetCursor(oledrgbPtr, 1, 1);
	OLEDrgb_PutString(oledrgbPtr, "::((o))::");
	OLEDrgb_SetCursor(oledrgbPtr, 2, 4);
	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB( 255,  0, 0)); // blue font
	OLEDrgb_PutString(oledrgbPtr, "*ODMK*");

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB( 200, 200, 44));	//captain cobra grey
	OLEDrgb_SetCursor(oledrgbPtr, 1, 6);

	OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
	OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
	switch(fader2Sel) {
		case 0:
			printf("Fader2Mux screen: <<DDL MIX>> MuxSel = %d\n\r", fader2Sel);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL MIX>>");
			break;

		case 1:
			printf("Fader2Mux screen: <<DDL LEN>> MuxSel = %d\n\r", fader2Sel);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL LEN>>");
			break;

		case 2:
			printf("Fader2Mux screen: <<DDL FB G>> MuxSel = %d\n\r", fader2Sel);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL FB G>>");
			break;

	}

}

/*------------------------------------------------------------------------------------------------*/

//----------------------------------------------------
// Moog Half Ladder Filter Parameter update
//----------------------------------------------------
/*
cutoff = 500, res = 1.707
setFcAndRes_ARM: g = 0.0327366,	G = 0.0316989,	K = 1.707,	beta1 = -0.0287482,	beta2 = -0.906913,	beta3 = 1.9366,	alpha0 = 1.00161


cutoff = 8600, res = 1.0
setFcAndRes_ARM: g = 0.630954,	G = 0.386862,	K = 1,	beta1 = -0.0536727,	beta2 = -0.138739,	beta3 = 1.22628,	alpha0 = 1.03505
*/


void setFcAndRes_ARM(float sampleRate, float cutoff, float resonance, struct moogHLParam_type *fParamSW)
{
	// prewarp for BZT
	double wd = 2*pi*cutoff;
	double T  = 1/(double)sampleRate;
	double wa = (2/T)*tan(wd*T/2);
	double g  = wa*T/2;

	// G - the feedforward coeff in the VA One Pole
	float G = g/(1.0 + g);

	// the allpass G value
	float GA = 2.0*G-1;

	// calculate alpha0
	// for 2nd order, K = 2 is max so limit it there
	float K = resonance;
	if(K > 2.0)
		K = 2.0;

	// parameter HLS type: ap_fixed<24,3>, therefore, scale by 2^21

//	fParamSW->beta1 = GA*G / (1.0 + g);
//	fParamSW->beta2 = GA / (1.0 + g);
//	fParamSW->beta3 = 2.0 / (1.0 + g);
//
//	fParamSW->K = K;
//	fParamSW->alpha = G;
//	fParamSW->alpha0 = 1.0 / (1.0 + K*GA*G*G);

//	printf("alpha = %f,  K = %f\n", fParamSW->alpha, fParamSW->K);
//	printf("alpha0 = %f,  beta1 = %f\n", fParamSW->alpha0, fParamSW->beta1);
//	printf("beta2 = %f,  beta3 = %f\n", fParamSW->beta2, fParamSW->beta3);

	fParamSW->beta1 = (int)( (GA*G / (1.0 + g)) * pow(2, 21) );
	fParamSW->beta2 = (int)( (GA / (1.0 + g)) * pow(2, 21) );
	fParamSW->beta3 = (int)( (2.0 / (1.0 + g)) * pow(2, 21) );

	fParamSW->K = (int)( K * pow(2, 21) );
	fParamSW->alpha = (int)( G * pow(2, 21) );
	fParamSW->alpha0 = (int)( (1.0 / (1.0 + K*GA*G*G)) * pow(2, 21) );

}



//----------------------------------------------------
// INITIAL SETUP FUNCTIONS
//----------------------------------------------------

void OLEDrgbInitialize()
{
	OLEDrgb_begin(&oledrgb, XPAR_PMODOLEDRGB_1_AXI_LITE_GPIO_BASEADDR, XPAR_PMODOLEDRGB_1_AXI_LITE_SPI_BASEADDR);
}


int InterruptSystemSetup(XScuGic *XScuGicInstancePtr, XGpio *GpioInstancePtr, int INT_CH_MASK)
{
	// Enable interrupt
	XGpio_InterruptEnable(GpioInstancePtr, INT_CH_MASK);
	XGpio_InterruptGlobalEnable(GpioInstancePtr);

	/* Connect the interrupt controller interrupt handler to the hardware
	* interrupt handling logic in the ARM processor. */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			 	 	 	 	 	 (Xil_ExceptionHandler)XScuGic_InterruptHandler,
			 	 	 	 	 	 XScuGicInstancePtr);

	/* Enable interrupts in the ARM */
	Xil_ExceptionEnable();

	return XST_SUCCESS;

}


int IntcGpioInitFunction(u16 DeviceId, XGpio *GpioInstancePtr, Xil_ExceptionHandler X_Intr_Handler, int INT_X_ID, int INT_CH_MASK)
{
	XScuGic_Config *IntcConfig;
	int status;

	// Interrupt controller driver initialization
	IntcConfig = XScuGic_LookupConfig(DeviceId);
	status = XScuGic_CfgInitialize(&INTCInst, IntcConfig, IntcConfig->CpuBaseAddress);
	if(status != XST_SUCCESS) return XST_FAILURE;

	// Call to interrupt setup
	//status = InterruptSystemSetup(&INTCInst, &BTNInst, INT_CH_MASK);
	status = InterruptSystemSetup(&INTCInst, GpioInstancePtr, INT_CH_MASK);
	if(status != XST_SUCCESS) return XST_FAILURE;

	// Connect GPIO interrupt to device driver handler
	status = XScuGic_Connect(&INTCInst,
							 INT_X_ID,
					  	  	 (Xil_ExceptionHandler)X_Intr_Handler,
					  	  	 (void *)GpioInstancePtr);


	if(status != XST_SUCCESS) return XST_FAILURE;

	// Enable GPIO interrupts interrupt
	XGpio_InterruptEnable(GpioInstancePtr, 1);
	XGpio_InterruptGlobalEnable(GpioInstancePtr);

	// Enable GPIO and timer interrupts in the controller
	XScuGic_Enable(&INTCInst, INT_X_ID);

	return XST_SUCCESS;
}


/*------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/
/*------------------------------------------------------------------------------------------------*/

int main(void)
{

	int status;

	init_platform();

	printf("::((Zedboard Audio System))::\n\r");


	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // LED & Switch AXI GPIO Initialization
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Initialize Push Buttons
    status = XGpio_Initialize(&BTNInst, GPIO_BTNS_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;
    // Set all buttons direction to inputs
    XGpio_SetDataDirection(&BTNInst, 1, 0xFF);

    // Initialize LEDs & Switches
    status = XGpio_Initialize(&LEDSWSInst, GPIO_LED_SW_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;
    // Set LEDs direction to outputs, SW to inputs
    XGpio_SetDataDirection(&LEDSWSInst, 1, 0x00);
    XGpio_SetDataDirection(&LEDSWSInst, 2, 0xFF);

    // Initialize Push Buttons interrupt controller
    INTC_X_INTERRUPT_ID = INTC_BTNS_INTERRUPT_ID;
    INT_CH_MASK = BTN_CH_MASK;
    status = IntcGpioInitFunction(INTC_DEVICE_ID, &BTNInst, &BTN_Intr_Handler, INTC_X_INTERRUPT_ID, INT_CH_MASK);
    if(status != XST_SUCCESS) return XST_FAILURE;


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // AXI Timer Initialization
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//    status = XTmrCtr_Initialize(&TIMERInst, TMRCTR_TIMER_ID);
//	if(XST_SUCCESS != status)
//	print("TIMER INIT FAILED \n\r");
//	// Set Timer Handler
//	XTmrCtr_SetHandler(&TIMERInst, TIMER_Intr_Handler, &TIMERInst);
//	//XTmrCtr_SetHandler(&TIMERInst, &XTmrCtr_InterruptHandler, &TIMERInst);
//	// Set timer Reset Value
//	XTmrCtr_SetResetValue(&TIMERInst, 0, 0xf0000000);	//Change with generic value
//	// Set timer Option (Interrupt Mode And Auto Reload )
//	XTmrCtr_SetOptions(&TIMERInst, TMRCTR_TIMER_ID, (XTC_INT_MODE_OPTION | XTC_AUTO_RELOAD_OPTION ));

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Initialize Oscillators
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Initialize DDS Freq1 & SSB setStep Gpio
    status = XGpio_Initialize(&OSCF1SSBStepInst, GPIO_OSCF1_SSBF1_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Initialize DDS Freq2 & PWM setStep Gpio
    status = XGpio_Initialize(&OSCF2PWMStepInst, GPIO_OSCF2_PWM_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Initialize Filters
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Initialize Moog Half Ladder CTRL 1 Gpio
    status = XGpio_Initialize(&MHLALPHAKInst, GPIO_MHL_ALPHA_K_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Initialize Moog Half Ladder CTRL 1 Gpio
    status = XGpio_Initialize(&MHLCTRL1Inst, GPIO_MHL_PARAM1_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Initialize Moog Half Ladder CTRL 2 Gpio
    status = XGpio_Initialize(&MHLCTRL2Inst, GPIO_MHL_PARAM2_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Initialize Delay
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Initialize DDL3 CTRL Gpio
    status = XGpio_Initialize(&DDL3CTRLInst, GPIO_DDL3CTRL_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Initialize ..others..
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    // Initialize Volume Control Gpio
    status = XGpio_Initialize(&VOLCTRLInst, GPIO_VOLCTRL_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Initialize Audio Mux Gpio
    status = XGpio_Initialize(&AUDIOMUXInst, GPIO_AUDIOMUX_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Initialize Rotary Encoder 1 Gpio
    status = XGpio_Initialize(&ROTARY1Inst, GPIO_ROTARY1_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Initialize Rotary Encoder 1 interrupt controller
    INTC_X_INTERRUPT_ID = INTC_ROTARY1_INTERRUPT_ID;
    INT_CH_MASK = ROTARY1_INT;
    status = IntcGpioInitFunction(INTC_DEVICE_ID, &ROTARY1Inst, &ROTARY1_Intr_Handler, INTC_X_INTERRUPT_ID, INT_CH_MASK);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Initialize Rotary Encoder 2 Gpio
    status = XGpio_Initialize(&ROTARY2Inst, GPIO_ROTARY2_ID);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Initialize Rotary Encoder 1 interrupt controller
    INTC_X_INTERRUPT_ID = INTC_ROTARY2_INTERRUPT_ID;
    INT_CH_MASK = ROTARY2_INT;
    status = IntcGpioInitFunction(INTC_DEVICE_ID, &ROTARY2Inst, &ROTARY2_Intr_Handler, INTC_X_INTERRUPT_ID, INT_CH_MASK);
    if(status != XST_SUCCESS) return XST_FAILURE;

    // Initialize OLEDrgb display
    OLEDrgbInitialize();


	Xil_ICacheEnable();
	Xil_DCacheEnable();


    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // INITIALIZE Power-on System Behavior
    //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

	// 1us = 100000

	BPM = 133.0;	// Default BPM
	uSECPERBEAT = (unsigned long)(1000000*60/BPM);

	printf("BPM = %f\n", BPM);
	//printf("uSec per beat = %d\n\r", uSECPERBEAT);
	odmkInfoScreen(&oledrgb, BPM);

	XGpio_DiscreteWrite(&VOLCTRLInst, 1, volCtrlVal);

	XGpio_DiscreteWrite(&OSCF1SSBStepInst, 1, oscFreq1StepVal);
	XGpio_DiscreteWrite(&OSCF1SSBStepInst, 2, ssbSetStepVal);

	XGpio_DiscreteWrite(&OSCF2PWMStepInst, 1, oscFreq2StepVal);
	XGpio_DiscreteWrite(&OSCF2PWMStepInst, 2, oscPWMStepVal);

	// DDL3 INIT
	DDL3CTRL1Union.DDL3CTRL1.delayLength = 8600;
	DDL3CTRL1Union.DDL3CTRL1.dLengthFlag = 1;
	DDL3CTRL1Union.DDL3CTRL1.wetMix = 16384;				// 0.5<<14;
	XGpio_DiscreteWrite(&DDL3CTRLInst, 1, DDL3CTRL1Union.DDL3CTRL_CH1Val);
	DDL3CTRL1Union.DDL3CTRL1.dLengthFlag = 0;
	XGpio_DiscreteWrite(&DDL3CTRLInst, 1, DDL3CTRL1Union.DDL3CTRL_CH1Val);

	DDL3CTRL2Union.DDL3CTRL2.feedbackGain = 11469;		// ~0.7<<14 (11468.8)
	DDL3CTRL2Union.DDL3CTRL2.selectExtFB = 0;
	DDL3CTRL2Union.DDL3CTRL2.fbMuxSel = 0;
	XGpio_DiscreteWrite(&DDL3CTRLInst, 2, DDL3CTRL2Union.DDL3CTRL_CH2Val);

	// MOOG Half Ladder Filter INIT
	setFcAndRes_ARM(fs, mooghlCutoff, 0.0, &mooghlParam);
	XGpio_DiscreteWrite(&MHLALPHAKInst, 1, mooghlParam.alpha);
	XGpio_DiscreteWrite(&MHLALPHAKInst, 2, mooghlParam.K);
	XGpio_DiscreteWrite(&MHLCTRL1Inst, 1, mooghlParam.alpha0);
	XGpio_DiscreteWrite(&MHLCTRL1Inst, 2, mooghlParam.beta1);
	XGpio_DiscreteWrite(&MHLCTRL2Inst, 1, mooghlParam.beta2);
	XGpio_DiscreteWrite(&MHLCTRL2Inst, 2, mooghlParam.beta3);

	printf("MOOG Half Ladder Initial Parameters:\n");
	printf("moog-hl Cutoff = %6.2f,   moog-hl Resonance = %2.2f\n", mooghlCutoff, mooghlRes);
	//printf("alpha = %i,  K = %i\n", mooghlParam.alpha, mooghlParam.K);
	//printf("alpha0 = %i,  beta1 = %i\n", mooghlParam.alpha0, mooghlParam.beta1);
	//printf("beta2 = %i,  beta3 = %i\n", mooghlParam.beta2, mooghlParam.beta3);


	/*------------------------------------------------------------------------------------------------*/

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Begin Master Routine
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	while(1) {

		if (sysmode == 0) {
			/* Write Info to OLEDRGB - center button */

			if (trigOnce == 1) {

				btnDirection = !btnDirection;

				oscFreq1 = 100000000.0 * (float)oscFreq1StepVal / pow(2, hirezPhaseAccWidth);
				oscFreq2 = 100000000.0 * (float)oscFreq2StepVal / pow(2, lorezPhaseAccWidth);

				printf("::((ODMK Zedboard Audio System v1.4))::\n");
				printf("sysmode = %d:\n",sysmode);
				printf("Zedboard Button Function:\n");
				printf("Center: Top State <sysmode == 0>\n");
				printf("Bottom: Go (seq) State <sysmode == 1>\n");
				printf("Left: Switch Update <sysmode == 2>\n");
				printf("Right: Audio Source Select <sysmode == 3>\n");
				printf("Top: Fader Select <sysmode == 4>\n");
				printf("BPM = %3.2f\n", BPM);
				printf("BTN Direction {0:fwd, 1:rev} = %d\n", (int)btnDirection);
				printf("volCtrlDB = %5.2f,   volCtrlVal = %d\n", volCtrlDB, (int)volCtrlVal);
				printf("OSC Freq1 = %5.2f,   OSC Freq1 (LFO) = %5.2f\n", oscFreq1, oscFreq2);
				printf("oscFreq1StepVal = %d,   oscFreq2StepVal = %d\n", oscFreq1StepVal, oscFreq2StepVal);
				printf("moog-hl Cutoff = %6.2f,   moog-hl Resonance = %2.2f\n", mooghlCutoff, mooghlRes);
				printf("wetMixFloat = %2.2f,   wetMix = %d\n", wetMixFloat, (int)DDL3CTRL1Union.DDL3CTRL1.wetMix);
				printf("delayLength = %d\n", (int)DDL3CTRL1Union.DDL3CTRL1.delayLength);
				printf("feedbackGainFloat = %3.2f,   feedbackGain = %d\n", feedbackGainFloat, (int)DDL3CTRL2Union.DDL3CTRL2.feedbackGain);

				OLEDrgb_Clear(&oledrgb);
				odmkInfoScreen(&oledrgb, BPM);
				trigOnce = 0;

				usleep(3300000);	//Wait a moment, then switch to chebyshev poly's
			}

			odmkZynqLED1(uSECPERBEAT);		// *** may have to adjust timing for bpm sync???

			OLEDrgb_DrawBitmap(&oledrgb,0,0,95,63, (u8*)chbyshvIcon01_96x64);
			odmkZynqLED1(uSECPERBEAT-oledTempoLag);	// Wait 1 beat

			OLEDrgb_DrawBitmap(&oledrgb,0,0,95,63, (u8*)chbyshvIcon02_96x64);
			odmkZynqLED1(uSECPERBEAT-oledTempoLag);	// Wait 1 beat

			OLEDrgb_DrawBitmap(&oledrgb,0,0,95,63, (u8*)chbyshvIcon03_96x64);
			odmkZynqLED1(uSECPERBEAT-oledTempoLag);	// Wait 1 beat

			OLEDrgb_DrawBitmap(&oledrgb,0,0,95,63, (u8*)chbyshvIcon04_96x64);
			odmkZynqLED1(uSECPERBEAT-oledTempoLag);	// Wait 1 beat

			OLEDrgb_DrawBitmap(&oledrgb,0,0,95,63, (u8*)chbyshvIcon05_96x64);
			odmkZynqLED1(uSECPERBEAT-oledTempoLag);	// Wait 1 beat

			OLEDrgb_DrawBitmap(&oledrgb,0,0,95,63, (u8*)chbyshvIcon06_96x64);
			odmkZynqLED1(uSECPERBEAT-oledTempoLag);	// Wait 1 beat

			//OLEDrgb_DrawBitmap(&oledrgb,0,0,95,63, (u8*)masakiKmno01);
			//usleep(uSECPERBEAT-oledTempoLag);	// Wait 1 beat

			// wait for button interrupt to change state

		}

		else if (sysmode == 1) {
			/* increment fader 1 mux select - bottom button */
			if (trigOnce == 1) {
				printf("sysmode {%d}:   Fader 2 Mux Select = %d\n",sysmode, (int)fader2MuxSel);
				OLEDrgb_Clear(&oledrgb);
				odmkFader2MuxScreen(&oledrgb, fader2MuxSel);
				trigOnce = 0;
			}

			// wait for button interrupt to change state
		}


		else if (sysmode == 2) {
			/* update Audio Mux Routing - left button */
			//auMuxUnion.audioSelect.audioMux3_0 = (u8)(3&swsUnion.swValue);

			printf("sysmode = %d:   Switch System Update\n",sysmode);
			printf("All Switch - swsUnion.swValue = %d\n",(int)swsUnion.swValue);
			printf("Switches[1:8]:   %d %d %d %d %d %d %d %d\n",swsUnion.switches.sw8,
					swsUnion.switches.sw7,
					swsUnion.switches.sw6,
					swsUnion.switches.sw5,
					swsUnion.switches.sw4,
					swsUnion.switches.sw3,
					swsUnion.switches.sw2,
					swsUnion.switches.sw1);

			// auto switch back to Go State
			trigOnce = 1;
			sysmode=1;
		}

		else if (sysmode == 3) {
			/* increment audio mux select - right button */
			if (trigOnce==1) {
				printf("sysmode = %d:   Increment Audio Mux Select\n",sysmode);
				/* Update OLED display */
				OLEDrgb_Clear(&oledrgb);
				odmkAudioMuxScreen(&oledrgb, auMuxUnion.audioMuxSel);
			}

			trigOnce = 0;
			//sysmode=1;
			// wait for button interrupt to change state
		}

		else if (sysmode == 4) {
			/* increment fader 1 mux select - top button */
			if (trigOnce==1) {
				printf("sysmode = %d:   Fader 1 Mux Select = %d\n",sysmode, (int)faderMuxSel);
				/* Update OLED display */
				OLEDrgb_Clear(&oledrgb);
				odmkFaderMuxScreen(&oledrgb, faderMuxSel);
				if (faderMuxSel == 5 || faderMuxSel == 6) {
					printf("MOOG Half Ladder Parameters:\n");
					printf("moog-hl Cutoff = %6.2f,   moog-hl Resonance = %2.2f\n", mooghlCutoff, mooghlRes);
					//printf("alpha = %i,  K = %i\n", mooghlParam.alpha, mooghlParam.K);
					//printf("alpha0 = %i,  beta1 = %i\n", mooghlParam.alpha0, mooghlParam.beta1);
					//printf("beta2 = %i,  beta3 = %i\n", mooghlParam.beta2, mooghlParam.beta3);
				}
			}

			trigOnce = 0;
			//sysmode=1;
			// wait for button interrupt to change state
		}

	}

	cleanup_platform();

	return 0;
}
