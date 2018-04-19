/*------------------------------------------------------------------------------------------------*/
/* ___::((odmkZedSystem1_4.c))::___

   ___::((JIROBATA Programming Industries))::___
   ___::((ODMK:2018))::___
   ___::((created by eschei))___

	Purpose: ARM software for odmkZyncSystem1
	Device: zedboard - xc7z020clg484-1
	Revision History: May 16, 2017 - initial
	Revision History: Feb 03, 2018 - version 1_4
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
#define GPIO_BTNS_ID  				XPAR_AXI_GPIO_0_DEVICE_ID
#define INTC_BTNS_INTERRUPT_ID 		XPAR_FABRIC_AXI_GPIO_0_IP2INTC_IRPT_INTR

//#define ROTARY1_DEVICE_ID
#define GPIO_ROTARY1_ID				XPAR_AXI_GPIO_1_DEVICE_ID
#define INTC_ROTARY1_INTERRUPT_ID 	XPAR_FABRIC_AXI_GPIO_1_IP2INTC_IRPT_INTR

//#define LEDS_SWS_DEVICE_ID
// DUAL: ?LSB[7:0]=LED, MSB[7:0]=SW
#define GPIO_LED_SW_ID  			XPAR_AXI_GPIO_2_DEVICE_ID

//#define OSC4T Freq1 DEVICE_ID (single channel)
#define GPIO_OSCF1_SSBF1_ID 		XPAR_AXI_GPIO_3_DEVICE_ID

//#define OSC4T Freq2 & PWM DEVICE_ID (dual channel)
// DUAL: ?LSB[27:0]=DDS FREQ2, MSB[27:0]=PWM FREQ
#define GPIO_OSCF2_PWM_ID 			XPAR_AXI_GPIO_4_DEVICE_ID

//#define AUDIOMUX_DEVICE_ID
#define GPIO_AUDIOMUX_ID 			XPAR_AXI_GPIO_5_DEVICE_ID

//#define MHL_ALPHA_K_DEVICE_ID
// DUAL: ?LSB[27:0]=ALPHA (G - cutoff), MSB[27:0]=K (resonance)
#define GPIO_MHL_ALPHA_K_ID 		XPAR_AXI_GPIO_6_DEVICE_ID

//#define VOLCTRL_DEVICE_ID
#define GPIO_VOLCTRL_ID 			XPAR_AXI_GPIO_7_DEVICE_ID

//#define DDL3CTRL_DEVICE_ID
#define GPIO_DDL3CTRL_ID 			XPAR_AXI_GPIO_8_DEVICE_ID

//#define MHL_PARAM1_DEVICE_ID
// DUAL: ?LSB[23:0]=ALPHA0, MSB[23:0]=Beta1
#define GPIO_MHL_PARAM1_ID 			XPAR_AXI_GPIO_9_DEVICE_ID

//#define MHL_PARAM2_DEVICE_ID
// DUAL: ?LSB[23:0]=Beta2, MSB[23:0]=Beta3
#define GPIO_MHL_PARAM2_ID 			XPAR_AXI_GPIO_10_DEVICE_ID


/*------------------------------------------------------------------------------------------------*/
/* instantiate objects */
XScuGic INTCInst;
XTmrCtr TIMERInst;
PmodOLEDrgb oledrgb;
XGpio LEDSWSInst;
XGpio BTNInst;
XGpio AUDIOMUXInst;
XGpio ROTARY1Inst;
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

const float fs = 48000;

// OLEDrgb_DrawBitmap function delay (approximate! - needs accurate tuning)
// *empirically measured tempo lag + included usleep(5000) func call
const unsigned long oledTempoLag = 50000 + 5000;

const float mstrVolScale = 0.003;
const float filtVolScale = 2.7;
const int oscFreqScale = 560;
const int lfoFreqScale = 1;
const int ssbFreqScale = 756000;
const int mooghlCutoffScale = 56;
//const int mooghlResScale = 0.03;

const int ledFlash1[] = {24, 36, 66, 129};		// center out LED strobe pattern

/*------------------------------------------------------------------------------------------------*/
/* (((Static Variables))) */

static int INT_CH_MASK = XGPIO_IR_CH1_MASK;
static int INTC_X_INTERRUPT_ID = INTC_BTNS_INTERRUPT_ID;	// arbitrarily defaults to button interrupt ID

static float BPM = 133.0;	// Default BPM
static unsigned long uSECPERBEAT = 451128;		// 1000000*60/BPM - default initial value assuming 133 bpm

static u8 audioMux3_prev = 0;

static float mooghlCutoff = 5000;
static float mooghlRes = 0.23;

//static int ledData = 0;
static int ledState = 0;	// used for LED manipulation function
static bool trigOnce = 1;

static int btnValue;
static bool btnDirection = 1;

// fader Mux select
static u8 faderMuxSel = 0;
static int sysmode = 0;	// 0=>OLED info ; 1=>OLED imgRotate ; 2=>?

static int rotary_AB;

static double volCtrlDB = 0.707;	// audio out volume control { 0 - 1 }
static unsigned int volCtrlVal = 560;		// audio out volume control val { 0 - 4095 }, default = -19.3dB


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

// audioMux3_0: {1,0} = output audio MUX sel
// 4 = sin SSB   5=sqr SSB   6=pwm SSB   7=ext SSB

// audioMux5_0: {4,2} = input audio MUX sel
// 0 = sin noSSB 1=sqr noSSB 2=pwm noSSB 3=ext noSSB

struct auMux_type {
	u8 audioMux5_0: 3;
	u8 audioMux3_0: 2;
};

union auMux_u {
	u8 audioMuxSel;
	struct auMux_type audioSelect;
};

static union auMux_u auMuxUnion;


/*------------------------------------------------------------------------------------------------*/
//  __OSC4T__
//
// double oscOutFreq = 300000.0;
// oscStep = oscOutFreq * pow(2, hirezPhaseAccWidth) / Fs;
//
// 560 Hz osc output 384829 (hex: 0x00005DF3D)
// 560 / 2 : 192414 (hex: 0x00002EF9E)
//static int oscFreq1StepVal = 192414;
static int oscFreq1StepVal = 38483;		// ~ 56 Hz
static int oscFreq2StepVal = 777;		// ~ 1.131 Hz
static int oscPWMStepVal = 3848;		// ~ 5.6 Hz

static int ssbSetStepVal = 11000093;	// SSM Modulation frequency

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
// define struct for Moog Half-Ladder Filter parameters

// ap_fixed<24,3>

struct moogHLParam_type {
//	float K;
//	float alpha0;
//	float alpha;
//	float beta1;
//	float beta2;
//	float beta3;
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

// DDL3 CH1
struct DDL3CTRL1_type {
	u16 delayLength;
	bool dLengthFlag;
	u16 wetMix;
};

union DDL3CTRL1_u {
	u32 DDL3CTRL_CH1Val;
	struct DDL3CTRL1_type DDL3CTRL1;
};

static union DDL3CTRL1_u DDL3CTRL1Union;


// DDL3 CH2
struct DDL3CTRL2_type {
	u16 feedbackGain;
	bool selectExtFB;
	bool fbMuxSel;
};

union DDL3CTRL2_u {
	u32 DDL3CTRL_CH2Val;
	struct DDL3CTRL2_type DDL3CTRL2;
};

static union DDL3CTRL2_u DDL3CTRL2Union;


/*
ap_uint<1> setDlengthFlag = 0;
delay_t dlyLength = 500;		// delay_t - ap_uint<DLYBWIDTH>
ap_uint<1> selectExtFB = 0;
ctrl_t wetMix = 0.5;			// ctrl_t - ap_fixed<16,2> => 0.5 << 14 = 8192
ctrl_t feedbackGain = 0.7;

DDL3CTRL1Union.DDL3CTRL1.delayLength = 500;

*/

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
static int InterruptSystemSetup(XScuGic *XScuGicInstancePtr, XGpio *GpioInstancePtr, int INT_CH_MASK);
int IntcGpioInitFunction(u16 DeviceId, XGpio *GpioInstancePtr, Xil_ExceptionHandler X_Intr_Handler, int INT_X_ID, int INT_CH_MASK);
//void audioMux3_switch(void *audioMuxGpioInstPtr, u8 auMux3sel);
void odmkZynqLED1(unsigned long uSECPERBEAT);
void ftoa(char *buffer, float d, int precision);
void odmkInfoScreen(void *oledrgbPtr, float bpm);
void odmkAudioMuxScreen(void *oledrgbPtr, int muxSel);
void odmkFaderMuxScreen(void *oledrgbPtr, int faderSel);
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
		printf("BTN[%d] Go State\n", btnValue);
		sysmode = 1;
		trigOnce = 1;
	}
	// Left Button pressed
	else if(btnValue == 4) {
		printf("BTN[%d] Update SW State\n", btnValue);
		sysmode = 2;
		trigOnce = 0;
		swsUnion.swValue = XGpio_DiscreteRead(&LEDSWSInst, 2);
		auMuxUnion.audioSelect.audioMux3_0 = (u8)(3&swsUnion.swValue);
		XGpio_DiscreteWrite(&AUDIOMUXInst, 1, auMuxUnion.audioMuxSel);
		if (audioMux3_prev != 3 && auMuxUnion.audioSelect.audioMux3_0 == 3) {
			printf("Enabled Filter Gain Compensation\n");
			//volCtrlDB+=filtVolScale;
			volCtrlDB+=2.7;
			if (volCtrlDB >= 140.0) {
				volCtrlDB = 140;	//volCtrlVal = 0;
			}
			volCtrlVal = (unsigned int)4095*(pow(10, -volCtrlDB/20));
			XGpio_DiscreteWrite(&VOLCTRLInst, 1, volCtrlVal);
		} else if(audioMux3_prev == 3 && auMuxUnion.audioSelect.audioMux3_0 != 3) {
			printf("Disabled Filter Gain Compensation\n");
			//volCtrlDB-=filtVolScale;
			volCtrlDB-=2.7;
			if (volCtrlDB <= 0.0) {
				volCtrlDB = 0;	//volCtrlVal = 4095;
			}
			volCtrlVal = (unsigned int)4095*(pow(10, -volCtrlDB/20));
			XGpio_DiscreteWrite(&VOLCTRLInst, 1, volCtrlVal);
		}
		audioMux3_prev = auMuxUnion.audioSelect.audioMux3_0;
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

	usleep(500000);	//Wait 1/1000 seconds - button debounce

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

		// Audio Output volume control {0 : -96 dB}
		// Saturate volume at max 0 and min -96 dB
		// convert dB to 12 bit integer values {0:4095}
		if(rotary_AB == 0) {
			//printf("Rotary Enc -Left- rotary_AB = %d\n\r",rotary_AB);
			volCtrlDB-=mstrVolScale;
			if (volCtrlDB <= 0.0) {
				volCtrlDB = 0;	//volCtrlVal = 4095;
			}
			volCtrlVal = (unsigned int)4095*(pow((double)10, -volCtrlDB/20));
			XGpio_DiscreteWrite(&VOLCTRLInst, 1, volCtrlVal);
		}
		else if(rotary_AB == 2) {
			//printf("Rotary Enc -Right- rotary_AB = %d\n\r",rotary_AB);
			volCtrlDB+=mstrVolScale;
			if (volCtrlDB >= 140.0) {
				volCtrlDB = 140;	//volCtrlVal = 0;
			}
			volCtrlVal = (unsigned int)4095*(pow(10, -volCtrlDB/20));
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
			mooghlRes-=0.03;
			if (mooghlRes <= 0.0) {
				mooghlRes = 0.0;
			}
			printf("Rotary Intr Handler - mooghlRes = %f\n", mooghlRes);
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
			mooghlRes+=0.03;
			if (mooghlRes >= 2.0) {
				mooghlRes = 2.0;
			}
			printf("Rotary Intr Handler - mooghlRes = %f\n", mooghlRes);
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

	switch(muxSel) {
		case 0:
			printf("AudioMux screen: <<OSC SIN>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<OSC SIN>>");
			break;

		case 1:
			printf("AudioMux screen: <<OSC SAW>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<OSC SAW>>");
			break;

		case 2:
			printf("AudioMux screen: <<OSC SQR>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<OSC SQR>>");
			break;

		case 3:
			printf("AudioMux screen: <<OSC PWM>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<OSC PWM>>");
			break;

		case 4:
			printf("AudioMux screen: <<EXTERN>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<EXTERN>>");
			break;

		case 8:
			printf("AudioMux screen: <<DDL SIN>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL SIN>>");
			break;

		case 9:
			printf("AudioMux screen: <<DDL SAW>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL SAW>>");
			break;

		case 10:
			printf("AudioMux screen: <<DDL SQR>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL SQR>>");
			break;

		case 11:
			printf("AudioMux screen: <<DDL PWM>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL PWM>>");
			break;

		case 12:
			printf("AudioMux screen: <<DDL EXT>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<DDL EXT>>");
			break;

		case 16:
			printf("AudioMux screen: <<SSB SIN>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB SIN>>");
			break;

		case 17:
			printf("AudioMux screen: <<SSB SAW>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB SAW>>");
			break;

		case 18:
			printf("AudioMux screen: <<SSB SQR>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB SQR>>");
			break;

		case 19:
			printf("AudioMux screen: <<SSB PWM>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB PWM>>");
			break;

		case 20:
			printf("AudioMux screen: <<SSB EXT>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB EXT>>");
			break;

		case 24:
			printf("AudioMux screen: <<FLT SAW>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT SAW>>");
			break;

		case 25:
			printf("AudioMux screen: <<FLT SQR>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT SQR>>");
			break;

		case 26:
			printf("AudioMux screen: <<FLT PWM>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT PWM>>");
			break;

		case 27:
			printf("AudioMux screen: <<FLT EXT>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT EXT>>");
			break;

		case 28:
			printf("AudioMux screen: <<FLT EXT>> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT EXT>>");
			break;

		default:
			printf("AudioMux screen: <DEFAULT> MuxSel = %d\n\r",muxSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
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

	switch(faderSel) {
		case 0:
			printf("FaderMux screen: <<VOL CTR>> MuxSel = %d\n\r",faderSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<VOL CTRL>>");
			break;

		case 1:
			printf("FaderMux screen: <<OSC FREQ>> MuxSel = %d\n\r",faderSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<OSC FREQ>>");
			break;

		case 2:
			printf("FaderMux screen: <<LFO FREQ>> MuxSel = %d\n\r",faderSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<LFO FREQ>>");
			break;

		case 3:
			printf("FaderMux screen: <<PWM FREQ>> MuxSel = %d\n\r",faderSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<PWM FREQ>>");
			break;

		case 4:
			printf("FaderMux screen: <<SSB FREQ>> MuxSel = %d\n\r",faderSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<SSB FREQ>>");
			break;

		case 5:
			printf("FaderMux screen: <<FLT CUT>> MuxSel = %d\n\r",faderSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT CUT>>");
			break;

		case 6:
			printf("FaderMux screen: <<FLT RES>> MuxSel = %d\n\r",faderSel);
			OLEDrgb_SetFontColor(oledrgbPtr, OLEDrgb_BuildRGB(200, 12,44));
			OLEDrgb_SetCursor(oledrgbPtr, 0, 7);
			OLEDrgb_PutString(oledrgbPtr, "<<FLT RES>>");
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
	OLEDrgb_begin(&oledrgb, XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR, XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR);
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

    // Initialize Volume Control Gpio
    status = XGpio_Initialize(&VOLCTRLInst, GPIO_VOLCTRL_ID);
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
	DDL3CTRL1Union.DDL3CTRL1.delayLength = 5000;
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
	printf("Input Cutoff = %f,  Input Resonance = %f\n", mooghlCutoff, 0.0);
	printf("alpha = %i,  K = %i\n", mooghlParam.alpha, mooghlParam.K);
	printf("alpha0 = %i,  beta1 = %i\n", mooghlParam.alpha0, mooghlParam.beta1);
	printf("beta2 = %i,  beta3 = %i\n", mooghlParam.beta2, mooghlParam.beta3);


	/*------------------------------------------------------------------------------------------------*/

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Begin Master Routine
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	while(1) {

		if (sysmode == 0) {
			/* Write Info to OLEDRGB - center button */

			if (trigOnce == 1) {

				btnDirection = !btnDirection;

				printf("::((ODMK Zedboard Audio System v1.4))::\n");
				printf("sysmode = %d:\n",sysmode);
				printf("Zedboard Button Function:\n");
				printf("Center: Top State <sysmode == 0>\n");
				printf("Bottom: Go (seq) State <sysmode == 1>\n");
				printf("Left: Switch Update <sysmode == 2>\n");
				printf("Right: Audio Source Select <sysmode == 3>\n");
				printf("Top: Fader Select <sysmode == 4>\n");
				printf("BPM = %f\n", BPM);
				printf("BTN Direction = %d\n", (int)btnDirection);

				OLEDrgb_Clear(&oledrgb);
				odmkInfoScreen(&oledrgb, BPM);
				trigOnce = 0;
			}
			odmkZynqLED1(uSECPERBEAT);

			// wait for button interrupt to change state

		}

		else if (sysmode == 1) {
			/* Go State */
			/* Draw image to OLEDrgb display - bottom button */
			/* func OLEDrgb_DrawBitmap includes a 5000 uSec delay, so subtract from uSECPERBEAT */

			if (trigOnce == 1) {
				printf("sysmode = %d:   Go State (Tempo Flash LEDs)\n",sysmode);
			}
				trigOnce = 0;

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


		else if (sysmode == 2) {
			// update Audio MSTR Mux
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
			/* increment fader mux select - top button */
			if (trigOnce==1) {
				printf("sysmode = %d:   Fader Mux Select = %d\n",sysmode, (int)faderMuxSel);
				/* Update OLED display */
				OLEDrgb_Clear(&oledrgb);
				odmkFaderMuxScreen(&oledrgb, faderMuxSel);
				if (faderMuxSel == 5 || faderMuxSel == 6) {
					printf("MOOG Half Ladder Parameters:\n");
					printf("Input Cutoff = %f,  Input Resonance = %f\n", mooghlCutoff, mooghlRes);
					printf("alpha = %i,  K = %i\n", mooghlParam.alpha, mooghlParam.K);
					printf("alpha0 = %i,  beta1 = %i\n", mooghlParam.alpha0, mooghlParam.beta1);
					printf("beta2 = %i,  beta3 = %i\n", mooghlParam.beta2, mooghlParam.beta3);
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
