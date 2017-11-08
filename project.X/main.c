/*
 * File:   main.c
 * Author: mccda
 *
 * Created on October 10, 2017, 1:18 PM
 */

#pragma config ICS = PGx3 //this allows debugging.... use PGC/D pair 3 (pins 9,10) for debugging
#pragma config FNOSC = FRCDIV // select 8MHz oscillator with postscaler

#include "xc.h"
#include "libpic30.h" //include library for _delay_ms() function)

// Create a set of possible states
enum {Looking_For_Dispencer, Moving_Tward_Dispencer, Getting_Balls, Moving_Away_From_Dispencer, Finding_Goal, Shooting} state;

void initInputOutput();
void initDigitalPorts();
void initAnalogPorts();
void initPwmPorts();
void initInterupts();


// Change Notification Interrupt Service Routine (ISR)
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void)
{
    _CNIF = 0; // Clear interrupt flag (IFS1 register)
    
    switch (state){
        case Looking_For_Dispencer:
            break;
        case Moving_Tward_Dispencer:
            break;
        case Getting_Balls:
            break;
        case Moving_Away_From_Dispencer:
            break;
        case Finding_Goal:
            break;
        case Shooting:
            break;
        default:
            break;
    }
}

// Timer1 Interrupt Service Routine (ISR)
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void)
{
    _T1IF = 0; // Clear interrupt flag
    
    switch (state){
        case Looking_For_Dispencer:
            break;
        case Moving_Tward_Dispencer:
            break;
        case Getting_Balls:
            break;
        case Moving_Away_From_Dispencer:
            break;
        case Finding_Goal:
            break;
        case Shooting:
            break;
        default:
            break;
    }
    
}



int main(void) {
    
    _RCDIV = 0b010; // postscale oscillator (divide-by-4))

    /* PIC 24 PINOUT
    Pin 1/RA5: --
    Pin 2/RA0: digital output for LED
    Pin 3/RA1: digital output for Launch feeder
    Pin 4/RB0: PWM (Turret motor)
    Pin 5/RB1: Launch motor digital output/PWM
    Pin 6/RB2: Analog for ROS
    Pin 7/RA2: Analog input for Light sensor 1
    Pin 8/RA3: Touch sensor(s) input
    Pin 9/RB4: --
    Pin 10/RA4: --
    Pin 11/RB7: Analog input for Light sensor 2
    Pin 12/RB8: --
    Pin 13/RB9: --
    Pin 14/RA6: PWM for H-Bridge
    Pin 15/RB12: --
    Pin 16/RB13: Dir 1
    Pin 17/RB14: Sleep
    Pin 18/RB15: Dir 2
    Pin 19/VSS: Vss
    Pin 20/VDD: Vdd
     * */
    
    initInputOutput();
    initDigitalPorts();
    initInterupts();
    initPwmPorts();
    initAnalogPorts();

    state = Looking_For_Dispencer;
    
    while(1){
        
        switch (state){
            case Looking_For_Dispencer:
                //Rotate base to look for IR emitter
                //Done when IR receiver emitter interrupts
                break;
            case Moving_Tward_Dispencer:
                //Set direction of both base motors
                //Turn on both base motors
                //Done when both touch sensors interrupt
                break;
            case Getting_Balls:
                //Flash the light
                for(int i=0;i<12;i++){
                    //flash
                    //delay
                }
                //Done after ________ flashes
                break;
            case Moving_Away_From_Dispencer:
                //Set direction of both base motors
                //Turn on both base motors
                //Done after motor step interrupt
                break;
            case Finding_Goal:
                //Look at which sensors are on
                break;
            case Shooting:
                //Turn on shooting motors
                //Turn on Geniva
                //Done after timer and no balls?
                break;
            default:
                break;
        }
    
    }
    
    return 0;
}

void initInputOutput(){
    TRISAbits.TRISA0 = 0;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 0;
}

void initDigitalPorts(){
    ANSAbits.ANSA0 = 0;
    ANSBbits.ANSB1 = 0;
    ANSBbits.ANSB2 = 0;
}

void initInterupts(){
    
    // Configure Timer1 using T1CON register
    _TON = 1;       // Turn Timer1 on
//    _TCKPS = 0b11;  // 1:256 prescaling
    _TCS = 0;       // Internal clock source (FOSC/2)
    TMR1 = 0;       // Reset Timer1
    // Configure Timer1 interrupt
    _T1IP = 4; // Select interrupt priority
    _T1IE = 1; // Enable interrupt
    _T1IF = 0; // Clear interrupt flag
    PR1 = 10000; // Set period for interrupt to occur
    _T1IF = 0; // Clear timer interrupt flag


    // Configure Change Notification interrupt
    _CN5IE = 1; // Enable CN on pin 5 (CNEN1 register)
    _CN5PUE = 0; // Disable pull-up resistor (CNPU1 register)
    _CNIP = 6; // Set CN interrupt priority (IPC4 register)
    _CNIF = 0; // Clear interrupt flag (IFS1 register)
    _CNIE = 1; // Enable CN interrupts (IEC1 register)
    _CNIF = 0; // Clear Change Notification interrupt flag (IFS1 register)
    
    
}

void initAnalogPorts(){
    
	/*** Configure Desired Port Pins as Analog Inputs ***/
	_TRISA0 = 1;		// TRISA/B, pg. 45 datasheet
	_ANSA0 = 1;			// ANSA/B, pg. 136-137


	/*** Select Voltage Reference Source ***/
	// use AVdd for positive reference
	_PVCFG = 00;		// AD1CON2<15:14>, pg. 212-213 datasheet
	// use AVss for negative reference
	_NVCFG = 0;			// AD1CON2<13>


	/*** Select Analog Conversion Clock Rate ***/
	// make sure Tad is at least 600ns, see Table 29-41 datasheet
	_ADCS = 0x02;	// AD1CON3<7:0>, pg. 213 datasheet


	/*** Select Sample/Conversion Sequence ***/
	// use auto-convert
	_SSRC = 0b0111;		// AD1CON1<7:4>, pg. 211 datasheet
	// use auto-sample
	_ASAM = 1;			// AD1CON1<2>
	// choose a sample time >= 1 Tad, see Table 29-41 datasheet
	_SAMC = 0b0001;		// AD1CON3<12:8>


	/*** Choose Analog Channels to be Used ***/
	// scan inputs
	_CSCNA = 1;			// AD1CON2<10>
	// choose which channels to scan, e.g. for ch AN12, set _CSS12 = 1;
	_CSS0 = 1;			// AD1CSSH/L, pg. 217


	/*** Select How Results are Presented in Buffer ***/
	// set 12-bit resolution
	_MODE12 = 1;		// AD1CON1<10>
	// use absolute decimal format
	_FORM = 00;			// AD1CON1<9:8>
	// load results into buffer determined by converted channel, e.g. ch AN12 
    // results appear in ADC1BUF12
	_BUFREGEN = 1;		// AD1CON2<11>


	/*** Select Interrupt Rate ***/
	// interrupt rate should reflect number of analog channels used, e.g. if 
    // 5 channels, interrupt every 5th sample
	_SMPI = 0;		// AD1CON2<6:2>


	/*** Turn on A/D Module ***/
	_ADON = 1;			// AD1CON1<15>
    
    
    //How to read the buffer
    //float in1 = ADC1BUF0;
    //OC2R = 2000;
}

void initPwmPorts(){
    
    //-----------------------------------------------------------
    // CONFIGURE PWM1 USING OC1 (on pin 14)
    
    // Clear control bits initially
    OC1CON1 = 0;
    OC1CON2 = 0;
   
    // Set period and duty cycle
    OC1R = 3000;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC1RS = 3999;               // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC1CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC1CON2bits.SYNCSEL = 0x1F; // Select OC1 as synchronization source
                                // (self synchronization) -- Although we
                                // selected the system clock to determine
                                // the rate at which the PWM timer increments,
                                // we could have selected a different source
                                // to determine when each PWM cycle initiates.
                                // From the FRM: When the SYNCSEL<4:0> bits
                                // (OCxCON2<4:0>) = 0b11111, they make the
                                // timer reset when it reaches the value of
                                // OCxRS, making the OCx module use its
                                // own Sync signal.
    OC1CON2bits.OCTRIG = 0;     // Synchronizes with OC1 source instead of
                                // triggering with the OC1 source
    OC1CON1bits.OCM = 0b110;    // Edge-aligned PWM mode    
    //-----------------------------------------------------------
}