/*
 * File:   main.c
 * Author: mccda
 *
 * Created on October 10, 2017, 1:18 PM
 */

#pragma config ICS = PGx3 //this allows debugging.... use PGC/D pair 3 (pins 9,10) for debugging
#pragma config FNOSC = FRCDIV // select 8MHz oscillator with postscaler

#include "xc.h"
#define FCY 4000000
#include <libpic30.h>

//Configurations------------------------------------
float motorPwmPeriod = 3999;
float turretPwmPeriod = 3999;
float launcherPwmPeriod = 3999;
int numberOfBalls = 6;
int maxIrSensorAnalogInput = 2048;
int irSensorAnalogThreshold = 1500;
//--------------------------------------------------

float motorSpeedPercent = 0;
float turretSpeedPercent = 0;
float launcherSpeedPercent = 0;
float irInput1 = 0;
float irInput2 = 0;
float blackBallInput = 0;
int goal = 0;

// Create a set of possible states
enum {Looking_For_Dispencer, Moving_Tward_Dispencer, Getting_Balls, Moving_Away_From_Dispencer, Finding_Goal, Shooting, ERROR, DEBUG} state;
enum {Forward, Backward, RotateLeft, RotateRight} baseDirection;
enum {Left, Right} turretDirection;

void initInputOutput();
void initDigitalPorts();
void initAnalogPorts();
void initPwmPorts();
void initInterupts();
void moveBase();
void moveTurret();
void moveLauncher();
void setTimer();


// Change Notification Interrupt Service Routine (ISR)
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void)
{
    _CNIF = 0; // Clear interrupt flag (IFS1 register)
    
    //CURRENTLY ONLY HANDLING PIN 8 ---> toutch sensors
    switch (state){
        case Looking_For_Dispencer:
            break;
        case Moving_Tward_Dispencer:
            state = Getting_Balls;
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
            irInput1 = ADC1BUF10;
            irInput2 = ADC1BUF9;
            if(irInput1>irSensorAnalogThreshold){
                goal = 1;
            }
            else if(irInput2>irSensorAnalogThreshold){
                goal = 2;
            }
            else{
                goal = 3;
            }
            state = Finding_Goal;
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
    Pin 6/RB2: Analog input for ROS
    Pin 7/RA2: Sleep
    Pin 8/RA3: Touch sensor(s) input
    Pin 9/RB4: --
    Pin 10/RA4: --
    Pin 11/RB7: --
    Pin 12/RB8: --
    Pin 13/RB9: --
    Pin 14/RA6: PWM for H-Bridge
    Pin 15/RB12: Dir 1
    Pin 16/RB13: Dir 2
    Pin 17/RB14: Analog input for Light sensor 1
    Pin 18/RB15: Analog input for Light sensor 2
    Pin 19/VSS: Vss
    Pin 20/VDD: Vdd
     * */
    
    initInputOutput();
    initDigitalPorts();
    initInterupts();
    initPwmPorts();
    initAnalogPorts();

    state = Moving_Tward_Dispencer;
    
    __delay_ms(3000);
    
    int i = 0;
    
    while(1){
        
        switch (state){
            case Looking_For_Dispencer:
                //Rotate base to look for IR emitter
                motorSpeedPercent = .3;
                baseDirection = RotateLeft;
                moveBase();
                //Done when IR receiver emitter interrupts
                break;
            case Moving_Tward_Dispencer:
                //Set direction of both base motors
                motorSpeedPercent = .8;
                baseDirection = Forward;
                //Turn on both base motors
                moveBase();
                //Done when both touch sensors interrupt
                break;
            case Getting_Balls:
                //Flash the light
                for(i=0;i<numberOfBalls;i++){
                    //flash
                    _LATA0 = 1;
                    //delay
                    __delay_ms(800);
                    //flash
                    _LATA0 = 0;
                    //delay
                    __delay_ms(800);
                }
                //Done after ________ flashes
                state = Moving_Away_From_Dispencer;
                break;
            case Moving_Away_From_Dispencer:
                //Set direction of both base motors
                motorSpeedPercent = .8;
                baseDirection = Backward;
                //Turn on both base motors
                moveBase();
                //Done after motor step interrupt
                setTimer(20000);
                break;
            case Finding_Goal:
                //Look at which sensors are on
                turretSpeedPercent = .8;
                if(goal == 0){
                    turretDirection = Left;
                }else if(goal == 3){
                    turretDirection = Right;
                }
                //Turn on both base motors
                moveTurret();
                break;
            case Shooting:
                //Turn on shooting motors
                launcherSpeedPercent = .8;
                moveLauncher();
                //Turn on Geniva
                //Done after timer and no balls?
                break;
            case ERROR:
                //flash
                _LATA0 = 1;
                //delay
                __delay_ms(300);
                //flash
                _LATA0 = 0;
                //delay
                __delay_ms(300);
                break;
            case DEBUG:
                //flash
                _LATA0 = 1;
                //delay
                __delay_ms(500);
                //flash
                _LATA0 = 0;
                //delay
                __delay_ms(500);
                break;
            default:
                break;
        }
    
    }
    
    return 0;
}

void initInputOutput(){
    TRISAbits.TRISA0 = 0;//Pin 2/RA0: digital output for LED
    TRISAbits.TRISA1 = 0;//Pin 3/RA1: digital output for Launch feeder
    TRISBbits.TRISB0 = 0;//Pin 4/RB0: PWM (Turret motor)
    TRISBbits.TRISB1 = 0;//Pin 4/RB0: PWM (Turret motor)
    TRISBbits.TRISB2 = 1;//Pin 6/RB2: Analog input for ROS
    TRISAbits.TRISA2 = 0;//Pin 7/RA2: Sleep
    TRISAbits.TRISA3 = 1;//Pin 8/RA3: Touch sensor(s) input
    //TRISBbits.TRISB4 = 0;
    //TRISAbits.TRISA4 = 0;
    //TRISBbits.TRISB7 = 0;
    //TRISBbits.TRISB8 = 0;
    //TRISBbits.TRISB9 = 0;
    TRISAbits.TRISA6 = 0;//Pin 14/RA6: PWM for H-Bridge
    TRISBbits.TRISB12 = 0;//Pin 15/RB7: Dir 1
    TRISBbits.TRISB13 = 0;//Pin 16/RB13: Dir 2
    TRISBbits.TRISB14 = 1;//Pin 17/RB14: Analog input for Light sensor 1
    TRISBbits.TRISB15 = 1;//Pin 18/RB15: Analog input for Light sensor 2
}

void initDigitalPorts(){
    ANSAbits.ANSA0 = 0;//Pin 2/RA0: digital output for LED
    ANSAbits.ANSA1 = 0;//Pin 3/RA1: digital output for Launch feeder
    ANSBbits.ANSB0 = 0;//Pin 4/RB0: PWM (Turret motor)
    ANSBbits.ANSB1 = 0;//Pin 4/RB0: PWM (Turret motor)
    ANSBbits.ANSB2 = 1;//Pin 6/RB2: Analog input for ROS
    ANSAbits.ANSA2 = 0;//Pin 7/RA2: Sleep
    ANSAbits.ANSA3 = 0;//Pin 8/RA3: Touch sensor(s) input
    ANSBbits.ANSB4 = 0;
    //ANSAbits.ANSA4 = 0;
    //ANSBbits.ANSB7 = 0;
    //ANSBbits.ANSB8 = 0;
    //ANSBbits.ANSB9 = 0;
    //ANSAbits.ANSA6 = 0;//Pin 14/RA6: PWM for H-Bridge
    ANSBbits.ANSB12 = 0;//Pin 15/RB7: Dir 1
    ANSBbits.ANSB13 = 0;//Pin 16/RB13: Dir 2
    ANSBbits.ANSB14 = 1;//Pin 17/RB14: Analog input for Light sensor 1
    ANSBbits.ANSB15 = 1;//Pin 18/RB15: Analog input for Light sensor 2
}

void initInterupts(){
    
    // Configure Timer1 using T1CON register
    _TON = 1;       // Turn Timer1 on
    _TCKPS = 0b11;  // 1:256 prescaling
    _TCS = 0;       // Internal clock source (FOSC/2)
    TMR1 = 0;       // Reset Timer1
    // Configure Timer1 interrupt
    _T1IP = 4; // Select interrupt priority
    _T1IE = 1; // Enable interrupt
    _T1IF = 0; // Clear interrupt flag
    PR1 = 10000; // Set period for interrupt to occur
    _T1IF = 0; // Clear timer interrupt flag


    // Configure Change Notification interrupt
    _CN8IE = 1; // Enable CN on pin 8 (CNEN1 register)
    _CN8PUE = 0; // Disable pull-up resistor (CNPU1 register)
    _CNIP = 6; // Set CN interrupt priority (IPC4 register)
    _CNIF = 0; // Clear interrupt flag (IFS1 register)
    _CNIE = 1; // Enable CN interrupts (IEC1 register)
    _CNIF = 0; // Clear Change Notification interrupt flag (IFS1 register)
    
    
}

void initAnalogPorts(){

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
	_CSS4 = 1;			// AD1CSSH/L, pg. 217
	_CSS9 = 1;			// AD1CSSH/L, pg. 217
	_CSS10 = 1;			// AD1CSSH/L, pg. 217

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
	_SMPI = 2;		// AD1CON2<6:2>  ??mccann is this right??


	/*** Turn on A/D Module ***/
	_ADON = 1;			// AD1CON1<15>
    
    
    //*****************How to read the analog buffer******************
    //float blackBallInput = ADC1BUF4;  ------> pin6
    //float irInput1 = ADC1BUF10;  ------> pin18
    //float irInput2 = ADC1BUF9;  ------> pin17
}

void initPwmPorts(){
    
    //-----------------------------------------------------------
    // CONFIGURE PWM1 USING OC1 (on pin 14)
    
    // Clear control bits initially
    OC1CON1 = 0;
    OC1CON2 = 0;
   
    // Set period and duty cycle
    OC1R = motorPwmPeriod*motorSpeedPercent;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC1RS = motorPwmPeriod;               // Period of OC1 to achieve desired PWM 
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
    //-----------------------------------------------------------
    // CONFIGURE PWM2 USING OC2 (on pin 4)
    
    // Clear control bits initially
    OC2CON1 = 0;
    OC2CON2 = 0;
   
    // Set period and duty cycle
    OC2R = turretPwmPeriod*turretSpeedPercent;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC2RS = turretPwmPeriod;               // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC2CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC2CON2bits.SYNCSEL = 0x1F; // Select OC1 as synchronization source
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
    OC2CON2bits.OCTRIG = 0;     // Synchronizes with OC1 source instead of
                                // triggering with the OC1 source
    OC2CON1bits.OCM = 0b110;    // Edge-aligned PWM mode    
    //-----------------------------------------------------------
    //-----------------------------------------------------------
    // CONFIGURE PWM2 USING OC3 (on pin 5)
    
    // Clear control bits initially
    OC3CON1 = 0;
    OC3CON2 = 0;
   
    // Set period and duty cycle
    OC3R = launcherPwmPeriod*launcherSpeedPercent;                // Set Output Compare value to achieve
                                // desired duty cycle. This is the number
                                // of timer counts when the OC should send
                                // the PWM signal low. The duty cycle as a
                                // fraction is OC1R/OC1RS.
    OC3RS = launcherPwmPeriod;               // Period of OC1 to achieve desired PWM 
                                // frequency, FPWM. See Equation 15-1
                                // in the datasheet. For example, for
                                // FPWM = 1 kHz, OC1RS = 3999. The OC1RS 
                                // register contains the period when the
                                // SYNCSEL bits are set to 0x1F (see FRM)
    
    // Configure OC1
    OC3CON1bits.OCTSEL = 0b111; // System (peripheral) clock as timing source
    OC3CON2bits.SYNCSEL = 0x1F; // Select OC1 as synchronization source
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
    OC3CON2bits.OCTRIG = 0;     // Synchronizes with OC1 source instead of
                                // triggering with the OC1 source
    OC3CON1bits.OCM = 0b110;    // Edge-aligned PWM mode    
    //-----------------------------------------------------------
}

void moveBase(){
    OC1R = motorPwmPeriod*motorSpeedPercent;
    switch (baseDirection){
        case Forward:
            _LATB13 = 0;
            _LATB15 = 0;
            break;
        case Backward:
            _LATB13 = 1;
            _LATB15 = 1;
            break;
        case RotateLeft:
            _LATB13 = 1;
            _LATB15 = 0;
            break;
        case RotateRight:
            _LATB13 = 0;
            _LATB15 = 1;
            break;
        default:
            motorSpeedPercent = 0;
            break;
    }
}
void moveTurret(){
    OC2R = turretPwmPeriod*turretSpeedPercent;
    switch (turretDirection){
        case Left:
            break;
        case Right:
            break;
        default:
            turretSpeedPercent = 0;
            break;
    }
}
void moveLauncher(){
    OC3R = launcherPwmPeriod*launcherSpeedPercent;
}
void setTimer(int period){
    PR1 = period; // Timer period
    TMR1 = 0;       // Reset Timer1
}