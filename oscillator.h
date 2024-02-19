
#define _XTAL_FREQ 64000000

// CONFIG1H
//#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
//#pragma config PRICLKEN = OFF ////   // Primary clock enable bit (Primary clock can be disabled by software)
////#pragma config PLLCFG = OFF
//#pragma config FCMEN = OFF  ///// // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
//#pragma config IESO = ON      // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

void set_internal_oscillator_with_PLL() {
    // oscillator settings

    OSCCONbits.IRCF = 0b111; // setting internal oscillator to 16Mhz
    OSCCONbits.SCS = 0b00; // System Clock Select bit, Primary clock, because we want to go throught the PLL multiplexer
    
    PLLEN = 1; // PLL enabled for HFINTOSC (8 MHz and 16 MHz only)
    
//    // PRISD doesn't fucking matter
//    PRISD = 1; // Primary Oscillator Drive Circuit Shutdown bit. Oscillator drive circuit off (zero power)
//    
//    SOSCEN = 0;
//    SOSCGO = 0; // Secondary oscillator is shut off if no other sources are requesting it.
//    SOSCRUN = 0; // System clock comes from an oscillator, other than SOSC
}