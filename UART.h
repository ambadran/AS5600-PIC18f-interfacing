#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

char buffer[50];

unsigned char uartdata;
bool received = false;

//void print(char* string, uint8_t num_arg, ...) { //TODO: add placeholder arguments feature to my print function
//    // getting arguments
//    va_list valist; 
//}

void print(char* value) {

    while(*value != '\0') {
        while(!TRMT1){
            
        }
        TXREG1 = *value;
        value++;
    }
    
}

void initUART1(long int baud_rate) {
    TXEN1 = 1;
    CREN1 = 1;
    SYNC1 = 0;
    SPEN1 = 1;
   
    TX91 = 0;
    SENDB1 = 1;
    
    TX1_TRIS = 1; // TX pin
    RX1_TRIS = 1; // RX pin
    TX1_ANS = 0;
    RX1_ANS = 0;
    
    BRGH1 = 1;
    BRG161 = 1;
    
    uint16_t num1 = (uint16_t)((_XTAL_FREQ/baud_rate)/4 - 1);
    SPBRGH1 = ((uint16_t)num1 >> 8) & 0xFF;
    SPBRG1 = ((uint16_t)num1 >> 0) & 0xFF;
    //TODO: fix the stupid 
    //warning: implicit conversion loses integer precision: 'long' to 'unsigned char'
    // I tried using (unsigned char) and (uint8_t) 
    //      it affects the output value to SPBRG the baud rate become wrong
    // uint16_t gives the same warning as it is essentially (unsigned int)
    //(unsigned char)(uint16_t) doesn't work too 
    
    //interrupts
    GIE = 1;
    PEIE = 1;
    RC1IE = 1; // UART receive interrupt enable
}

void UART_ISR1() {
    if(RC1IF) {
        uartdata = RCREG1;
        received = true;
        RC1IF = 0;
    }
}

///////////////////////////////// for second UART module ///////////////////////
unsigned char uartdata2;
bool received2 = false;

void sendUART(unsigned char value) {
    while(!TRMT2);
    TXREG2 = value;
//    __delay_ms(50);
}

void initUART2(long int baud_rate) {
    TXEN2 = 1;
    CREN2 = 1;
    SYNC2 = 0;
    SPEN2 = 1;
   
    TX92 = 0;
    SENDB2 = 1;
    
//TODO: set the right pins and make the #define in the GPIO header
//    TRISC6 = 1; // TX pin
//    TRISC7 = 1; // RX pin
    
    BRGH2 = 1;
    BRG162 = 1;
    
    uint16_t num2 = (uint16_t)((_XTAL_FREQ/baud_rate)/4 - 1);
    SPBRGH2 = ((uint16_t)num2 >> 8) & 0xFF;
    SPBRG2 = ((uint16_t)num2 >> 0) & 0xFF;
    //TODO: fix the stupid 
    //warning: implicit conversion loses integer precision: 'long' to 'unsigned char'
    // I tried using (unsigned char) and (uint8_t) 
    //      it affects the output value to SPBRG the baud rate become wrong
    // uint16_t gives the same warning as it is essentially (unsigned int)
    //(unsigned char)(uint16_t) doesn't work too 
    
    //interrupts
    GIE = 1;
    PEIE = 1;
    RC2IE = 1; // UART receive interrupt enable
}

void UART_ISR2() {
    if(RC2IF) {
        uartdata2 = RCREG2;
        received2 = true;
        RC2IF = 0;
    }
}
