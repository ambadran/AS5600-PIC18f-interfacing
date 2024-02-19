/*
 * File:   main.c
 * Author: ambadran717
 *
 * Created on March 20, 2022, 12:30 PM
 */


#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "config.h"
#include "oscillator.h"
#include "GPIO.h"
#include "UART.h"
#include "I2C.h"
#include "AS5600.h"
#include "MPU6050.h"

bool start = false;

void main(void) {
    
    // oscillator
    set_internal_oscillator_with_PLL();
    
    // GPIO
    setGPIO();
    
    // UART
    initUART1(115200);
    
    // I2C
    init_I2C_Master1(100000);
    
    //AS5600
//    initAS5600();
    
    //MPU6050
    initMPU6050(0, 0);
    
    // Main Routine
    while(1) {
        if(button) {
            start = ~start;
            __delay_ms(300);
        }
        
        if(start) {
            get_burn_times();
            get_CORDIC_MAGNITUDE();
            get_AGC();
            check_magnet();
            getRawReading();
            getReading();
            print("\n");
            sendGYROS();
            sendACCELS();
            print("\n");
            
            start = false;
        }
    }
    
    return;
}
