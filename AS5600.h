
#define AS5600 0x36

#define ZMCO 0x00
#define ZPOS0 0x01
#define ZPOS1 0x02
#define MPOS0 0x03
#define MPOS1 0x04
#define MANG0 0x05
#define MANG1 0x06
#define CONF0 0x07
#define CONF1 0x08

#define RAW_ANGLE0 0x0C
#define RAW_ANGLE1 0x0D
#define ANGLE0 0x0E
#define ANGLE1 0x0F

#define AS5600_STATUS 0x0B
#define AGC 0x1A
#define MAGNITUDE0 0x1B
#define MAGNITUDE1 0x1C

#define BURN 0xFF

void initAS5600() {
    
    uint8_t tmp;
    // to set the bit (tmp |= 1 << n)  
    // to clear the bit (tmp &= ~(1 << n))
    // the n is from the right to left and counting from 0
    
    tmp = readI2C1(AS5600, CONF0);
    
    tmp |= 1 << 0; // bit0 set - SF0 (SLOW SPEED filter ms response vs noise)
    tmp &= ~(1 << 1); // bit1 cleared - SF1
    
    tmp |= 1 << 2; // bit2 set  - FTH0 (HIGH SPEED filter LSB)
    tmp |= 1 << 3; // bit3 set - FTH1
    tmp |= 1 << 4; // bit4 set - FTH2
    
    tmp &= ~(1 << 5); // bit5 cleared - WD(watch dog)

    sendI2C1(AS5600, CONF0, tmp);
    
    // PWMF - PWM freq 920Hz, (920^-1)/((20*10^6/4)^-1) = 5434 > (2^12=4096), (this assumes we run at 20Mhz in the uC)
    // OUTS - output state, 00-analog(GND-Vdd), 01-analog(like 00 but max and min decrease by 10% each towards each other), 11-PWM output
    // HYST - hysteresis, 00-OFF, 01-1LSBs, 10-2LSBs, 11-3LSBs
    // PM - Power Mode, NOM-00-fully on, LPM1-01-polling period-5ms, LPM2-10-polling period-20ms, LPM3-11-polling period-100ms, 
    tmp = 0b11000100;
    sendI2C1(AS5600, CONF1, tmp);
    
}

uint8_t get_burn_times() {
    /*
     * reads ZMCO which saves how much times burn command was executed
     * which is a maximum of 3 for the device but the value is 3-bit value
     */
    uint8_t zmco = readI2C1(AS5600, ZMCO);
    sprintf(buffer, "Number of BURN commands done: %d\n", zmco);
    print(buffer);
    return zmco;
}


bool check_magnet() {
    // STATUS register
    
    uint8_t state = readI2C1(AS5600, AS5600_STATUS);
    if (state == 0b00100000) {
        print("Magnet is detected Successfully\n");
        return true;
    } else if (state == 0b00010000) {
        print("Magnet is too weak!!\n");
        return false;
    } else if (state == 0b00001000) {
        print("Magnet is too strong!!\n");
        return false;
    } else {
        sprintf(buffer, "No idea how the value is: %d\n", state);
        print(buffer);
        return false;
    }
}

void get_AGC() {
    //The AS5600 uses Automatic Gain Control in a closed loop to compensate for 
    // variations of the magnetic field strength due to changes of temperature,
    // airgap between IC and magnet, and magnet degradation. The AGC register 
    // indicates the gain. For the most robust performance, the gain value 
    // should be in the center of its range. The airgap of the physical system 
    // can be adjusted to achieve this value.
    //In 5V operation, the AGC range is 0-255 counts. The AGC range is reduced to 0-128 counts in 3.3V mode.
    
    sprintf(buffer, "AGC: %d\n", readI2C1(AS5600, AGC)); 
    print(buffer);
   
}

void get_CORDIC_MAGNITUDE() {
    // The output of the ADC is processed by the hardwired CORDIC block 
    // (Coordinate Rotation Digital Computer) to compute the angle and magnitude 
    // of the magnetic field vector.
    
    int cordic_magnitude = ((int)readI2C1(AS5600, MAGNITUDE0) << 8) | (int)readI2C1(AS5600, MAGNITUDE1);
    sprintf(buffer, "Cordic magnitude value: %d\n", cordic_magnitude);
    print(buffer);
}

void getRawReading() {
    int output = ((int)readI2C1(AS5600, RAW_ANGLE0) << 8) | (int)readI2C1(AS5600, RAW_ANGLE1);
    sprintf(buffer, "Raw Reading: %d\n", output);
    print(buffer);
}

void getReading() {
    int output = ((int)readI2C1(AS5600, ANGLE0) << 8) | (int)readI2C1(AS5600, ANGLE1);
    sprintf(buffer, "Reading: %d\n", output);
    print(buffer);
}

bool DO_BURN(bool mode) {
    /*
     * mode true: The host microcontroller can perform a permanent programming of ZPOS and MPOS with a BURN_ANGLE command.
     * NOTE: The BURN_ANGLE command can be executed up to 3 times. 
     * NOTE: This command may only be executed if the presence of the magnet is detected (MD = 1).
     * 
     * mode false: The host microcontroller can perform a permanent writing of MANG and CONFIG with a BURN_SETTING command.
     * NOTE: The BURN_ SETTING command can be performed only one time.
     * NOTE: MANG can be written only if ZPOS and MPOS have never been permanently written (ZMCO = 00). 
     */
    

    if(mode) {
        if (check_magnet()) {
            
            // BURN_ANGLE command
            sendI2C1(AS5600, BURN, 0x80);
            print("BURN_ANGLE command Successful\n");
            return true;
            
        } else {
            
            print("BURN_ANGLE command Failed\n");
            return false;
            
        }
    } else {
        
        if(get_burn_times() == 0) {
            // BURN_SETTING command
            sendI2C1(AS5600, BURN, 0x40);
            print("BURN_SETTING command Successful\n");
            return true;
            
        } else {
            
            print("BURN_SETTING command Failed\n");
            return false;
            
        }
    }
}
