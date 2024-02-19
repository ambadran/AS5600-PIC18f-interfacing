
// Define registers per MPU6050, Register Map and Descriptions, Rev 4.2, 08/19/2013 6 DOF Motion sensor fusion device
// Invensense Inc., www.invensense.com
// See also MPU-6050 Register Map and Descriptions, Revision 4.0, RM-MPU-6050A-00, 9/12/2012 for registers not listed in 
// above document; the MPU6050 and MPU 9150 are virtually identical but the latter has an on-board magnetic sensor
//
#include <stdio.h>

#define XGOFFS_TC        0x00 // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD                 
#define YGOFFS_TC        0x01                                                                          
#define ZGOFFS_TC        0x02
#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E    
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope; supported in MPU-6050?
#define XG_OFFS_USRL     0x14
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FF_THR           0x1D  // Free-fall
#define FF_DUR           0x1E  // Free-fall
#define MOT_THR          0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24   
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68

#define MPU6050_ADDRESS 0b11010000

uint8_t accel_range = 0; // 0 or 1 or 2 or 3
uint8_t gyro_range = 0; // 0 or 1 or 2 or 3
float gyro_sensitivity[] = {131.0, 65.5, 32.8, 16.4};
float accel_sensitivity[] = {16384.0, 8192.0, 4096.0, 2048.0};

void initMPU6050(uint8_t accel_range_input, uint8_t gyro_range_input) {
    
    // accel_range parameter
    //    0 : ±2g  : 16384 LSB/g
    //    1 : ±4g  : 8192 LSB/g
    //    2 : ±8g  : 4096 LSB/g
    //    3 : ±16g : 2048 LSB/g
   
    // gyro_range parameter
    //    0 : ± 250 ?/s  : 131 LSB/?/s 
    //    1 : ± 500 ?/s  : 65.5 LSB/?/s 
    //    2 : ± 1000 ?/s : 32.8 LSB/?/s 
    //    3 : ± 2000 ?/s : 16.4 LSB/?/s
    
    accel_range = accel_range_input;
    gyro_range = gyro_range_input;
    
    // wake up the device
    sendI2C1(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);
    __delay_ms(200);
    
    
    /***************************** PWR_MGMT_1 *********************************/
    // bit7 - When set to 1, this bit resets all internal registers to their default values. The bit automatically clears to 0 once the reset is done.
    //       The default values for each register can be found in Section 3.
    // bit6 - When set to 1, this bit puts the MPU-60X0 into sleep mode.
    // bit5 - When this bit is set to 1 and SLEEP is disabled, the MPU-60X0 will cycle between sleep mode and waking up to take a single sample of data from active sensors at a rate determined by LP_WAKE_CTRL (register 108).
    // bit4 - reserved
    // bit3 - When set to 1, this bit disables the temperature sensor.
    // bit2&1&0 - 3-bit unsigned value. Specifies the clock source of the device.
    //
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    sendI2C1(MPU6050_ADDRESS, PWR_MGMT_1, 0b00000001);
    /**************************************************************************/
    
    
    /****************************** CONFIG ************************************/
    // bit7 & bit6 - reserved
    // bit5 & 4 & 3 - EXT_SYNC_SET: Configures the external Frame Synchronization output (FSYNC) pin sampling. 
    // bit2 & 1 & 0 - DLPF_CFG: set the Digital Low Pass Filter value
    //
    // setting DLPF - 0b011
    sendI2C1(MPU6050_ADDRESS, CONFIG, 0b00000011);
    /**************************************************************************/
    
        
    /**************************** SMPLRT_DIV **********************************/
    //8-bit unsigned value. The Sample Rate is determined by dividing the gyroscope output rate by this value.
    // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    sendI2C1(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);
    /**************************************************************************/
    
    
    /**************************** GYRO_CONFIG *********************************/
    // bit7 - Setting this bit causes the X axis gyroscope to perform self test.
    // bit6 - Setting this bit causes the Y axis gyroscope to perform self test.
    // bit5 - Setting this bit causes the Z axis gyroscope to perform self test.
    // bit4 & bit3 - 2-bit unsigned value. Selects the full scale range of gyroscopes.
    // bit2 & 1 & 0 - reserved
    //
    uint8_t value = (uint8_t)(0b00000000 | gyro_range<<3);
    sendI2C1(MPU6050_ADDRESS, GYRO_CONFIG, value);
    /**************************************************************************/

    
    /**************************** ACCEL_CONFIG ********************************/
    // bit7 - When set to 1, the X- Axis accelerometer performs self test.
    // bit6 - When set to 1, the Y- Axis accelerometer performs self test.
    // bit5 - When set to 1, the Z- Axis accelerometer performs self test.
    // bit4 & 3 - 2bit unsigned value. Selects the full scale range of accelerometers.
    // bit2 & 1 & 0 - reserved
    //
    uint8_t value2 = (uint8_t)(0b00000000 | accel_range<<3);
    sendI2C1(MPU6050_ADDRESS, ACCEL_CONFIG, value2);
    /**************************************************************************/

    
    /****************************** FIFO_EN ***********************************/
    // bit7 - When set to 1, this bit enables TEMP_OUT_H and TEMP_OUT_L (Registers 65 and 66) to be written into the FIFO buffer.
    // bit6 - When set to 1, this bit enables GYRO_XOUT_H and GYRO_XOUT_L (Registers 67 and 68) to be written into the FIFO buffer.
    // bit5 - When set to 1, this bit enables GYRO_YOUT_H and GYRO_YOUT_L (Registers 69 and 70) to be written into the FIFO buffer.
    // bit4 - When set to 1, this bit enables GYRO_ZOUT_H and GYRO_ZOUT_L (Registers 71 and 72) to be written into the FIFO buffer.
    // bit3 - When set to 1, this bit enables ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L, ACCEL_ZOUT_H, and ACCEL_ZOUT_L (Registers 59 to 64) to be written into the FIFO buffer.
    // bit2 - When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96) associated with Slave 2 to be written into the FIFO buffer.
    // bit1 - When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96) associated with Slave 1 to be written into the FIFO buffer.
    // bit0 - When set to 1, this bit enables EXT_SENS_DATA registers (Registers 73 to 96) associated with Slave 0 to be written into the FIFO buffer.
    
    // shutting down all FIFO stuff
    sendI2C1(MPU6050_ADDRESS, FIFO_EN, 0b00000000);
    /**************************************************************************/

    
    /**************************** I2C_MST_CTRL ********************************/
    // bit7 - When set to 1, this bit enables multi-master capability.
    // bit6 - When set to 1, this bit delays the Data Ready interrupt until External Sensor data from the Slave devices have been loaded into the EXT_SENS_DATA registers.
    // bit5 - When set to 1, this bit enables EXT_SENS_DATA registers associated with Slave 3 to be written into the FIFO. The corresponding bits for Slaves 0-2 can be found in Register 35.
    // bit4 - Controls the I2C Master?s transition from one slave read to the next slave read. When this bit equals 0, there is a restart between reads. When this bit equals 1, there is a stop and start marking the beginning of the next read.
    // bit3 & 2 & 1 & 0 - 4 bit unsigned value. Configures the I2C master clock speed divider.
    //
    // closing multi-master capability, not wait for external sensor, no FIFO stuff,
    // a restart is between reads, and i don't care about master clock divider
    sendI2C1(MPU6050_ADDRESS, I2C_MST_CTRL, 0b00000000);
    /**************************************************************************/

    
    // not setting all the I2C_SLVx_ADDR/REG/DO/CTRL/DI registers because i am operating in single master mode
    
    
    /***************************** INT_PIN_CFG ********************************/
    // bit7 - When this bit is equal to 0, the logic level for the INT pin is active high. 
    //        When this bit is equal to 1, the logic level for the INT pin is active low.
    // bit6 - When this bit is equal to 0, the INT pin is configured as push-pull. 
    //        When this bit is equal to 1, the INT pin is configured as open drain.
    // bit5 - When this bit is equal to 0, the INT pin emits a 50us long pulse.
    //        When this bit is equal to 1, the INT pin is held high until the interrupt is cleared.
    // bit4 - When this bit is equal to 0, interrupt status bits are cleared only by reading INT_STATUS (Register 58)
    //        When this bit is equal to 1, interrupt status bits are cleared on any read operation.
    // bit3 - When this bit is equal to 0, the logic level for the FSYNC pin (when used as an interrupt to the host processor) is active high.
    //        When this bit is equal to 1, the logic level for the FSYNC pin (when used as an interrupt to the host processor) is active low.
    // bit2 - When equal to 0, this bit disables the FSYNC pin from causing an interrupt to the host processor.
    //        When equal to 1, this bit enables the FSYNC pin to be used as an interrupt to the host processor.
    // bit1 - When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to 0, the host application processor will be able to directly access the auxiliary I2C bus of the MPU-60X0.
    //        When this bit is equal to 0, the host application processor will not be able to directly access the auxiliary I2C bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106 bit[5]).
    // bit0 - reserved
    // 
    // i don't care about these options as i am not using interrupts
    /**************************************************************************/

    
    /****************************** INT_ENABLE ********************************/
    // bit7 - reserved
    // bit6 - reserved
    // bit5 - reserved
    // bit4 - When set to 1, this bit enables a FIFO buffer overflow to generate an interrupt.
    // bit3 - When set to 1, this bit enables any of the I2C Master interrupt sources to generate an interrupt.
    // bit2 - reserved
    // bit1 - reserved
    // bit0 - When set to 1, this bit enables the Data Ready interrupt, which occurs each time a write operation to all of the sensor registers has been completed.
    //
    // closing all interrupt options
    sendI2C1(MPU6050_ADDRESS, INT_ENABLE, 0b00000000);
    /**************************************************************************/

    
    /************************** I2C_MST_DELAY_CTRL ****************************/
    // bit7 - When set, delays shadowing of external sensor data until all data has been received.
    // bit6 - reserved
    // bit5 - reserved
    // bit4 - When enabled, slave 4 will only be accessed at a decreased rate.
    // bit3 - When enabled, slave 3 will only be accessed at a decreased rate.
    // bit2 - When enabled, slave 2 will only be accessed at a decreased rate.
    // bit1 - When enabled, slave 1 will only be accessed at a decreased rate.
    // bit0 - When enabled, slave 0 will only be accessed at a decreased rate.
    //
    // I am not using these functionalities
    /**************************************************************************/
    
    
    /************************** I2C_MST_DELAY_CTRL ****************************/
    // bit7 - reserved
    // bit6 - When set to 1, this bit enables FIFO operations.
    //        When this bit is cleared to 0, the FIFO buffer is disabled. The FIFO buffer cannot be written to or read from while disabled.
    //        The FIFO buffer?s state does not change unless the MPU-60X0 is power cycled.
    // bit5 - When set to 1, this bit enables I2C Master Mode.
    //        When this bit is cleared to 0, the auxiliary I2C bus lines (AUX_DA and AUX_CL) are logically driven by the primary I2C bus (SDA and SCL).
    // bit4 - MPU-6000: When set to 1, this bit disables the primary I2C interface and enables the SPI interface instead.
    //        MPU-6050: Always write this bit as zero.
    // bit3 - reserved
    // bit2 - This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This bit automatically clears to 0 after the reset has been triggered.
    // bit1 - This bit resets the I2C Master when set to 1 while I2C_MST_EN equals 0. This bit automatically clears to 0 after the reset has been triggered.
    // bit0 - When set to 1, this bit resets the signal paths for all sensors (gyroscopes, accelerometers, and temperature sensor). This operation will also clear the sensor registers. This bit automatically clears to 0 after the reset has been triggered.
    //        When resetting only the signal path (and not the sensor registers), please use Register 104, SIGNAL_PATH_RESET.
    //
    // closing everything and not triggering any resets
    sendI2C1(MPU6050_ADDRESS, I2C_MST_DELAY_CTRL, 0b00000000);
    /**************************************************************************/
    
    
    /***************************** PWR_MGMT_2 *********************************/
    // bit7 & 6 - 2-bit unsigned value. Specifies the frequency of wake-ups during Accelerometer Only Low Power Mode.
    // bit5 - When set to 1, this bit puts the X axis accelerometer into standby mode.
    // bit4 - When set to 1, this bit puts the Y axis accelerometer into standby mode.
    // bit3 - When set to 1, this bit puts the Z axis accelerometer into standby mode.
    // bit2 - When set to 1, this bit puts the X axis gyroscope into standby mode.
    // bit1 - When set to 1, this bit puts the Y axis gyroscope into standby mode.
    // bit0 - When set to 1, this bit puts the Z axis gyroscope into standby mode.
    //
    sendI2C1(MPU6050_ADDRESS, PWR_MGMT_2, 0b00000000);
    /**************************************************************************/
}

uint8_t bin8u_from_twosComp(uint8_t value) {
  return ~(value-1);
}

uint16_t bin16u_from_twosComp(uint16_t value) {
  return ~(value-1);
}

/******************************************************************************/
/************************** READING GYROSCOPE OUTPUT **************************/
/******************************************************************************/
int raw_read_GYRO(uint8_t axis) {
    uint8_t output_H = 0;
    uint8_t output_L = 0;
    int output = 0;
    
    if(axis == 0) { // X-AXIS
        output_H = readI2C1(MPU6050_ADDRESS, GYRO_XOUT_H);
        output_L = readI2C1(MPU6050_ADDRESS, GYRO_XOUT_L);
        output = ((int)output_H << 8) | (int)output_L;
        
    } else if(axis == 1) { // Y-AXIS
        output_H = readI2C1(MPU6050_ADDRESS, GYRO_YOUT_H);
        output_L = readI2C1(MPU6050_ADDRESS, GYRO_YOUT_L);
        output = ((int)output_H << 8) | (int)output_L;
        
    } else if(axis == 2) { // Z - AXIS
        output_H = readI2C1(MPU6050_ADDRESS, GYRO_ZOUT_H);
        output_L = readI2C1(MPU6050_ADDRESS, GYRO_ZOUT_L);
        output = ((int)output_H << 8) | (int)output_L;
        
    }
    
    return output;
}

float read_GYRO(uint8_t axis) {
    uint8_t output_H = 0;
    uint8_t output_L = 0;
    int output = 0;
    
    if(axis == 0) { // X-AXIS
        output_H = readI2C1(MPU6050_ADDRESS, GYRO_XOUT_H);
        output_L = readI2C1(MPU6050_ADDRESS, GYRO_XOUT_L);
        output = ((int)output_H << 8) | (int)output_L;
        
    } else if(axis == 1) { // Y-AXIS
        output_H = readI2C1(MPU6050_ADDRESS, GYRO_YOUT_H);
        output_L = readI2C1(MPU6050_ADDRESS, GYRO_YOUT_L);
        output = ((int)output_H << 8) | (int)output_L;
        
    } else if(axis == 2) { // Z - AXIS
        output_H = readI2C1(MPU6050_ADDRESS, GYRO_ZOUT_H);
        output_L = readI2C1(MPU6050_ADDRESS, GYRO_ZOUT_L);
        output = ((int)output_H << 8) | (int)output_L;
        
    }
    
    return (float)output/gyro_sensitivity[gyro_range];
}
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/



/******************************************************************************/
/********************** READING ACCELEROMETER OUTPUT **************************/
/******************************************************************************/
int raw_read_ACCEL(uint8_t axis) {
    uint8_t output_H = 0;
    uint8_t output_L = 0;
    int output = 0;
    
    if(axis == 0) { // X-AXIS
        output_H = readI2C1(MPU6050_ADDRESS, ACCEL_XOUT_H);
        output_L = readI2C1(MPU6050_ADDRESS, ACCEL_XOUT_L);
        output = ((int)output_H << 8) | (int)output_L;
        
    } else if(axis == 1) { // Y-AXIS
        output_H = readI2C1(MPU6050_ADDRESS, ACCEL_YOUT_H);
        output_L = readI2C1(MPU6050_ADDRESS, ACCEL_YOUT_L);
        output = ((int)output_H << 8) | (int)output_L;
        
    } else if(axis == 2) { // Z - AXIS
        output_H = readI2C1(MPU6050_ADDRESS, ACCEL_ZOUT_H);
        output_L = readI2C1(MPU6050_ADDRESS, ACCEL_ZOUT_L);
        output = ((int)output_H << 8) | (int)output_L;
        
    }
    
    return output;
}

float read_ACCEL(uint8_t axis) {
    uint8_t output_H = 0;
    uint8_t output_L = 0;
    int output = 0;
    
    if(axis == 0) { // X-AXIS
        output_H = readI2C1(MPU6050_ADDRESS, ACCEL_XOUT_H);
        output_L = readI2C1(MPU6050_ADDRESS, ACCEL_XOUT_L);
        output = ((int)output_H << 8) | (int)output_L;
        
    } else if(axis == 1) { // Y-AXIS
        output_H = readI2C1(MPU6050_ADDRESS, ACCEL_YOUT_H);
        output_L = readI2C1(MPU6050_ADDRESS, ACCEL_YOUT_L);
        output = ((int)output_H << 8) | (int)output_L;
        
    } else if(axis == 2) { // Z - AXIS
        output_H = readI2C1(MPU6050_ADDRESS, ACCEL_ZOUT_H);
        output_L = readI2C1(MPU6050_ADDRESS, ACCEL_ZOUT_L);
        output = ((int)output_H << 8) | (int)output_L;
        
    }
    
    return (float)output/accel_sensitivity[accel_range];
}
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/




/******************************************************************************/
/************************ READING TEMPERATURE OUTPUT **************************/
/******************************************************************************/
double read_TEMPERATURE() {
    uint8_t output_H = 0;
    uint8_t output_L = 0;
    int output = 0;
  
    output_H = readI2C1(MPU6050_ADDRESS, TEMP_OUT_H);
    output_L = readI2C1(MPU6050_ADDRESS, TEMP_OUT_L);
    output = ((int)output_H << 8) | (int)output_L;
    
    return ((double)output/340.0) + 36.53;
}
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/



/******************************************************************************/
/************************* Sending Data OUTPUTs *******************************/
/******************************************************************************/
void sendGYRO(uint8_t axe) {
    char message[9];
    char axes[3] = {'X', 'Y', 'Z'};
 
    sprintf(message, "G_%c: %.2f\n", axes[axe], read_GYRO(axe));
    print(message);
}

void sendGYROS() {
    sendGYRO(0);
    sendGYRO(1);
    sendGYRO(2);
}

void sendACCEL(uint8_t axe) {
    char message[9];
    char axes[3] = {'X', 'Y', 'Z'};
 
    sprintf(message, "A_%c: %.2f\n", axes[axe], read_ACCEL(axe));
    print(message);
}

void sendACCELS() {
    sendACCEL(0);
    sendACCEL(1);
    sendACCEL(2);

}

void sendTEMP() {
    char message[9];
    sprintf(message, "TEMP: %.2lf\n", read_TEMPERATURE());
}
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/



void read_FIFO() {
    //TODO: use FIFO_R_W to read FIFO
}
void write_FIFO() {
    //TODO: use FIFO_R_W to write to FIFO
}

void read_FIFO_COUNT() {
    //TODO: read FIFO_COUNT_H and FIFO_COUNT_L registers
}

void POWER_MANAGEMENT() {
    //TODO: use PWR_MGMT_x register as i want
}

void disable_TEMPERATUER_SENSOR() {
    //TODO: use TEMP_DIS(bit3) in PWR_MGMT_1 register to do that
}

void read_EXT_SENS_DATA(int packet_num) {
    //TODO: finish this function
}

void read_INT_STATUS() {
    //TODO: finish this function
}

void read_I2C_MST_STATUS() {
    //TODO: finish this function
}

void reset_ALL_SIGNAL_PATH_AND_DATA() {
    //using SIG_COND_RESET(bit0) in USER_CTRL register to do that
    sendI2C1(MPU6050_ADDRESS, I2C_MST_DELAY_CTRL, 0b00000001);

}

void reset_EVERYTHING() {
    sendI2C1(MPU6050_ADDRESS, PWR_MGMT_1, 0b10000000);
}

void reset_signal_path(bool gyroscope_reset, bool accelerometer_reset, bool temperature_reset) {
  
    /****************************** XXXXXXXXXX ********************************/
    // bit7 & 6 & 5 & 4 & 3 - reserved
    // bit2 - When set to 1, this bit resets the gyroscope analog and digital signal paths.
    // bit1 - When set to 1, this bit resets the accelerometer analog and digital signal paths.
    // bit0 - When set to 1, this bit resets the temperature sensor analog and digital signal paths.
    //
    if(gyroscope_reset) {
        sendI2C1(MPU6050_ADDRESS, SIGNAL_PATH_RESET, 0b00000100);

    }
    if(accelerometer_reset) {
        sendI2C1(MPU6050_ADDRESS, SIGNAL_PATH_RESET, 0b00000010);
    
    }
    if(temperature_reset) {
        sendI2C1(MPU6050_ADDRESS, SIGNAL_PATH_RESET, 0b00000001);

    }
    /**************************************************************************/
    
}

