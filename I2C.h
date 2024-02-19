#include "config.h"
//#include "GPIO.h"

void init_I2C_Master1(long int baudRate) {
    
    SCL1_TRIS = 1; // SDA pin input
    SDA1_TRIS = 1; // SCL pin input
    SCL1_ANS = 0;
    SDA1_ANS = 0;
    
    SSP1CON1bits.SSPM = 0b1000; // Master mode
    SSP1CON1bits.SSPEN = 1; // MSSP module ON
    SSP1CON2 = 0x00; // defaulting to 0
    SSP1STAT = 0x00; // defaulting to 0
    SSP1ADD = (unsigned char)((_XTAL_FREQ/4)/baudRate) - 1;
}

void wait1() {
  while ((SSP1STAT & 0x04) || (SSP1CON2 & 0x1F));
  //0x04 = 0b100, 0x1F = 0b11111
  
  // SSPSTATbits.BF == 0 // bit0: in transmit mode, SSPBUF is empty
  // SSPSTATbits.UA == 0 // bit1: Address does need to be updated
  // SSPSTATbits.R ==1 // bit2: in master mode, transmit in progress
  
  // SSPCON2bits.SEN == 1 // bit0: start condition
  // SSPCON2bits.RSEN == 1 // bit1: repeat start condition
  // SSPCON2bits.PEN == 1 // bit2: stop condition
  // SSPCON2bits.RCEN == 1 // bit3: receive enable bit
  // SSPCON2bits.ACKEN == 1 // bit4: Acknowledge sequence enable bit
  
  // once all these conditions are NOT met, wait is over and we can proceed
}

void start_condition1() {
  // Start Condition Sequence
  wait1();
  SEN1 = 1;
}

void stop_condition1() {
  // Stop Condition Sequence
  wait1();
  PEN1 = 1;
}

void restart_condition1() {
  // Restart Condition Sequence
  wait1();
  RSEN1 = 1;
}

void sendACK1(void) {
  // Send ACK - For Master Receiver Mode
  wait1();
  ACKDT1 = 0; // 0 -> ACK, 1 -> NACK
  ACKEN1 = 1; // Send ACK Signal!
}

void sendNACK1(void) {
  // Send NACK - For Master Receiver Mode
  wait1();
  ACKDT1 = 1; // 1 -> NACK, 0 -> ACK
  ACKEN1 = 1; // Send NACK Signal!
}



//////////////////////////////////// sending functions ////////////////////////
unsigned char rawSendI2C1(unsigned char data) {
  // Send Byte, Return The ACK/NACK
  wait1();
  SSP1BUF = data;
  wait1();
  return ACKSTAT1;
}

void basicSendI2C1(unsigned char address, unsigned char data) {
    start_condition1();
    rawSendI2C1(address); // address
    rawSendI2C1(data); // data
    stop_condition1();
}

void sendI2C1(unsigned char module_address, unsigned char address, unsigned char data) {
    
    //master writing one byte sequence
    start_condition1(); // ST
    rawSendI2C1(module_address); // SAD+W(0) Assuming SDO=1
    rawSendI2C1(address); // SUB, NOTE that it is missing a digit and that's okay
    rawSendI2C1(data); // DATA
    stop_condition1(); //SP
    
}
///////////////////////////////////////////////////////////////////////////////



////////////////////////////////// Reading functions //////////////////////////
uint8_t rawReadI2C1(void) {
    uint8_t Data;
    wait1();
    RCEN1 = 1;
    wait1();
    Data = SSP1BUF;
    sendNACK1(); // to tell the slave master don't want to receive any more
    return Data;
}

uint8_t basicReadI2C1(unsigned char address) {
    start_condition1();
    rawSendI2C1(address + 1);
    uint8_t result = rawReadI2C1();
    stop_condition1();
    return result;
}

uint8_t readI2C1(unsigned char module_address, unsigned char address) {
    
    uint8_t data;
    
    //master reading one byte sequence
    start_condition1(); // ST
    rawSendI2C1(module_address); // SAD+W(0) 
    rawSendI2C1(address); // SUB, NOTE that it is missing a digit and that's okay
    restart_condition1(); // SR
    rawSendI2C1(module_address+1); // SAD+R(1)
    data = rawReadI2C1(); // DATA read
    stop_condition1(); //SP
    
    return data; 
}
///////////////////////////////////////////////////////////////////////////////


void init_I2C_Slave1(unsigned char address) {
    
    SDA1_TRIS = 1; // SDA pin input
    SCL1_TRIS = 1; // SCL pin input
    SDA1_ANS = 0;
    SDA1_ANS = 0;
    
    SSP1ADD = address; // setting device address
    SSP1STATbits.SMP = 1; // disabling slew rate for standard speed mode (100kHz)
    SSP1CON1bits.SSPM = 0b0110; // setting to I2C slave mode, 7bit address
    SSP1CON1bits.CKP = 1; // in slave mode, disabling clock stretching, I2C line left high
    SSP1CON1bits.SSPEN = 1; // enabling MSSP module
    
    GIE = 1; // enabling interrupts
    PEIE = 1;
    SSP1IE = 1;
    SSP1IF = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// SECOND MODULE AS I2C /////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

void init_I2C_Master2(long int baudRate) {
    
    SCL2_TRIS = 1; // SDA pin input
    SDA2_TRIS = 1; // SCL pin input
    SCL2_ANS = 0;
    SDA2_ANS = 0;
    
    SSP2CON1bits.SSPM = 0b1000; // Master mode
    SSP2CON1bits.SSPEN = 1; // MSSP module ON
    SSP2CON2 = 0x00; // defaulting to 0
    SSP2STAT = 0x00; // defaulting to 0
    SSP2ADD = (unsigned char)((_XTAL_FREQ/4)/baudRate) - 1;
}

void wait2() {
  while ((SSP2STAT & 0x04) || (SSP2CON2 & 0x1F));
  //0x04 = 0b100, 0x1F = 0b11111
  
  // SSPSTATbits.BF == 0 // bit0: in transmit mode, SSPBUF is empty
  // SSPSTATbits.UA == 0 // bit1: Address does need to be updated
  // SSPSTATbits.R ==1 // bit2: in master mode, transmit in progress
  
  // SSPCON2bits.SEN == 1 // bit0: start condition
  // SSPCON2bits.RSEN == 1 // bit1: repeat start condition
  // SSPCON2bits.PEN == 1 // bit2: stop condition
  // SSPCON2bits.RCEN == 1 // bit3: receive enable bit
  // SSPCON2bits.ACKEN == 1 // bit4: Acknowledge sequence enable bit
  
  // once all these conditions are NOT met, wait is over and we can proceed
}

void start_condition2() {
  // Start Condition Sequence
  wait2();
  SEN2 = 1;
}

void stop_condition2() {
  // Stop Condition Sequence
  wait2();
  PEN2 = 1;
}

void restart_condition2() {
  // Restart Condition Sequence
  wait2();
  RSEN2 = 1;
}

void sendACK2(void) {
  // Send ACK - For Master Receiver Mode
  wait2();
  ACKDT2 = 0; // 0 -> ACK, 1 -> NACK
  ACKEN2 = 1; // Send ACK Signal!
}

void sendNACK2(void) {
  // Send NACK - For Master Receiver Mode
  wait2();
  ACKDT2 = 1; // 1 -> NACK, 0 -> ACK
  ACKEN2 = 1; // Send NACK Signal!
}

unsigned char rawSend2(unsigned char data) {
  // Send Byte, Return The ACK/NACK
  wait2();
  SSP2BUF = data;
  wait2();
  return ACKSTAT2;
}

void sendI2C2(unsigned char address, unsigned char data) {
    start_condition2();
    rawSend2(address); // address
    rawSend2(data); // data
    stop_condition2();
}

unsigned char rawRead2(void) {
    unsigned char Data;
    wait2();
    RCEN2 = 1;
    wait2();
    Data = SSP2BUF;
    sendNACK2(); // to tell the slave master don't want to receive any more
    return Data;
}

unsigned char readI2C2(unsigned char address) {
    start_condition2();
    rawSend2(address + 1);
    unsigned char result = rawRead2();
    stop_condition2();
    return result;
}

void init_I2C_Slave2(unsigned char address) {
    
    SDA2_TRIS = 1; // SDA pin input
    SCL2_TRIS = 1; // SCL pin input
    SDA2_ANS = 0;
    SDA2_ANS = 0;
    
    SSP2ADD = address; // setting device address
    SSP2STATbits.SMP = 1; // disabling slew rate for standard speed mode (100kHz)
    SSP2CON1bits.SSPM = 0b0110; // setting to I2C slave mode, 7bit address
    SSP2CON1bits.CKP = 1; // in slave mode, disabling clock stretching, I2C line left high
    SSP2CON1bits.SSPEN = 1; // enabling MSSP module
    
    GIE = 1; // enabling interrupts
    PEIE = 1;
    SSP2IE = 1;
    SSP2IF = 0;
}
