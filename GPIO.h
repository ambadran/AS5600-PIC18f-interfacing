#define test_pin PORTDbits.RD1
#define button PORTDbits.RD2

#define TX1_TRIS TRISC6
#define RX1_TRIS TRISC7
#define TX1_ANS ANSC6
#define RX1_ANS ANSC7

#define SDA1_TRIS TRISC4
#define SCL1_TRIS TRISC3
#define SDA1_ANS ANSC4
#define SCL1_ANS ANSC3

#define SDA2_TRIS TRISD1
#define SCL2_TRIS TRISD0
#define SDA2_ANS ANSD1
#define SCL2_ANS ANSD0

//    ~
void setGPIO() {
    
    //TRIS register
    TRISD1 = 0;
    TRISD2 = 1;
    
    // ANSEL register
    ANSD1 = 0;
    ANSD2 = 0;
    
    test_pin = 0;
    
    ANSELA = 0x00;
    
}