/*
 * File:   BJT_Levitation.c
 * Authors: Roberto Oliveira, uc2010144010@student.uc.pt  
 *          Miguel Diogo, uc2015240094@student.uc.pt
 *
 * Description: PI controller generates analogue signal that drives a BJT transistor that turns on and off an inductor 
 *              in order to levitate an object around a setpoint (TARGET).
 */


#include "xc.h"
#include <stdio.h>
#include "config.h"

#define TARGET  841 //843.5361
#define KP 80 //100.0  cem esta fraquinho tal como 160
#define KI 2//10.0
#define DAC_PWM_MAX 172 //255 mais oscilatorio
//---------------------------
unsigned int hall1 ;//up
unsigned int  hall2;//dowm


//--------Funções-------------
inline void ConfigCLK(void);
inline void ConfigTMR1(void);
void _ISRFAST _T1Interrupt(void);
inline void UART1Init(unsigned long int baud);
inline void ConfigIO(void);
inline void ConfigADC(void);
unsigned int readADC(unsigned int ch);
inline void ConfigOA(void);
//------------------------
//-----------------------
int main(void) {
     ConfigCLK();
     ConfigTMR1();
     UART1Init(19200);
     ConfigIO();
     ConfigADC();
     ConfigOA();
     
     printf("\r\nStart");
     while(1){
     
     }
    return 0;
}
//---------------------
//---------------------
inline void ConfigCLK(void)
{
    //configurar oscilador para 32MHz
    CLKDIVbits.DOZE = 0;    // 1:1
    CLKDIVbits.RCDIV = 0;   // 8 MHz
}
//----------------------
inline void ConfigTMR1(void)
{
    T1CON=0x8010; // Div 1:8
    PR1=200-1; // TTick = 0.00001s
    // T1CON=0x8020; // Div 64
    //PR1=250-1; // TTick = 0.01s
    _T1IF=0;
    _T1IE=1;
}
//------------------------------
void _ISRFAST _T1Interrupt(void)  // 0.00001s
{    static int16_t pwm = DAC_PWM_MAX; //(256 -1);
    static int erro,posicao_atual;
    static int integral = 0;
    
    hall1 = readADC(1);   //pino 3 AN1 RA1 (halll UP) TP2
    hall2 = readADC(2);  //pino 4 AN2 RB0 (hall down) TP3
    

    posicao_atual = hall2 -hall1;
    erro  = TARGET - posicao_atual;     
    integral += erro;
    
    pwm =(int) (KP*erro) + (KI * integral);
    
    if(pwm > DAC_PWM_MAX)
        pwm = DAC_PWM_MAX;
    else if (pwm < 0)
        pwm = 0;
    
    DAC2DAT  = pwm;    
    _T1IF = 0;              // clear the interrupt flag   
} //T1Interrupt

//-----------------------
inline void UART1Init(unsigned long int baud){
    U1BRG =  (FCY / (16 * baud)) -1; //BAUDRATEREG1;
    U1MODE = 0;
    U1MODEbits.BRGH = 0;
    U1STA = 0;
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;
    U1STAbits.URXISEL = 2;
    U1MODEbits.UEN = 3;
}
//-------------------------
inline void ConfigIO(void)
{
    TRISB= 0b1111111111111111;
    TRISBbits.TRISB0 = 1;   // entrada pino 4 AN2 RB0
    TRISBbits.TRISB2 = 1;   // entrada (U1RX)W
    TRISBbits.TRISB7 = 0;   // saida (U1TX)  
    TRISBbits.TRISB14= 0;   // saida (DAC2OUT/pin25)(OA1IND/OA2IND/DAC2OUT/pin25)
    TRISBbits.TRISB15 = 1;   // (entrada) nao saida (OA2OUT/pin26)
    TRISA= 0b1111111111111111;
    TRISAbits.TRISA1 = 1;   // entrada pino 3 AN1 RA1
    ANSB = 0; // U1RX e U1TX digitais
    ANSBbits.ANSB0 = 1;   //analogico pino 4 AN2 RB0
    ANSBbits.ANSB14= 1; // analogico Rb14 (OA1IND/OA2IND/DAC2OUT/pin25)
    ANSBbits.ANSB15 = 1;   // analogico Rb15 (OA2OUT)
    ANSA = 0; // Porto A digital
    ANSAbits.ANSA1 = 1;   //analogico pino 3 AN1 RA1
}
//------------------------
inline void ConfigADC(void)
{
    AD1CON1 = 0x0470;   // 12-bit A/D operation (MODE12=1) // SSRC<3:0> = 0111 Internal counter ends sampling // and starts conversion (auto-convert)
    AD1CON2 = 0x0000;   // Configure A/D voltage reference and buffer fill modes.
    AD1CON3 = 0x1003;   // Sample time = 16Tad, Tad = 4Tcy (= 250ns)
 
    AD1CSSL = 0;        // No inputs are scanned.
 
    _AD1IF = 0; // Clear A/D conversion interrupt.
    _AD1IE = 0; // Disable A/D conversion interrupt
    AD1CON1bits.ADON = 1; // Turn on A/D
}
//-------------------------
unsigned int readADC(unsigned int ch)
{
    AD1CHS = ch;            // Select analog input channel
    AD1CON1bits.SAMP = 1;   // start sampling, then go to conversion
    
    while (!AD1CON1bits.DONE); // conversion done?
    return(ADC1BUF0);       // yes then get ADC value
}
 //-----------------------------
inline void ConfigOA(void){
    //DAC1CON = 0;
    //DAC1CONbits.DACOE=1;        // DACx output pin is enabled //Activa o pino DACxOUT 
    //DAC1CONbits.DACREF=0b10; //DAC1  // DACx Reference Source AVDD
    //DAC1CONbits.DACEN=1;        // Enable DAC
    DAC2CON = 0;
    DAC2CONbits.DACOE=1;        // DACx output pin is enabled //Activa o pino DACxOUT 
    DAC2CONbits.DACREF=0b10;  // DACx Reference Source AVDD
    DAC2CONbits.DACEN=1;        // Enable DAC
    AMP2CON = 0;
    AMP2CONbits.PINSEL = 0b101; // Positive input connected to DAC2
    AMP2CONbits.NINSEL = 0b101; // Negative input connected to OA2OUT
    AMP2CONbits.AMPEN = 1;      // Enable Op Amp
}
