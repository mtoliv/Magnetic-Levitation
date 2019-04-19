/*
 * File:   MOSFET_Levitation.c
 * Authors: Roberto Oliveira, uc2010144010@student.uc.pt  
 *          Miguel Diogo, uc2015240094@student.uc.pt
 *
 * Description: PID controller generates PWM signal that drives a MOSFET transistor that turns on and off an inductor 
 *              in order to levitate an object around a setpoint (TARGET).
 * Created on April 13, 2019, 3:48 PM
 */


#include "xc.h"
#include <stdio.h>
#include "config.h"
#include <math.h>
#define TARGET  940
#define KP 250
#define KI 0.01
#define KD 50
#define MAXPWM 4000


//---------Variaveis globais------------------
unsigned int hall1 ;//up
unsigned int  hall2;//dowm
unsigned int erro_anterior = 0;

//--------Funções-------------
inline void ConfigCLK(void);
inline void ConfigTMR1(void);
void _ISRFAST _T1Interrupt(void);
inline void UART1Init(unsigned long int baud);
inline void ConfigIO(void);
inline void ConfigADC(void);
unsigned int readADC(unsigned int ch);
inline void CCP1Config(void);
//------------------------
//-----------------------
int main(void) {
     ConfigCLK();
     ConfigTMR1();
     UART1Init(19200);
     ConfigIO();
     ConfigADC();
     CCP1Config();
     
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
    T1CON=0x8000; // Div 1:1
    PR1=1600-1; // TTick = 0.0001s
    // T1CON=0x8020; // Div 64
    //PR1=250-1; // TTick = 0.01s
    _T1IF=0;
    _T1IE=1;
}
//------------------------------
void _ISRFAST _T1Interrupt(void)  // 0.00001s
{   static int erro,posicao_atual;
    static int integral=0;
    static int duty=0;
    hall1 = readADC(1);   //pino 3 AN1 RA1 (halll UP) TP2
    hall2 = readADC(2);  //pino 4 AN2 RB0 (hall down) TP3
    

    posicao_atual = hall2 -hall1;
    //printf("\n\r pos at %d",posicao_atual);
    
    erro  = TARGET - posicao_atual;
    integral += erro;
    duty=(int)ceil((KP*erro) + (KI * integral)+(KD*(erro-erro_anterior)));
    
    if(duty > MAXPWM)
       duty = MAXPWM;
    else if (duty < 0)
        duty = 0;

     CCP1RB=duty; // 
     erro_anterior=erro;
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
    TRISBbits.TRISB8 = 0;  
    TRISA= 0b1111111111111111;
    TRISAbits.TRISA1 = 1;   // entrada pino 3 AN1 RA1
    ANSB = 0; // U1RX , U1TX  digitais
    ANSBbits.ANSB0 = 1;   //analogico pino 4 AN2 RB0
    ANSBbits.ANSB8 = 0; //digital pino 17 OC1B RB8 para pwm
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
inline void CCP1Config(void){

 
    // Set MCCP operating mode
    CCP1CON1L = 0;
    // CCP1CON1Lbits.CCSEL = 0; // Set MCCP operating mode (OC mode)
    CCP1CON1Lbits.MOD = 0b0101; // Set mode (Buffered Dual-Compare/PWM mode)
    CCP1CON1H = 0;
    
    CCP1CON2Hbits.OCBEN = 1; // Enable output signal (OC1B/RB8/pin17)
    CCP1CON3H = 0;
    CCP1CON1Lbits.TMRPS = 0b00;
    // CCP1CON3Hbits.OUTM = 0b000; // Set advanced output modes (Standard output)
    // CCP1CON3Hbits.POLACE = 0;   //Configure output polarity (Active High)
    CCP1TMRL = 0; //Initialize timer prior to enable module.
    CCP1PRL = MAXPWM;  //Configure timebase period
    CCP1RA = 0;   // Set the rising edge compare value
    CCP1RB = MAXPWM/2; // Set the falling edge compare value (duty = 50%)
    CCP1CON1Lbits.CCPON = 1; // Turn on MCCP module
}
