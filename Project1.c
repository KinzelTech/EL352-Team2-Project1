#include <xc.h>

#define _XTAL_FREQ 8000000  // tells XC8 the crystal frequency

// CONFIG1H
#pragma config FOSC     = HSMP  // Oscillator Selection bits->HS oscillator (medium power 4-16 MHz)
#pragma config PLLCFG   = OFF   // 4X PLL Enable->Oscillator used directly
#pragma config PRICLKEN = ON    // Primary clock enable bit->Primary clock is always enabled
#pragma config FCMEN    = OFF   // Fail-Safe Clock Monitor Enable bit->Fail-Safe Clock Monitor
                                // disabled
#pragma config IESO     = OFF   // Internal/External Oscillator Switchover bit->Oscillator
                                // Switchover mode disabled

// CONFIG2L
#pragma config PWRTEN = ON       // Power-up Timer Enable bit->Power up timer enabled
#pragma config BOREN  = SBORDIS  // Brown-out Reset Enable bits->Brown-out Reset enabled in
                                 // hardware only (SBOREN is disabled)
#pragma config BORV   = 190      // Brown Out Reset Voltage bits->VBOR set to 1.90 V nominal

// CONFIG2H
#pragma config WDTEN = OFF    // Watchdog Timer Enable bits->WDT is always OFF
#pragma config WDTPS = 2048   // Watchdog Timer Postscale Select bits->1:2048

// CONFIG3H
#pragma config CCP2MX = PORTB3   // CCP2 MUX bit->CCP2 input/output is multiplexed with RB3
#pragma config PBADEN = OFF      // PORTB<5:0> pins are configured as digital I/O on Reset
#pragma config CCP3MX = PORTB5   // P3A/CCP3 Mux bit->P3A/CCP3 input/output is multiplexed with RB5
#pragma config HFOFST = ON       // HFINTOSC Fast Start-up->HFINTOSC output and ready status are
                                 // not delayed by the oscillator stable status
#pragma config T3CMX  = PORTC0   // Timer3 Clock input mux bit->T3CKI is on RC0
#pragma config P2BMX  = PORTD2   // ECCP2 B output mux bit->P2B is on RD2
#pragma config MCLRE  = EXTMCLR  // MCLR Pin Enable bit->MCLR pin enabled, RE3 input pin disabled

// CONFIG4L
#pragma config STVREN = ON    // Stack Full/Underflow Reset Enable bit->Stack full/underflow will cause Reset
#pragma config LVP    = OFF   // Single-Supply ICSP Enable bit->Single-Supply ICSP disabled
#pragma config XINST  = OFF   // Extended Instruction Set Enable bit->Instruction set extension and
                              // Indexed Addressing mode disabled (Legacy mode)
#pragma config DEBUG  = OFF   // Background Debug->Disabled

// CONFIG5L
#pragma config CP0 = OFF    // Code Protection Block 0->Block 0 (000800-003FFFh) not code-protected
#pragma config CP1 = OFF    // Code Protection Block 1->Block 1 (004000-007FFFh) not code-protected
#pragma config CP2 = OFF    // Code Protection Block 2->Block 2 (008000-00BFFFh) not code-protected
#pragma config CP3 = OFF    // Code Protection Block 3->Block 3 (00C000-00FFFFh) not code-protected

// CONFIG5H
#pragma config CPB = OFF    // Boot Block Code Protection bit->Boot block (000000-0007FFh) not code-protected
#pragma config CPD = OFF    // Data EEPROM Code Protection bit->Data EEPROM not code-protected

// CONFIG6L
#pragma config WRT0 = OFF    // Write Protection Block 0->Block 0 
                             // (000800-003FFFh) not write-protected
#pragma config WRT1 = OFF    // Write Protection Block 1->Block 1 
                             // (004000-007FFFh) not write-protected
#pragma config WRT2 = OFF    // Write Protection Block 2->Block 2 
                             // (008000-00BFFFh) not write-protected
#pragma config WRT3 = OFF    // Write Protection Block 3->Block 3 
                             // (00C000-00FFFFh) not write-protected

// CONFIG6H
#pragma config WRTC = OFF    // Configuration Register Write Protection bit->Configuration
                             // registers (300000-3000FFh) not write-protected
#pragma config WRTB = OFF    // Boot Block Write Protection bit->Boot Block 
                             //(000000-0007FFh) not write-protected
#pragma config WRTD = OFF    // Data EEPROM Write Protection bit->Data EEPROM not write-protected

// CONFIG7L
#pragma config EBTR0 = OFF    // Table Read Protection Block 0->Block 0 (000800-003FFFh) not 
                              // protected from table reads executed in other blocks
#pragma config EBTR1 = OFF    // Table Read Protection Block 1->Block 1 (004000-007FFFh) not 
                              // protected from table reads executed in other blocks
#pragma config EBTR2 = OFF    // Table Read Protection Block 2->Block 2 (008000-00BFFFh) not   
                              // protected from table reads executed in other blocks
#pragma config EBTR3 = OFF    // Table Read Protection Block 3->Block 3 (00C000-00FFFFh) not 
                              // protected from table reads executed in other blocks

// CONFIG7H
#pragma config EBTRB = OFF    // Boot Block Table Read Protection bit->Boot Block (000000-0007FFh) 
                              // not protected from table reads executed in other blocks


//Constants
enum states
{
    INIT,
    WAIT,
    TIMING,
    RIGHT_TO_LEFT, 
    LEFT_TO_RIGHT,
    SCORE,
    WIN
};

#define LEFT           1
#define RIGHT          2
#define TRUE           1
#define FALSE          0
#define DEBOUNCE_DELAY 10
#define BASE_SPEED     300
#define MIN_SPEED      50
#define MAX_SPEED      100

unsigned int get_left_button (void);
unsigned int get_right_button(void);
void set_ball                (unsigned char position);
void shift_ball              (unsigned int dir, unsigned int *scorer);
void set_rand_speed          (unsigned int *speed);
void signal_foul             (void);
void signal_score            (unsigned int player);
void configure_interrupts    (void);
void configure_pins          (void);
void clear_court             (void);

volatile unsigned int timer_tick = 0;
unsigned char ball               = 0x00;
unsigned int last_button         = 1;
unsigned int cur_state           = INIT;

/*
 PINS:
 RD0-RD7    Court LEDs
 RA5        Left Button       
 RB0        Right Button        
 RA0        Left Score LED
 RA1        Right Score LED
 RA2        Foul LED
 RE0        Analog seeder    (holding off for now)
 */

void main()
{
    unsigned int direction;
    unsigned int left_score  = 0;
    unsigned int right_score = 0;
    unsigned int scorer      = 0;
    unsigned int speed       = BASE_SPEED;
    configure_pins();
    configure_interrupts();

    
    while(1)
    {
        switch(cur_state)
        {
            case INIT:
                //srand(time(0));
                left_score  = 0;
                right_score = 0;
                cur_state   = WAIT;
                break;
            
            case WAIT:
                set_rand_speed(&speed);
                if(get_left_button())
                {
                    set_ball(0x80);
                    direction = RIGHT;
                    cur_state = TIMING;
                }
                if(get_right_button())
                {
                    set_ball(0x01);
                    direction = LEFT;
                    cur_state = TIMING;
                }
                timer_tick = 0;
                break;
            
            case TIMING:
                if(timer_tick == speed)
                {
                    if(direction == LEFT)
                        cur_state = RIGHT_TO_LEFT;
                    if(direction == RIGHT)
                        cur_state = LEFT_TO_RIGHT;
                }
                
                if(get_left_button())
                {
                    if(ball != 0x80)
                    {
                        cur_state = SCORE;
                        scorer = RIGHT;
                    }
                    else
                    {
                        set_rand_speed(&speed);
                        direction = RIGHT;
                    }
                }
                else if(get_right_button())
                {
                    if(ball != 0x01)
                    {
                        scorer = LEFT;
                        cur_state = SCORE;
                    }
                    else
                    {
                        set_rand_speed(&speed);
                        direction = LEFT;
                    }
                }
                break;
            
            case RIGHT_TO_LEFT:
                timer_tick = 0;
                shift_ball(LEFT, &scorer);
                if(scorer)
                    cur_state = SCORE;
                else
                    cur_state = TIMING;
                break;
            
            case LEFT_TO_RIGHT:
                timer_tick = 0;
                shift_ball(RIGHT, &scorer);
                if(scorer)
                    cur_state = SCORE;
                else
                    cur_state = TIMING;
                break;
            
            case SCORE:
                signal_foul();
                if(scorer == RIGHT)
                {
                    right_score++;
                    signal_score(RIGHT);
                    cur_state = WAIT;
                }
                else if(scorer == LEFT)
                {
                    left_score++;
                    signal_score(LEFT);
                    cur_state = WAIT;
                }
                scorer = 0;
                clear_court();
                break;
            
            //WIN case is only for an optional bonus.
            case WIN:
                break;
        }
    }
    return;
}

void set_ball(unsigned char position)
{
    LATD = ball = position;
    return;
}

void shift_ball(unsigned int dir, unsigned int *scorer)
{
    unsigned char prior_value = ball;
    unsigned char current_value;
    unsigned char increment = 1;
    if(dir == LEFT)
        LATD = ball = (unsigned char) (prior_value << increment);
    else if(dir == RIGHT)
        LATD = ball = prior_value >> increment;
    
    current_value = ball;
    if(current_value == 0)
    {
        if(prior_value == 0x80)
            *scorer = RIGHT;
        else if(prior_value == 0x01)  
            *scorer = LEFT;
    }
    return;
}

//Set the delay between led changes in ms.
void set_rand_speed(unsigned int *speed)
{
    if(rand() % 2 == 0)
        *speed = BASE_SPEED + (rand() % (MAX_SPEED - MIN_SPEED + 1)) + MIN_SPEED;
    else
        *speed = BASE_SPEED - (rand() % (MAX_SPEED - MIN_SPEED + 1)) + MIN_SPEED;
    return;
}

//Returns the status of the left button then resets it.
unsigned int get_left_button(void)
{
    unsigned int button_status = PORTBbits.RB0;
    if(button_status == 0)
    {
        __delay_ms(DEBOUNCE_DELAY);
        if(button_status == 0)
            return 1;
    }
    return 0;
}


unsigned int get_right_button(void)
{
    unsigned int button_status = PORTAbits.RA5;
    if(button_status == 0)
    {
        __delay_ms(DEBOUNCE_DELAY);
        if(button_status == 0)
            return 1;
    }
    return 0;
}


void signal_foul(void)
{
    ; //Add commands here to light foul led for 500ms.
    LATAbits.LATA2 = 1;
    __delay_ms(1000);
    LATAbits.LATA2 = 0;
    
    return;
}


void signal_score(unsigned int player)
{
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    if(player == RIGHT)
        LATAbits.LATA1 = 1;
    else if(player == LEFT)
        LATAbits.LATA0 = 1;
    __delay_ms(500);
    LATA = 0x00;
    return;
}


void configure_interrupts(void)
{
    RCONbits.IPEN       = 0;	//Disable interrupt priority levels
    INTCONbits.GIE      = 0;	//Disable interrupts.
    T0CON    		    = 0x46;	//Configure to 8bit timer with 1:128 prescale.
    TMR0L    		    = 0xF0;	//Set preload to 240, increment from 240-256
    INTCONbits.TMR0IE   = 1;    //Enable Timer0 interrupts.
    INTCONbits.GIE      = 1;	//Enable interrupts.
    T0CONbits.TMR0ON    = 1;	//Turn on Timer0.
                                //Calcs = 4*prescale(256-preload)/f
                                //      = 4*128(256-240)/8M = 1.024ms
    return;
}


void configure_pins(void)
{
    ANSELA = 0x00;  //Disable analog input on port A
    ANSELB = 0x00;  //Disable analog input on port B
    ANSELC = 0x00;  //Disable analog input on port C
    ANSELD = 0x00;  //Disable analog input on port D
    ANSELE = 0x01;  //Disable analog input on port E except RE0.
    
    TRISA  = 0x20;  //Set port A to Output except RA5.
    TRISB  = 0x01;  //Set port B to Output except RB0.
    TRISC  = 0x00;  //Set port C to Output.
    TRISD  = 0x00;  //Set port D to Output.
    TRISE  = 0x01;  //Set port E to Output except RE0.
    
    LATA = 0x00;    //Set port A to initial output LOW.
    LATB = 0x00;    //Set port B to initial output LOW.
    LATC = 0x00;    //Set port C to initial output LOW.
    LATD = 0x00;    //Set port D to initial output LOW.
    LATE = 0x00;    //Set port E to initial output LOW.
    return;
}


void __interrupt() isr(void)
{
   if(TMR0IE && TMR0IF) //if Timer0 is enabled and Timer0 has overflowed.
   {
      TMR0IF = 0;               //Reset Timer0 overflow flag.
      timer_tick++;             //Increment timer_tick.
      TMR0L = 0xF0;             //Reset preload to 240.
   }
   return;
}

void clear_court(void)
{
    LATD = ball = 0x00;
    return;
}
