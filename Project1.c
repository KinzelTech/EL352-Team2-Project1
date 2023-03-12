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

//Program States
enum states
{
    INIT,           //Resets variables for new game.
    WAIT,           //Waits for player to serve the ball.
    TIMING,         //Delay between ball movements.
    RIGHT_TO_LEFT,  //Shifts the ball to the left. 
    LEFT_TO_RIGHT,  //Shifts the ball to the right.
    SCORE           //Signals and records scores and fouls.
};

//Constants
#define LEFT           1    //Identifier for the left player.
#define RIGHT          2    //Identifier for the right player.
#define DEBOUNCE_DELAY 10   //Delay to prevent button bounce errors.
#define BASE_SPEED     300  //Base speed of the ball movements.
#define MIN_SPEED      50   //Minimum possible speed variation.
#define MAX_SPEED      100  //Maximum possible speed variation.

//Function Prototypes
unsigned int get_left_button (void);                    //Return the state of the left button.
unsigned int get_right_button(void);                    //Return the state of the right button.
unsigned int shift_ball      (unsigned int dir);        //Shift the ball once in a direction.
unsigned int set_rand_speed  (void);                    //Set a random ball speed.
void set_ball                (unsigned char position);  //Set the ball at a position on the court.
void signal_foul             (void);                    //Flash the foul LED.
void signal_score            (unsigned int player);     //Flash the appropriate score LED.
void configure_interrupts    (void);                    //Configure timer interrupts.            
void configure_pins          (void);                    //Configure pin directions and initial outputs.
void clear_court             (void);                    //Turn off all court LEDs.

//Global Variables
volatile unsigned int timer_tick = 0;       //Increment every 1ms based upon Timer0.
unsigned char ball               = 0x00;    //Current position of the ball.
unsigned int  cur_state          = INIT;    //Current state of the program.

//Main Function
void main()
{
    unsigned int direction;
    unsigned int left_score  = 0;
    unsigned int right_score = 0;
    unsigned int scorer      = 0;
    unsigned int speed       = BASE_SPEED;
    
    configure_pins();
    configure_interrupts();

    //Infinite while loop acts as operating system.
    while(1)
    {
        //Select the current state of program execution.
        switch(cur_state)
        {
            //State to reset game and configure variables.
            case INIT:
                //srand(time(0));
                left_score  = 0;        //Reset left's score.
                right_score = 0;        //Reset right's score.
                cur_state   = WAIT;     //Change state to WAIT to wait on serve.
                break;
            
            //State to wait for player to serve the ball.
            case WAIT:
                speed = set_rand_speed();   //Set the ball's speed.
                if(get_left_button())       //Check if the left button was pressed.
                {
                    set_ball(0x80);         //Start ball on left-most edge.
                    direction = RIGHT;      //Set ball to move to the right.
                    cur_state = TIMING;     //Change to TIMING state to wait for next ball movement.
                }
                if(get_right_button())      //Check if the right button was pressed.
                {
                    set_ball(0x01);         //Start ball on right-most edge.
                    direction = LEFT;       //Set ball to move to the left.
                    cur_state = TIMING;     //Change to TIMING state to wait for next ball movement.
                }
                timer_tick = 0;             //Reset the timer.
                break;
            
            //State for waiting between ball movements.
            case TIMING:
                if(timer_tick == speed)                 //Shift ball after specified delay.
                {
                    if(direction == LEFT)
                        cur_state = RIGHT_TO_LEFT;      //Change state to RIGHT_TO_LEFT if moving left.
                    if(direction == RIGHT)
                        cur_state = LEFT_TO_RIGHT;      //Change state to LEFT_TO_RIGHT if moving right.
                }
                else                                    //Check for button presses.
                {
                    if(get_left_button())               //Check for left button press.
                    {
                        if(ball != 0x80)                //Check if ball is not on the left-most edge.
                        {
                            scorer    = RIGHT;          //Right gets point since left swung at invalid ball. 
                            cur_state = SCORE;          //Change state to SCORE.
                        }
                        else                            //Ball is on the left-most edge.
                        {
                            direction = RIGHT;          //Ball now moves right after successful hit.
                            speed = set_rand_speed();   //Change the speed of the ball.
                            
                        }
                    }
                    else if(get_right_button())         //Check for right button press.
                    {
                        if(ball != 0x01)                //Check if ball is not on the right-most edge.
                        {
                            scorer    = LEFT;           //Left gets point since right swung at invalid ball.
                            cur_state = SCORE;          //Change state to SCORE.
                        }
                        else                            //Ball is on the right-most edge.
                        {
                            direction = LEFT;           //Ball now moves left after successful hit.
                            speed = set_rand_speed();   //Change the speed of the ball.
                        }
                    }
                }
                break;
            
            //State to move ball right-to-left.
            case RIGHT_TO_LEFT:
                timer_tick = 0;             //Reset the timer.
                scorer = shift_ball(LEFT);  //Shift ball left, return scorer if out-of-bounds.
                if(scorer)                  
                    cur_state = SCORE;      //Change state to SCORE if a player scored.
                else
                    cur_state = TIMING;     //Change state to TIMING to wait for next ball movement.
                break;
            
            //State to move ball left-to-right.
            case LEFT_TO_RIGHT:
                timer_tick = 0;             //Reset the timer.
                scorer = shift_ball(RIGHT); //Shift ball right, return scorer if out-of-bounds.
                if(scorer)          
                    cur_state = SCORE;      //Change state to SCORE if player scored.
                else
                    cur_state = TIMING;     //Change state to TIMING to wait for next ball movement.
                break;
            
            //State to signal scores and fouls
            case SCORE:
                signal_foul();              //Flash foul LED.
                if(scorer == RIGHT)         //Check if right was the scorer.
                {
                    right_score++;          //Increment right's score by 1.
                    signal_score(RIGHT);    //Flash right's score LED.
                }
                else if(scorer == LEFT)     //Check if left was the scorer.
                {
                    left_score++;           //Increment left's score by 1.
                    signal_score(LEFT);     //Flash left's score LED.   
                }
                scorer = 0;                 //Reset scorer to none.
                clear_court();              //Turn off all court LEDs.
                cur_state = WAIT;           //Change state to WAIT to wait for next serve.
                break;
        }
    }
    return; //End of Main
}

//Configure pin directions and initial states.
void configure_pins(void)
{
    ANSELA = 0x00;  //Disable analog input on port A.
    ANSELB = 0x00;  //Disable analog input on port B.
    ANSELC = 0x00;  //Disable analog input on port C.
    ANSELD = 0x00;  //Disable analog input on port D.
    ANSELE = 0x01;  //Disable analog input on port E except RE0.
    TRISA  = 0x20;  //Set port A to Output except RA5.
    TRISB  = 0x01;  //Set port B to Output except RB0.
    TRISC  = 0x00;  //Set port C to Output.
    TRISD  = 0x00;  //Set port D to Output.
    TRISE  = 0x01;  //Set port E to Output except RE0.
    LATA   = 0x00;  //Set port A to initial output LOW.
    LATB   = 0x00;  //Set port B to initial output LOW.
    LATC   = 0x00;  //Set port C to initial output LOW.
    LATD   = 0x00;  //Set port D to initial output LOW.
    LATE   = 0x00;  //Set port E to initial output LOW.
    return;
}

//Configure program interrupts and hardware timer.
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

//Sets the ball to a specific position on the court.
void set_ball(unsigned char position)
{
    LATD = ball = position;   //Set the ball and turn on the correct LED.
    return;
}

//Shifts the ball once in a direction, returns if an out-of-bounds score occurred.
unsigned int shift_ball(unsigned int dir)
{
    unsigned char prior_position = ball;    //Position of ball prior to shift.
    unsigned char current_position;         //Position of ball after shift.
    unsigned int scorer = 0;                //Scorer ID if out-of-bounds score occurred.
    
    if(dir == LEFT)                         //Check the direction to shift the ball
        LATD = ball = (unsigned char) (prior_position << 1);    //Shift the ball left.
    else if(dir == RIGHT)
        LATD = ball = (unsigned char) (prior_position >> 1);    //Shift the ball right.
    current_position = ball;                //Record the ball's shifted position.
    
    if(current_position == 0)               //Check if the ball is outside the court.
    {
        if(prior_position == 0x80)  
            scorer = RIGHT;                 //Reward point to right if left fouled.
        else if(prior_position == 0x01)  
            scorer = LEFT;                  //Reward point to left if right fouled.
    }
    return scorer;                          //Return the id of the scoring player, otherwise zero.
}

//Set a random delay between led changes in ms.
unsigned int set_rand_speed()
{
    unsigned int speed;  
    if(rand() % 2 == 0)
        speed = BASE_SPEED + (rand() % (MAX_SPEED - MIN_SPEED + 1)) + MIN_SPEED;
    else
        speed = BASE_SPEED - (rand() % (MAX_SPEED - MIN_SPEED + 1)) + MIN_SPEED;
    return speed;
}

//Returns the status of the left button.
unsigned int get_left_button(void)
{
    if(PORTBbits.RB0 == 0)          //Check if left button was pressed.
    {
        __delay_ms(DEBOUNCE_DELAY); //Delay response to prevent debounce errors.
        return 1;                   //Return 1 to signal button press.
    }
    return 0;                       //If not pressed, return zero.
}

//Returns the status of the right button.
unsigned int get_right_button(void)
{
    if(PORTAbits.RA5 == 0)          //Check if right button was pressed.
    {
        __delay_ms(DEBOUNCE_DELAY); //Delay response to prevent debounce errors.
        return 1;                   //Return 1 to signal button press.
    }
    return 0;                       //If not pressed, return zero.
}

//Flash the foul LED.
void signal_foul(void)
{
    LATAbits.LATA2 = 1;     //Turn on the foul LED.
    __delay_ms(1000);       //Delay 1 second.
    LATAbits.LATA2 = 0;     //Turn off the foul LED.
    return;
}

//Signal and record that a score occurred.
void signal_score(unsigned int player)
{
    if(player == RIGHT)
        LATAbits.LATA1 = 1;                 //Turn on the right LED if right player scored.
    else if(player == LEFT)
        LATAbits.LATA0 = 1;                 //Turn on the left LED if left player scored.
    __delay_ms(500);                        //Delay 1/2 seconds.
    LATAbits.LATA1 = LATAbits.LATA0 = 0x00; //Clear both player's LEDs.
    return;
}



//Increments the timer every millisecond.
void __interrupt() isr(void)
{
   if(TMR0IE && TMR0IF) //Increment time if Timer0 is enabled and Timer0 has overflowed.
   {
      TMR0IF = 0;       //Reset Timer0 overflow flag.
      timer_tick++;     //Increment time.
      TMR0L = 0xF0;     //Reset preload to 240.
   }
   return;
}

//Turn off all court LEDs.
void clear_court(void)
{
    LATD = ball = 0x00;     //Set all court LEDs to zero and reset ball position.
    return;
}
