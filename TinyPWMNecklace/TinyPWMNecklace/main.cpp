/*
 * TinyPWMNecklace.cpp
 *
 * Created: 5/14/2019 9:18:52 PM
 * Author : Mark Ferrari
 *
 * Dead bug necklace with four PWM LEDs controlled by ATtiny85
 * Four-PWM technique stolen from http://www.technoblogy.com/show?LE0
 *
 *    Physical pin: Buffer bit: Logical pin
 *    2: PB3: Simulated OC1A using interrupts
 *    3: PB4: OC1B
 *    5: PB0: OC0A
 *    6: PB1: OC0B
 *
 * 17 May 2019 version
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>

// List of compare registers and physical indexes into it
volatile uint8_t* brite[] = {&OCR0A, &OCR0B, &OCR1A, &OCR1B};
const int pin_t = 2;
const int pin_r = 3;
const int pin_b = 0;
const int pin_l = 1;

void delay_loop(void);
void blink(int pin);
void shift(int p_old, int p_new);


int main(void)
{
    // Set relevant pins to output
    DDRB |= 1 << PB0 | 1 << PB1 | 1 << PB3 | 1 << PB4;
    
    // Set Timer/Counter0 to fast PWM, inverting mode (255=off, 0=full on)
    //     TC0 drives OC0A and OC0B directly
    //     For some reason, non-inverting mode is never completely off
    TCCR0A |= 0b11 << COM0A0 | 0b11 << COM0B0 | 0b11 << WGM00;
    
    // Clock for TC0 divided by 64 for consistency with Arduino.  Necessary?
    TCCR0B |= 0b11 << CS00;
    
    // Set Timer/Counter1 to PWM inverting mode
    //     TC1 drives OC1B directly and PB3 via interrupt routines
    GTCCR |= 1 << PWM1B | 0b11 << COM1B0;
    
    // Clock for TC1 divided by 64
    TCCR1 |= 0b111 << CS10;
    
    // PWM will not work on OC1B unless COM1Ai are also set (bug in ATtiny85)
    TCCR1 |= 0b11 << COM1A0;
    
    // Enable interrupts on TC1 match to OCR1A and on TC1 overflow
    TIMSK |= 1 << OCIE1A | 1 << TOIE1;
    sei();
    
    *brite[pin_t] = *brite[pin_r] = *brite[pin_b] = *brite[pin_l] = 255;
    
    while (1) {
        shift(pin_t, pin_r);
        shift(pin_r, pin_b);
        shift(pin_b, pin_l);
        shift(pin_l, pin_t);
        
        /*
        *brite[pin_l] = 255;
        *brite[pin_t] = 0;
        delay_loop();
        
        *brite[pin_t] = 255;
        *brite[pin_r] = 0;
        delay_loop();
        
        *brite[pin_r] = 255;
        *brite[pin_b] = 0;
        delay_loop();
        
        *brite[pin_b] = 255;
        *brite[pin_l] = 0;
        delay_loop();        
        */
    }
}

ISR(TIMER1_COMPA_vect)
{
    if (!((TIFR >> TOV1) & 1)) PORTB |= 1 << PB3;
}

ISR(TIMER1_OVF_vect)
{
    PORTB &= ~(1 << PB3);
}

void shift(int p_old, int p_new)
{
    int i;
    for (i = 1; i < 256; i++) {
        *brite[p_old] = i;
        *brite[p_new] = 255 - i;
        delay_loop();
    }
}

void blink(int pin)
{
    PORTB &= ~(1 << pin);
    delay_loop();
    PORTB |= 1 << pin;
    delay_loop();

}

void delay_loop(void)
{
    volatile unsigned int del = 200;
    
    while(del--);
}
