/*
Author:  sbaldzenka
Project: tiny13_night_light
Data  :  12.07.2023
E-mail:  venera.electronica@gmail.com
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>

//----------------------------------------------------------//
// Global variables                                         //
//----------------------------------------------------------//
uint8_t duty = 0;
uint8_t updown = 0;

ISR(WDT_vect)
{
    PORTB |= (1 << PB3);
    _delay_ms(1000);
    PORTB &= ~(1 << PB3);
    _delay_ms(1000);

    if(updown == 1)
    {
        TCCR0B |= (1 << CS01) | (1 << CS00);         // Enable Timer/Counter #0 with clk divider = 64

        while(duty > 0)
        {
            duty--;
            OCR0A = duty;
            _delay_ms(5);
        }

        OCR0A = 0x00;
        TCCR0B = 0x00;                               // Disable Timer/Counter #0
        updown = 0;
    }

    reti();                                          // Return to main function
}

void main(void)
{
    //------------------------------------------------------//
    // GPIO                                                 //
    //------------------------------------------------------//
    // PORTB0 = out
    // PORTB1 = out
    // PORTB3 = out
    DDRB |= (1 << PB3) | (1 << PB2) | (1 << PB1) | (1 << PB0);
    // PORTB2 = in
    // PORTB4 = in
    DDRB &= ~(1 << PB4) & ~(1 << PB2);
    // PORTB initial value
    PORTB = 0x00;

    //------------------------------------------------------//
    // ADC                                                  //
    //------------------------------------------------------//
    // Vref = Vcc
    ADMUX &= ~(1 << REFS0);
    // ADC channel #1
    // ADC left adjust result
    ADMUX |= (1 << MUX0) | (1 << ADLAR);
    // ADC enable
    // Free Running mode
    ADCSRA |= (1 << ADEN) | (1 << ADATE);
    // Digital pin disable
    DIDR0 |= (1 << ADC1D);

    //------------------------------------------------------//
    // Timer/Counter #0                                     //
    //------------------------------------------------------//
    // Pin B0 is PWM out
    TCCR0A |= (1 << COM0A1);
    // Fast PWM mode
    TCCR0A |= (1 << WGM01) | (1 << WGM00);

    //------------------------------------------------------//
    // Watchdog timer                                       //
    //------------------------------------------------------//
    // WDT enable
    WDTCR |= (1 << WDE) | (1 << WDTIE);
    // 8 seconds period
    WDTCR |= (1 << WDP3) | (1 << WDP0);

    sei();                                           // Global interrupt enable

    while(1)
    {
        ADCSRA |= (1 << ADSC);                       // Read ADC value

        if(ADCH > 0x80)
        {
            PORTB |= (1 << PB1);

            if(0 != (PINB & (1 << PB4)))
            {
                wdt_reset();                         // Watchdog timer reset
                TCCR0B |= (1 << CS01) | (1 << CS00); // Enable Timer/Counter #0 with clk divider = 64
                OCR0A = duty;

                if(updown == 0)
                {
                    for(duty = 0; duty < 255; duty++)
                    {
                        OCR0A = duty;
                        _delay_ms(5);
                    }

                    if(duty == 255)
                    {
                        updown = 1;
                    }
                }
            }
            else
            {
                if(updown == 1)
                {
                    for(duty = 255; duty > 0; duty--)
                    {
                        OCR0A = duty;
                        _delay_ms(5);
                    }

                    if(duty == 0)
                    {
                        updown = 0;
                    }
                }
                else
                {
                    OCR0A = 0x00;
                    TCCR0B = 0x00;                   // Disable Timer/Counter #0
                }
            }
        }
        else
        {
            PORTB &= ~(1 << PB1);
            updown = 0;

            while(duty > 0)
            {
                duty--;
                OCR0A = duty;
                _delay_ms(5);
            }

            OCR0A = 0x00;
            TCCR0B = 0x00;                          // Disable Timer/Counter #0
        }
    }
}
