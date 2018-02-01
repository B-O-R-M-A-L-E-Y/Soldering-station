#include "timer_service.h"
#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/interrupt.h>

// ----------------------------- HAL layer ------------------------------------
#define F_CPU 8000000
#define PRESCALER 64
#define TIMER_DIVIDER ((F_CPU/PRESCALER/1000)-1)
// ------------------------------------------------------------

static uint32_t SYSTEM_CLOCK=0;

void init_timer_service()
{
	cli();
  TCCR0B = 0; /* Stop TC0 */
  OCR0A = 0; /* Clear TC0 counter */
  TCCR0A |= _BV(WGM01); /* CTC mode */
  OCR0A = TIMER_DIVIDER; /* Counter value */
  TIMSK0 |= _BV(OCIE0A); /* Set the ISR COMPA vect */
  TCCR0B |= _BV(CS01)|_BV(CS00); /*Prescaler 64 and start the timer*/
  sei();
}

/* Timer duty run in interrupt every 1 ms */
ISR(TIMER0_COMPA_vect)
{
	cli();
  SYSTEM_CLOCK++;
  sei();
}
