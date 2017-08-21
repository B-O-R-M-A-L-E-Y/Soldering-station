#define F_CPU 8000000
#define BAUDRATE 9600

#include <avr/io.h>
#include <stdlib.h>
#include "macros.h"
#include "encoder.h"
#include "util/delay.h"


volatile unsigned int set_temp=40;

//void init_pwm();
void init_uart();

void do_encoder_scan();
void uart_send_symbol(unsigned char);
void uart_send_string(char*);
uint8_t uart_recieve_symbol();

char UART_SYMBOL_BUFFER;
char UART_STRING_BUFFER[5];

int main(void)
{
  //init_pwm();
  init_encoder();
  init_uart();
  while (1)
  {
    do_encoder_scan();
    uart_send_string("OKDA\n");
    uart_send_string(itoa(set_temp, UART_STRING_BUFFER, 10));
    uart_send_symbol(13);
  }
}

/* ----------------------------------------------------------------------------
-------------------------------- INIT BLOCK -----------------------------------
---------------------------------------------------------------------------- */

void init_pwm()
{
  SET_BIT(DDRD, DDD6); // Configure PD6 as Output
  SET_BIT(TCCR0A, COM0A1); // Clear OC0A at Compare match, set at Bottom
  CLEAR_BIT(TCCR0A, COM0A0); // Non-inverting mode
  SET_BIT(TCCR0A, WGM00); // Configure Fast PWM Waveform Generation Mode
  SET_BIT(TCCR0A, WGM01); // Top=0xFF, Update at BOTTOM
  CLEAR_BIT(TCCR0B, WGM02);
  SET_BIT(TCCR0B, CS02); // Set prescaler 1024 and run timer
  CLEAR_BIT(TCCR0B, CS01);
  SET_BIT(TCCR0B, CS00);
  OCR0A=40;
}

void init_uart()
{
  UBRR0L = (F_CPU/BAUDRATE/16)-1;
  UBRR0H = ((F_CPU/BAUDRATE/16)-1)>>8;
  UCSR0C = (1<<USBS0)|(3<<UCSZ00);// Set frame format: 8data, 2stop bit
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
}

/* ----------------------------------------------------------------------------
------------------------------- END OF INIT BLOCK -----------------------------
---------------------------------------------------------------------------- */

void uart_send_symbol(unsigned char data)
{
  //while ( !(UCSR0A & (1 << RXC0)) ) // Wait until data is received
  ///ReceivedChar = UDR0; // Read the data from the RX buffer
  while ( !(UCSR0A & (1 << UDRE0)) ); // Wait until buffer is empty
  UDR0 = data; // Send the data to the TX buffer
}

void uart_send_string(char *str)
{
	while (*str != '\0')
	{
		uart_send_symbol(*str);
		++str;
	}
}

uint8_t uart_recieve_symbol()
{
  while ( !(UCSR0A & (1 << RXC0)) ); // Wait until data is received
  UART_SYMBOL_BUFFER = UDR0; // Read the data from the RX buffer
  return UART_SYMBOL_BUFFER;
}

void do_encoder_scan()
{
  check_encoder_state();
  switch (get_encoder_state()) {
    case SPIN_LEFT:
      set_temp--;
      break;
    case SPIN_RIGHT:
      set_temp++;
      break;
    default:
      break;
  }
}
