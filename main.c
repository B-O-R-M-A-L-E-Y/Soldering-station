
// ---------------------- HAL layer ---------------------------
#define F_CPU 8000000UL
#define PRESCALER 64
#define TIMER_DIVIDER ((F_CPU/PRESCALER/1000)-1)
#define BAUDRATE 9600
#define PWM_REG OCR2B
// ------------------------------------------------------------

#include <avr/io.h>
#include <stdlib.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "macros.h"
#include "encoder.h"
#include "pid.h"

#define CHANGE_PWM_REG(count) PWM_REG=count

#define ADC_SCALING_FACTOR 100
#define BIT_TO_TEMP 80


#define K_P 20.0
#define K_I 0.1
#define K_D 0.3

void change_state();
void init_pins();
void init_clock_service();
void init_uart();
void init_adc();
void init_pwm();
void init_pid();

void uart_send_symbol(unsigned char);
void uart_send_string(char*);
void uart_send_data();
void do_adc_convertion();

char UART_SYMBOL_BUFFER;
char UART_STRING_BUFFER[20];

uint32_t SYSTEM_CLOCK=0;
uint32_t ADC_RESULT=0;
uint32_t measure_temp=200;
uint16_t set_temp=220;
uint8_t PWM_VALUE=0;
int32_t PID_VALUE=0;

pidData_t PID_Data_struct;

uint8_t state=1;

int main()
{
	init_clock_service();
  init_pins();
  init_encoder();
  init_uart();
  init_pwm();
  init_adc();
  init_pid();
  uint32_t prev_clk_state=0;
  uint32_t prev_pid=0;
  uint32_t prev_adc=0;
  uint32_t prev_debug=0;
  while(1)
  {
  	if ((SYSTEM_CLOCK-prev_clk_state>1000)&&(~(SYSTEM_CLOCK<=prev_clk_state)))
  	{
  		prev_clk_state=SYSTEM_CLOCK;
  		change_state();
  	}
  	
  	check_encoder_state();
  	uint8_t spin=get_encoder_state();
  	if (spin==SPIN_RIGHT) set_temp++;
  	if (spin==SPIN_LEFT) set_temp--;
  	if ((SYSTEM_CLOCK-prev_debug>3000)&&(~(SYSTEM_CLOCK<=prev_debug)))
  	{
  		prev_debug=SYSTEM_CLOCK;
  		uart_send_string("SET_TEMP: \0");
  		uart_send_string(itoa(set_temp, UART_STRING_BUFFER, 10));
  		uart_send_symbol(13); //Line End
  		uart_send_string("ADC_DATA: \0");
  		uart_send_string(itoa(ADC_RESULT, UART_STRING_BUFFER, 10));
  		uart_send_symbol(13);
  		uart_send_string("MES_TEMP: \0");
  		uart_send_string(itoa(measure_temp, UART_STRING_BUFFER, 10));
  		uart_send_symbol(13);
  		uart_send_string("PID_VALUE: \0");
  		uart_send_string(itoa(PID_VALUE, UART_STRING_BUFFER, 10));
  		uart_send_symbol(13);
  		uart_send_string("PWM_VALUE: \0");
  		uart_send_string(itoa(PWM_VALUE, UART_STRING_BUFFER, 10));
  		uart_send_symbol(13);
  		uart_send_string("--------------------\n\0");
  	}
  	//ADC Convertion run
  	if((SYSTEM_CLOCK-prev_adc>1000)&&(~(SYSTEM_CLOCK<=prev_adc)))
  	{
  		prev_adc=SYSTEM_CLOCK;
  		do_adc_convertion();
  		//pid_Controller(set_temp,measure_temp,PID_Data);
  		//if (pid_value<0) pwm_value=0;
  		//else pwm_value= (uint8_t) pid_value;
  	}
  	//PID scan run and set pwm
  	if((SYSTEM_CLOCK-prev_pid>200)&&(~(SYSTEM_CLOCK<=prev_pid)))
  	{
  		prev_pid=SYSTEM_CLOCK;
  		PID_VALUE = pid_Controller(set_temp,measure_temp,&PID_Data_struct);
  		if (PID_VALUE<0) PWM_VALUE=0;
  		else PWM_VALUE = PID_VALUE/128;
  		CHANGE_PWM_REG(PWM_VALUE);
  	}
  }
}

void init_pins()
{
  SET_BIT(DDRD, DDD2);
  CLEAR_BIT(PORTD, PORTD2);
}

void change_state()
{
  if(state==0)
  {
    SET_BIT(PORTD, PORTD2);
    state=1;
    return;
  }
  else
  {
    CLEAR_BIT(PORTD, PORTD2);
    state=0;
    return;
  }
}


void init_pid()
{
	pid_Init(K_P*SCALING_FACTOR, K_I*SCALING_FACTOR, K_D*SCALING_FACTOR, &PID_Data_struct);
}

void init_clock_service()
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

void init_adc()
{
  CLEAR_BIT(DDRC, DDC0); // Configure PC0 as Input Hi-Z
  CLEAR_REG(ADMUX);
  CLEAR_BIT(ADMUX,MUX0); // set ADC input ADC0
  CLEAR_BIT(ADMUX,MUX1);
  CLEAR_BIT(ADMUX,MUX2);
  CLEAR_BIT(ADMUX,MUX3);
  SET_BIT(ADMUX, REFS0); //enable AVcc reference with ext capacitor at AREF
  CLEAR_REG(ADCSRA);
  CLEAR_BIT(ADCSRA, ADPS0); //set ADC prescaler = 4
  SET_BIT(ADCSRA, ADPS1);
  CLEAR_BIT(ADCSRA, ADPS2);
  SET_BIT(ADCSRA, ADEN); //enable ADC
}

void init_uart()
{
  UBRR0L = (F_CPU/BAUDRATE/16)-1;
  UBRR0H = ((F_CPU/BAUDRATE/16)-1)>>8;
  UCSR0C = (1<<USBS0)|(3<<UCSZ00);// Set frame format: 8data, 2stop bit
  UCSR0B = (1<<RXEN0)|(1<<TXEN0);
}

void init_pwm()
{
	SET_BIT(DDRD, DDD3); //Configure pin Out
	SET_BIT(TCCR2A,COM2B1); //Enable OC2A non-inverting mode
	CLEAR_BIT(TCCR2A,COM2B0);
	SET_BIT(TCCR2A,WGM20);
	SET_BIT(TCCR2A,WGM21);
	CLEAR_BIT(TCCR2B,WGM22);
	SET_BIT(TCCR2B,CS22); //Prescaler 64
	CLEAR_BIT(TCCR2B,CS21);
	CLEAR_BIT(TCCR2B,CS20);
	OCR2B=200;
}

void uart_send_symbol(unsigned char data)
{
  //while ( !(UCSR0A & (1 << RXC0)) ) // Wait until data is received
  //ReceivedChar = UDR0; // Read the data from the RX buffer
  cli();
  while ( !(UCSR0A & (1 << UDRE0)) ); // Wait until buffer is empty
  UDR0 = data; // Send the data to the TX buffer
  sei();
}

void uart_send_string(char *str)
{
	while (*str != '\0')
	{
		uart_send_symbol(*str);
		++str;
	}
}

/* Make conversion to Volts */
void do_adc_convertion()
{
	cli();
  uint32_t voltage_reference = 5*ADC_SCALING_FACTOR;
  SET_BIT(ADCSRA, ADSC); //start conversion
  while(IS_TRUE_BIT(ADCSRA, ADSC)){} //wait for conversion is completed
  uint32_t conversion_result = 0;
  conversion_result = ADCL; //read low byte, then high
  conversion_result |=  (ADCH<<8);
  ADC_RESULT = (conversion_result*voltage_reference/1024);
  measure_temp=BIT_TO_TEMP*ADC_RESULT/ADC_SCALING_FACTOR;
  sei();
}

/* Timer duty run in interrupt every 1 ms */
ISR(TIMER0_COMPA_vect)
{
	cli();
  SYSTEM_CLOCK++;
  sei();
}
