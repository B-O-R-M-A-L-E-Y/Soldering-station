#define F_CPU 8000000

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

#include "pid.h"
#include "macros.h"
#include "encoder.h"

#define SOUND_ON OCR0B=64
#define SOUND_OFF OCR0B=0

#define Prop_FACTOR 4
#define Diff_FACTOR 0
#define Integ_FACTOR 0
struct PID_DATA* REGULATOR_DATA;

uint16_t set_temp=250;
uint16_t current_temp=20;
float vol_to_temp = 55; //????
char is_heated_first_time = 1;

/*TODO
Раскидать функции в заголовочные файлы
Настроить библиотеку дисплея
Настроить АЦП в соответствии с напряжением термопары и передачей через ОУ
*/

void init_all();
void init_adc(); // ADC0 on PC0
void init_pwm(); // OC0A on PD6
void init_pwm_sound(); // OC0B on PD5 !!Make sure it runs AFTER init_pwm func

float do_adc_convertion();
void start_pwm_duty(uint8_t);
void do_encoder_scan();
void pwm_control();

int main()
{
  init_all();
  do_encoder_scan();
  pwm_control();
}

/* ----------------------------------------------------------------------------
-------------------------------- INIT BLOCK -----------------------------------
---------------------------------------------------------------------------- */

// configure all ports & timers
void init_all()
{
  init_pwm();
  init_pwm_sound();
  SOUND_ON;
  init_adc();
  init_encoder(); // Check define
  pid_Init(Prop_FACTOR, Integ_FACTOR, Diff_FACTOR, REGULATOR_DATA);
  SOUND_OFF;
}

void init_adc()
{
  CLEAR_BIT(DDRC, DDC0); // Configure PC0 as Input Hi-Z
  CLEAR_REG(ADMUX);
  CLEAR_BIT(ADMUX,MUX0); //set ADC input ADC0
  CLEAR_BIT(ADMUX,MUX1);
  CLEAR_BIT(ADMUX,MUX2);
  CLEAR_BIT(ADMUX,MUX3);
  SET_BIT(ADMUX, REFS0); // enable AVcc reference with ext capacitor at AREF
  CLEAR_REG(ADCSRA);
  CLEAR_BIT(ADCSRA, ADPS0); //set ADC prescaler = 4
  SET_BIT(ADCSRA, ADPS1);
  CLEAR_BIT(ADCSRA, ADPS2);
  SET_BIT(ADCSRA, ADEN); //enable ADC
}

void init_pwm()
{
  SET_BIT(DDRD, DDD6); // Configure PD6 as Output
  SET_BIT(TCCR0A, WGM00); // Configure Fast PWM Waveform Generation Mode
  SET_BIT(TCCR0A, WGM01); // Top=0xFF, Update at BOTTOM
  CLEAR_BIT(TCCR0B, WGM02);
  SET_BIT(TCCR0A, COM0A0); //Set Fast PWM Inverting Mode
  SET_BIT(TCCR0A, COM0A1);
  SET_BIT(TCCR0B, CS01); // Set prescaler 64 and run timer
  SET_BIT(TCCR0B, CS02);
}

void init_pwm_sound()
{
  SET_BIT(DDRD, DDD6);
  SET_BIT(TCCR0A, WGM00); // Configure Fast PWM Waveform Generation Mode
  SET_BIT(TCCR0A, WGM01); // Top=0xFF, Update at BOTTOM
  CLEAR_BIT(TCCR0B, WGM02);
  SET_BIT(TCCR0A, COM0B0); //Set Fast PWM Inverting Mode
  SET_BIT(TCCR0A, COM0B1);
}

void init_display()
{

}

/* ----------------------------------------------------------------------------
------------------------------- END OF INIT BLOCK -----------------------------
---------------------------------------------------------------------------- */

float do_adc_convertion()
{
  const uint8_t voltage_reference = 5;
  SET_BIT(ADCSRA, ADSC); //start conversion
  while(IS_TRUE_BIT(ADCSRA, ADSC)){} //wait for conversion is completed
  uint16_t conversion_result = 0;
  conversion_result = ADCL; //read low byte, then high
  conversion_result |=  (ADCH<<8);
  float voltage_in = (conversion_result*voltage_reference)/1024;
  return voltage_in*vol_to_temp;
}

void start_pwm_duty(uint8_t duty)
{
  OCR0A=duty;
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

void pwm_control()
{
  current_temp = do_adc_convertion();
  int16_t err_value = pid_Controller(set_temp, current_temp, REGULATOR_DATA);
  if (err_value<=0) start_pwm_duty(0x00);
  else start_pwm_duty((err_value*256)/1024); //conversion to uint8_t
  if (is_heated_first_time)
  {
    if (current_temp==set_temp);
    is_heated_first_time=0;
    SOUND_ON;
    _delay_ms(200);
    SOUND_OFF;
    _delay_ms(100);
    SOUND_ON;
    _delay_ms(200);
    SOUND_OFF;
  }
}
