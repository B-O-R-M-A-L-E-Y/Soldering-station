#include "encoder.h"
#include <avr/io.h>
#include <stdint.h>


#define GRAY_CODE_LEFT 0b11100001
#define GRAY_CODE_RIGHT 0b11010010

volatile static uint8_t ENC_SPIN = 0x00;

void init_encoder()
{
  //direction register
  CLEAR_BIT(ENC_DDR,ENC_PIN_FIRST);
  CLEAR_BIT(ENC_DDR,ENC_PIN_SECOND);
  //disable pull-up resistor (Hi-Z state)
  CLEAR_BIT(ENC_PORT,ENC_PIN_FIRST);
  CLEAR_BIT(ENC_PORT,ENC_PIN_SECOND);
}

void check_encoder_state()
{
  volatile static uint8_t pins_state;
  uint8_t current_state = 0;
  //if on pin detected 1, current encoder state write on 2 last bits
  if (IS_TRUE_BIT(ENC_PIN,ENC_PIN_FIRST)) SET_BIT(current_state,1);
  if (IS_TRUE_BIT(ENC_PIN,ENC_PIN_SECOND)) SET_BIT(current_state,2);

  //return if last 2 bits are same as in current_state
  //(0b00000011 is to make bitmask)
  if (current_state == (pins_state & 0b00000011)) return;

  //write on last 2 bits new bits state
  pins_state = (pins_state<<2)|current_state;
  //compare with Gray code for each turn
  if (pins_state==GRAY_CODE_LEFT) {
    ENC_SPIN = SPIN_LEFT;
    return;
  }
  if (pins_state==GRAY_CODE_RIGHT) {
    ENC_SPIN = SPIN_RIGHT;
    return;
  }
}

uint8_t get_encoder_state()
{
  uint8_t tmp = ENC_SPIN;
  ENC_SPIN = SPIN_NO;
  return tmp;
}
