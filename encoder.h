#ifndef ENCODER_H
#define ENCODER_H

#include <avr/io.h>
#include "macros.h"

//register for Hi-Z or pull-up resister
#define ENC_PORT PORTC
//direction register
#define ENC_DDR DDRC
//state register
#define ENC_PIN PINC
//numbers of pin in register
#define ENC_PIN_FIRST PINC2
#define ENC_PIN_SECOND PINC3

#define SPIN_NO 0x00
#define SPIN_LEFT 0x01
#define SPIN_RIGHT 0xFF

// Will initialize connection on pin that was declared in #define
void init_encoder();
// Check pin state and write SPIN state (look #define) in static variable
void check_encoder_state();
// Returns static variable and then makes it SPIN_NO
uint8_t get_encoder_state();

#endif
