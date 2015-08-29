// main.c

#include <avr/io.h>
#include <util/delay.h> 
 
int main (void)
{
  // set PB3 to be output
	DDRB = 0b00001000;
  while (1) {    
    PORTB = 0b00001000; 
    _delay_ms(200);
    PORTB = 0b00000000;
    _delay_ms(200);
  }
  return 1;
}