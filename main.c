// main.c

// #include <avr/io.h>
// #include <util/delay.h>
//
// int main (void)
// {
//   // set PB3 to be output
//   DDRB = 0b00001000;
//   while (1) {
//     PORTB = 0b00001000;
//     _delay_ms(500);
//     PORTB = 0b00000000;
//     _delay_ms(500);
//   }
//   return 1;
// }

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define CHMAX 1 // maximum number of PWM channels
#define PWMDEFAULT 0x00 // default PWM value at start up for all channels

#define EYE_CLEAR (pinlevelB &= ~(1 << EYE)) // map EYE to PB0
// #define GREEN_CLEAR (pinlevelB &= ~(1 << GREEN)) // map GREEN to PB1
// #define BLUE_CLEAR (pinlevelB &= ~(1 << BLUE)) // map BLUE to PB2

//! Set bits corresponding to pin usage above
#define PORTB_MASK  (1 << PB0)|(1 << PB1)|(1 << PB2)

#define set(x) |= (1<<x) 
#define clr(x) &=~(1<<x) 
#define inv(x) ^=(1<<x)

#define EYE PB0
#define LED_DDR DDRB

void delay_ms(uint16_t ms);
void init();

unsigned char compare[CHMAX];
volatile unsigned char compbuff[CHMAX];
int delayVal = 100;

uint8_t val = 0;

int main() {
  init();
  
  int step = 1;
  
  for(;;) {
    val += step;
    delayVal = val == 254 ? 10000 : 100;
    delayVal = val == 0 ? 4000 : delayVal; 
    compbuff[0] = val;
    delay_ms(delayVal);
  }
}


void delay_ms(uint16_t ms) {
  while (ms) {
    _delay_ms(1);
    ms--;
  }
}

void init(void) {
  // set the direction of the ports
  LED_DDR set(EYE);
  
  unsigned char i, pwm;

  CLKPR = (1 << CLKPCE);        // enable clock prescaler update
  CLKPR = 0;                    // set clock to maximum (= crystal)

  pwm = PWMDEFAULT;

  // initialise all channels
  for(i=0 ; i<CHMAX ; i++) {
    compare[i] = pwm;           // set default PWM values
    compbuff[i] = pwm;          // set default PWM values
  }

  TIFR = (1 << TOV0);           // clear interrupt flag
  TIMSK = (1 << TOIE0);         // enable overflow interrupt
  TCCR0B = (1 << CS00);         // start timer, no prescale

  sei();
}


ISR (TIM0_OVF_vect) {
  static unsigned char pinlevelB=PORTB_MASK;
  static unsigned char softcount=0xFF;

  PORTB = pinlevelB;            // update outputs
  
  if(++softcount == 0){         // increment modulo 256 counter and update
                                // the compare values only when counter = 0.
    compare[0] = compbuff[0];   // verbose code for speed

    pinlevelB = PORTB_MASK;     // set all port pins high
  }
  // clear port pin on compare match (executed on next interrupt)
  if(compare[0] == softcount) EYE_CLEAR;

}
