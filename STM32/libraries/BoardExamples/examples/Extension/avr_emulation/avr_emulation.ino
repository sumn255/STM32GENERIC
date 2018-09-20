/*
  avr_emulation.ino: avr registers PORTx PINx DDRx demo emulation demo
  
  huaweiwx@sina.com 2018.8.10
*/

#if USE_AVREMULATION > 0

// the setup function runs once when you press reset or power the board
void setup() {
  DDRF |= BIT6;   //set PF6  OUTPUT
}

// the loop function runs over and over again forever
void loop() {
#if 0
  PORTF ^= BIT6;   //PF6 toggle
  delay(1000);      //wait for a second
#else
  PORTF |= BIT6;   //PF6 high
  delay(1000);      //wait for a second
  PORTF &= ~BIT6;  //PF6 low
  delay(1000);      //wait for a second
#endif
}

#else
#error "avr emulation unable"
#endif
