/*
  gpio_speedTest.ino arduino gpio speed test example for stm32
  nucleo 401 84M run in sram:
	read:
		8403khz 119us  mode 1/3
    6493khz 154us  mode 4
		7633khz 131us  mode 2
		2793khz 358us  mode 0
	write:
		7936khz 126us  mode 1/3
    5076khz 197us  mode 4
		3663khz 273us  mode 2
		1757khz 569us  mode 0
  by  huawei <huaweiwx@sina.com> 2019.9.10
*/
#include "util/bitband.h"

#define GPIOMODE 2  /* select mode 0/1/2 */

#if GPIOMODE == 4
#define led PAout(5)
#define inp PAin(0)
#elif GPIOMODE == 3
GPIOPIN led(LED_BUILTIN);
GPIOPIN inp(PA0);
#elif GPIOMODE == 2
BB_PIN led(LED_BUILTIN);
BB_PIN inp(PA0);
#elif GPIOMODE == 1        /* cplus mode */
ARDUINOPIN_TypeDef led = LED_BUILTIN;   /* led is __ConstPin class type */
ARDUINOPIN_TypeDef inp = PA0;
#else                      /* c mode */
uint32_t led = LED_BUILTIN;
uint32_t inp = PA0;
#endif

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);  /*set param: 115200bps 8N1 (default 9600bps 8N1) */
  delay(2000);
#if GPIOMODE == 4
  pinMode(LED_BUILTIN, OUTPUT);
#elif GPIOMODE >= 2
  led.mode(OUTPUT);
#else
  pinMode(led, OUTPUT);
#endif

  uint32_t timers = getTimers(1000, 0);
  Serial.println("\nRead:");
  Serial.print(1000000 / timers);
  Serial.print(" khz  Timer elapsed:");
  Serial.print(timers);
  Serial.println(" us\n");

  timers = getTimers(1000, 1);
  Serial.println("Write:");
  Serial.print(1000000 / timers);
  Serial.print(" khz  Timer elapsed:");
  Serial.print(timers);
  Serial.println(" us\n");
}

//Measuring toggle frequency with an oscilloscope:
void loop() {
#if  GPIOMODE == 4
  digitalToggle(LED_BUILTIN);
#elif   GPIOMODE >= 2
  led.toggle();
#else
  digitalToggle(led);
#endif
  //  delay(500);
}


uint32_t getTimers(uint32_t count, uint8_t op) {
  uint32_t timeBegan, loopTimeElapsed, timeElapsed, i;
  i = 0;
  uint32_t tmp = 0;
  timeBegan = micros();
  while (i < count) {
    i++;
  }
  loopTimeElapsed = (micros() - timeBegan); // Time taken to do nothing but increment a variable
  i = 0;
  if (op) {  //write
    timeBegan = micros();
    while (i < count) {
#if   GPIOMODE == 4
      led = 1;
      led = 0;
#elif   GPIOMODE >= 2
      led.write(HIGH);
      led.write(LOW);
#else
      digitalWrite(led, HIGH);
      digitalWrite(led, LOW);
#endif
      i++;
    }
    timeElapsed = (micros() - timeBegan - loopTimeElapsed); // Time taken to read a pin
  } else { //read
    timeBegan = micros();
    while (i < count) {
#if   GPIOMODE == 4
      tmp += inp;
#elif   GPIOMODE >= 2
      tmp += inp.read();
#else
      tmp += digitalRead(inp);
#endif
      i++;
    }
    timeElapsed = (micros() - timeBegan - loopTimeElapsed); // Time taken to read a pin
  }
  return timeElapsed;
}
