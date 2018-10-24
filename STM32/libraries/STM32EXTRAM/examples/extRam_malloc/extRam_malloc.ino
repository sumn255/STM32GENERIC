/**  extRam_malloc.ino   sram(sdram) r/w pass whit following  boards:
      DISCOVERY_F746NG
      WaveShare F746I)(8M SDRAM) Checksum 4293918720
      DISCOVERY_F429ZI(8M SDRAM) Checksum 4293918720
      F429IG_CORE(8M SDRAM)      Checksum 4293918720
      ARMFLY_F407ZG (1M SRAM)    Checksum 4294836224
      HASEE_III_F103ZE
      ILLUMINATI_F407ZG
      REDBULL_V2_F103ZE
   Allocate ext ram, and write to Serial the results
*/

#include <STM32ExtRAM.h>
STM32EXTRAM& extRAM = STM32EXTRAM::getInstance(); /* One instance only*/

#define led  LED_BUILTIN


void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  delay(200);

  uint32_t size = 960 * 1024;
  uint8_t* ptr = extRAM.malloc<uint8_t>(size);
  if (ptr) {
    Serial.print("malloc ");
    Serial.print(size);
    Serial.print(" ok!");
  } else {
    Serial.print("Error: cannot malloc ");
    Serial.print(size);
    Serial.print(" bytes!");
    return;
  }

  Serial.print("\nperused: ");
  Serial.println(extRAM.perused());

  Serial.print("test ptr baes adr read:");
  Serial.println((uint32_t)ptr, HEX);

  for (uint32_t i = 0; i < 16; i++) {
    ptr[i] = i;
  }

  for (uint32_t i = 0; i < 16; i++) {
    Serial.print(ptr[i], HEX);
    Serial.print(" ");
  }
  Serial.println("\nok");

  extRAM.free(ptr);
  Serial.print("\nperused: ");
  Serial.println(extRAM.perused());

  size = 1024 * 1024; /*!for 51216 is error!*/
  ptr = extRAM.malloc<uint8_t>(size);
  if (ptr) {
    Serial.print("malloc ");
    Serial.print(size);
    Serial.print(" ok!");
  } else {
    Serial.print("Error: cannot malloc ");
    Serial.print(size);
    Serial.print(" bytes!");
    return;
  }
}

void loop() {
  delay(500);
  digitalToggle(led);
}

