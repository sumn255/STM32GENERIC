#ifndef STM32_PIN_LIST_H
#define STM32_PIN_LIST_H

#include "variant.h"
#include "stm32_HAL/stm32XXxx_ll_gpio.h"

#ifdef VARIANT_PIN_LIST
# define PIN_LIST VARIANT_PIN_LIST
#else
# define PIN_LIST CHIP_PIN_LIST
#endif

#ifdef __cplusplus

class __ConstPin {
  public:
    constexpr __ConstPin(const GPIO_TypeDef* port, const int pinMask, const int val): port(port), pinMask(pinMask), val(val) {};
    constexpr operator int() const {
      return val;
    }
    const GPIO_TypeDef* port;
    const int pinMask;
    const int val;
};

#define PIN(a, b) __P##a##b
enum {
  PIN_LIST
  NUM_DIGITAL_PINS,
};
#undef PIN

#ifndef STM32H7
#define PIN(a, b) P##a##b(GPIO##a,LL_GPIO_PIN_##b,__P##a##b)
#else
#define PIN(a, b) P##a##b(GPIO##a,GPIO_PIN_##b,__P##a##b)
#endif
constexpr __ConstPin PIN_LIST __IGNORE(0, 0, -1);
#undef PIN

#define ARDUINOPIN_TypeDef __ConstPin
#else

#define PIN(a, b) P ## a ## b
enum {
  PIN_LIST
  NUM_DIGITAL_PINS,
};
#undef PIN

#endif  //__cplusplus

#endif  //STM32_PIN_LIST_H
