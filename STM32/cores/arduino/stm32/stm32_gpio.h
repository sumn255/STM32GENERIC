/*
  Copyright (c) 2017 Daniel Fekete
  
  add avr emulation and gpio fastIO 2017 huaweiwx@sina.com
 
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#ifndef STM32_GPIO_H
#define STM32_GPIO_H

#include "stm32_def.h"
#include "stm32_pin_list.h"
#include "variant.h"
#include "stm32_HAL/stm32XXxx_ll_gpio.h"

/*compatable with avr*/
#ifdef GPIOA
# define PORTA      GPIOA->ODR
# define PINA       GPIOA->IDR
#endif
#ifdef GPIOB
# define PORTB      GPIOB->ODR
# define PINB       GPIOB->IDR
#endif
#ifdef GPIOC
# define PORTC      GPIOC->ODR
# define PINC       GPIOC->IDR
#endif
#ifdef GPIOD
# define PORTD      GPIOD->ODR
# define PIND       GPIOD->IDR
#endif
#ifdef GPIOE
# define PORTE      GPIOE->ODR
# define PINE       GPIOE->IDR
#endif
#ifdef GPIOF
# define PORTF      GPIOF->ODR
# define PINF       GPIOF->IDR
#endif
#ifdef GPIOG
# define PORTG      GPIOG->ODR
# define PING       GPIOG->IDR
#endif
#ifdef GPIOH
# define PORTH      GPIOH->ODR
# define PINH       GPIOH->IDR
#endif
#ifdef GPIOI
# define PORTI      GPIOI->ODR
# define PINI       GPIOI->IDR
#endif

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3

#define ANALOG 0x04

#define OUTPUT_PP 0x1    /* as OUTPUT add by huaweiwx@sina.com 2017.6.9   */
#define OUTPUT_OD 0x11   /*!< Output Open Drain Mode  add by huaweiwx@sina.com 2017.6.9   */


#ifndef GPIO_SPEED_FREQ_VERY_HIGH
    #define GPIO_SPEED_FREQ_VERY_HIGH GPIO_SPEED_FREQ_HIGH
#endif

#define RwReg  uint32_t

#ifdef __cplusplus
extern "C"{
#endif

typedef struct {
    GPIO_TypeDef *port;
    uint32_t pinMask;
} stm32_port_pin_type;

typedef uint32_t (*stm32_clock_freq_func)();

typedef struct {
    void *instance;
    stm32_clock_freq_func clock_freq_func;
} stm32_clock_freq_list_type;

extern const stm32_port_pin_type variant_pin_list[NUM_DIGITAL_PINS];

inline GPIO_TypeDef* pinToPort(uint8_t pin){return variant_pin_list[pin].port;}  /* equal_to digitalPinToPort(pin)*/
inline uint32_t pinToBitMask(uint8_t pin){return variant_pin_list[pin].pinMask;} /* equal_to digitalPinToBitMask(pin)*/

/**
 * Start clock for the fedined port
 */
void stm32GpioClockEnable(GPIO_TypeDef *port);

/**
 * If PWM is used at least once, this method is set to the PWM disable function in stm32_PWM.c
 */
typedef void (*stm32_pwm_disable_callback_func)(GPIO_TypeDef *port, uint32_t pin);
extern stm32_pwm_disable_callback_func stm32_pwm_disable_callback;

extern void attachInterrupt(uint8_t, void (*)(void), int mode);
extern void detachInterrupt(uint8_t);

inline void digitalWriteHigh(uint8_t pin) {
//    if (pin >= sizeof(variant_pin_list) / sizeof(variant_pin_list[0])) {
//        return;
//    }
    stm32_port_pin_type port_pin = variant_pin_list[pin];
    HAL_GPIO_WritePin(port_pin.port, port_pin.pinMask, GPIO_PIN_SET);
}
inline void digitalWriteLow(uint8_t pin) {
//    if (pin >= sizeof(variant_pin_list) / sizeof(variant_pin_list[0])) {
//        return;
//    }
    stm32_port_pin_type port_pin = variant_pin_list[pin];
    HAL_GPIO_WritePin(port_pin.port, port_pin.pinMask, GPIO_PIN_RESET);
}

inline void digitalWrite(uint8_t pin, uint8_t value) {
//    if (pin >= sizeof(variant_pin_list) / sizeof(variant_pin_list[0])) {
//        return;
//    }
    stm32_port_pin_type port_pin = variant_pin_list[pin];
    HAL_GPIO_WritePin(port_pin.port, port_pin.pinMask, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
inline int digitalRead(uint8_t pin) {
//    if (pin >= sizeof(variant_pin_list) / sizeof(variant_pin_list[0])) {
//        return 0;
//    }
    
    stm32_port_pin_type port_pin = variant_pin_list[pin];
    return HAL_GPIO_ReadPin(port_pin.port, port_pin.pinMask);
}

inline void digitalToggle(uint8_t pin) {
    stm32_port_pin_type port_pin = variant_pin_list[pin];
    HAL_GPIO_TogglePin(port_pin.port, port_pin.pinMask);
}

#ifdef __cplusplus
}
#endif


#define PIN(a, b)   b
static const uint8_t variant_gpiopin_pos_static[] = {
   PIN_LIST
};  
#undef PIN

#ifndef STM32H7
#define PIN(a, b) { GPIO##a , LL_GPIO_PIN_##b }
static const stm32_port_pin_type variant_pin_list_ll_static[] = {
  PIN_LIST
};
#undef PIN
#endif

#ifdef __cplusplus

inline static void digitalWriteHigh(__ConstPin cpin) {
     LL_GPIO_SetOutputPin((GPIO_TypeDef*)cpin.port,cpin.pinMask);
}
inline static void digitalWriteLow(__ConstPin cpin) {
    LL_GPIO_ResetOutputPin((GPIO_TypeDef*)cpin.port,cpin.pinMask);
}

template<typename T> 
inline static void digitalWrite(__ConstPin cpin, T value) {
    if (value) {
        LL_GPIO_SetOutputPin((GPIO_TypeDef*)cpin.port,cpin.pinMask);
    } else {
        LL_GPIO_ResetOutputPin((GPIO_TypeDef*)cpin.port,cpin.pinMask);
    }
}

template<typename T = bool> 
inline static T digitalRead(__ConstPin cpin) {
	return LL_GPIO_IsInputPinSet((GPIO_TypeDef*)cpin.port,cpin.pinMask);
}


extern "C" void pinModeLL(GPIO_TypeDef *port, uint32_t ll_pin, uint8_t mode);
inline static void pinMode(__ConstPin cpin, uint8_t mode) {
    pinModeLL((GPIO_TypeDef *)cpin.port, cpin.pinMask, mode);
}

inline static void digitalToggle(__ConstPin cpin) {
    LL_GPIO_TogglePin((GPIO_TypeDef*)cpin.port, cpin.pinMask);
}

template<class T> 
inline  void operator << (__ConstPin co, T value){
  digitalWrite(co, (uint8_t)value);
}

inline void operator << (__ConstPin co, __ConstPin ci){
  digitalWrite(co, (uint8_t)digitalRead(ci));
}

template<class T> 
inline void operator >> (__ConstPin ci, T &value) {
  value = digitalRead(ci);
}

inline void operator >> (__ConstPin ci, __ConstPin co){
  digitalWrite(co, (uint8_t)digitalRead(ci));
}


#if USE_AVREMULATION > 0

class DDRemulation
{
  public:
    DDRemulation(GPIO_TypeDef *port, uint16_t mask=0xff): port(port),mask(mask){}
   	GPIO_TypeDef *port;
	uint16_t mask;
	
    void mode(int pins, uint8_t mode) {
      if (this->inited == 0) {
        stm32GpioClockEnable(port);
        this->inited = 1;
      }
      gpio_init(pins, mode);
    }

    inline operator int (){
      return this->saveddr;
    }

    inline DDRemulation & operator = (int pins) __attribute__((always_inline)) {
      this->gpio_init(pins,  OUTPUT);
      this->gpio_init(~pins,  INPUT); 
      return *this;
    }

    inline DDRemulation & operator |= (int pins) __attribute__((always_inline)) {
      this->gpio_init(pins, OUTPUT);
      return *this;
    }
	
    inline DDRemulation & operator &= (int pins) __attribute__((always_inline)) {
      this->gpio_init(~pins, INPUT);
      return *this;
    }

  private:
    uint8_t inited = 0;
    uint16_t saveddr = 0;
	
    void gpio_init(int pins, uint8_t mode) {
	  GPIO_InitTypeDef GPIO_InitStruct;
	  
      switch ( mode ) {
        case INPUT:
          GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
          GPIO_InitStruct.Pull =  GPIO_NOPULL;
          break;

        case INPUT_PULLUP:
          GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
          GPIO_InitStruct.Pull =  GPIO_PULLUP;
          break;

        case INPUT_PULLDOWN:
          GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
          GPIO_InitStruct.Pull = GPIO_PULLDOWN;
          break;

        case ANALOG:
          GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
          GPIO_InitStruct.Pull = GPIO_NOPULL;
          break;

        case OUTPUT:
          GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
          GPIO_InitStruct.Pull =  GPIO_NOPULL;
          break;

        case OUTPUT_OD:
          GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
          GPIO_InitStruct.Pull = GPIO_NOPULL;
          break;

        default:
          assert_param(0);
          return;
          break;
      }
	  GPIO_InitStruct.Pin = pins & mask;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
      HAL_GPIO_Init(port, &GPIO_InitStruct);

	  if((mode == OUTPUT)||(mode == OUTPUT_OD)) saveddr |= GPIO_InitStruct.Pin;
	  else	 saveddr &= ~GPIO_InitStruct.Pin;
	  
	  saveddr &= mask;
    }
};

#ifdef GPIOA
extern DDRemulation DDRA;
#endif
#ifdef GPIOB
extern DDRemulation DDRB;
#endif
#ifdef GPIOC
extern DDRemulation DDRC;
#endif
#ifdef GPIOD
extern DDRemulation DDRD;
#endif
#ifdef GPIOE
extern DDRemulation DDRE;
#endif
#ifdef GPIOF
extern DDRemulation DDRF;
#endif
#ifdef GPIOG
extern DDRemulation DDRG;
#endif
#ifdef GPIOH
extern DDRemulation DDRH;
#endif
#ifdef GPIOI
extern DDRemulation DDRI;
#endif
#ifdef GPIOJ
extern DDRemulation DDRJ;
#endif
#ifdef GPIOK
extern DDRemulation DDRK;
#endif

#endif //USE_AVREMULATION > 0

/*gpio low layer interface class*/
class LL_PIN {
  public:
    LL_PIN(__ConstPin cpin): cpin(cpin) {}
    __ConstPin cpin;

    template<typename T>
    inline LL_PIN & operator = (T value) {
      this->write(value);
      return *this;
    }

    LL_PIN& operator = (LL_PIN& rhs) {
      this->write(rhs.read());
      return *this;
    }

    template<typename T>
    inline LL_PIN & operator << (T value) {
      this->write(value);
      return *this;
    }

    inline LL_PIN & operator << (bool value) {
      this->write(value);
      return *this;
    }

    template<class T>
    inline LL_PIN & operator >> (T &value) {
      value = this->read();
      return *this;
    }

    template<typename T>
    inline void write(T value) {
       digitalWrite(cpin, value);
    }

    inline __attribute__((always_inline))
    void high() {
      digitalWriteHigh(cpin);
    }

    inline __attribute__((always_inline))
    void low() {
      digitalWriteLow(cpin);
    }

    template<typename T = bool>
    inline operator T () const {
      return this->read();
    }

    template<typename T = bool>
    inline T read() const {
      return digitalRead(cpin);
    }

    inline void operator  !() __attribute__((always_inline)) {
      this->toggle();
    }

    /*----- comptabled with DigitalPin ----------*/
    inline __attribute__((always_inline))
    void toggle() {
      digitalToggle(cpin);
    }

    template<typename T>
    inline  void config(uint8_t mode, T level) {  /*compatale with digitalPin*/
      this->mode(mode);
      this->write(level);
    }

    inline void mode(uint8_t mode) {
      pinMode(cpin, mode);
    }

    inline void attach(voidFuncPtr callback, uint8_t mode) {
      attachInterrupt(cpin.val, callback, mode);
    }

    inline void detach(void) {
      detachInterrupt(cpin.val);
    }

};

class InputPin : public LL_PIN{
	public: 
	  InputPin(__ConstPin cpin, bool initial_value = 1): LL_PIN(cpin){
			digitalWrite(cpin,initial_value);
			pinMode(cpin,INPUT);
	  }
};

class OutputPin : public LL_PIN{
	public: 
	  OutputPin(__ConstPin cpin, bool initial_value = 0): LL_PIN(cpin) {
			digitalWrite(cpin,initial_value);
			pinMode(cpin,OUTPUT);
		}
};

#define GPIOPIN  LL_PIN

#endif //__cplusplus

#if USE_BITBAND >0
#include "util/bitband.h"
#endif 

#endif //STM32_GPIO_H
