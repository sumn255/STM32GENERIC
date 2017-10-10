/* i2ceepromconfig.h  iic_eeprom config define file ,arduino have not autoload! */
#ifndef __I2CEEPROMCONFIG_H__
#define __I2CEEPROMCONFIG_H__
#include <Arduino.h>

#define AT24CXX_TYPE AT24C02
#define AT24CXX_DEV	0	/*unused i2cport ,SOFT IIC*/
#define AT24CXX_SDA PC11
#define AT24CXX_SCL PC12
#define AT24CXX_A2A1A0    0

#endif   //__BSP_I2CEEPROM_H__