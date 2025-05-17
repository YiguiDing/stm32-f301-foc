#ifndef __I2C_H__
#define __I2C_H__

#include "stdint.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void i2c_init();
    uint8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
    uint8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif