#ifndef __SERIAL_H__
#define __SERIAL_H__

#include "stm32f30x.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void serial_init();
    uint8_t *serial_next_rx_buffer();
    uint8_t *serial_next_tx_buffer();
    void serial_send(uint8_t *data, uint16_t len);
    __attribute__((weak)) void serial_on_receive(uint8_t *data, uint16_t len);
    void serial_printf(const char *format, ...);
    void serial_test_printf();

#ifdef __cplusplus
}
#endif

#endif