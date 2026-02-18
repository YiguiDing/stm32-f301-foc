#ifndef __SWO_H__
#define __SWO_H__

#include "stm32f30x.h"
#include <stdarg.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C"
{
#endif

	void swo_init(uint32_t portMask, uint32_t cpuCoreFreqHz, uint32_t baudrate);
	void swo_sendchar(size_t const port, uint8_t ch);
	void swo_printf(const char *format, ...);

#ifdef __cplusplus
}
#endif

#endif
