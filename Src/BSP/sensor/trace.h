#ifndef __TRACE_H
#define __TRACE_H

#include "stm32f10x.h"
#include "config.h"

void trace_init(void);
void trace_read(u16 *buf);

#endif
