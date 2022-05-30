/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYS_TEMP102_H
#define __SYS_TEMP102_H

#include "stm32f429i_discovery.h"

uint16_t sys_tmp102_read_temp(uint8_t Addr);
void sys_tmp102_init(void);

#endif /* __SYS_TEMP102_H */
