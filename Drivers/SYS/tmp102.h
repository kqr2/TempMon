/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TMP102_H
#define __TMP102_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdbool.h>
#include "stm32f429i_discovery.h"

#define TMP102_MAX_SENSORS	4
#define TMP102_BASE_ADDR	(0x48)

typedef struct {
  uint8_t addr;
  bool detect;
} tmp102_t;

uint16_t tmp102_read_temp(tmp102_t *tmp);
bool tmp102_detect(tmp102_t *tmp);
void tmp102_init(tmp102_t *tmp, uint8_t Addr);

#ifdef __cplusplus
}
#endif

#endif /* __TMP102_H */
