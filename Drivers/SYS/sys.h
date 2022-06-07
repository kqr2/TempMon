/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYS__H
#define __SYS__H

#include "SparkFun_RV8803.h"
#include "tmp102.h"

typedef struct {
  int      ntmp;
  tmp102_t tmp[TMP102_MAX_SENSORS];
  RV8803 rtc;
} sys_t;

void sys_init(sys_t *sys);
bool sys_tmp_rescan(sys_t *sys);

#endif /* __SYS__H */   
