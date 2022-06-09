/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SYS__H
#define __SYS__H

#include "SparkFun_RV8803.h"
#include "tmp102.h"

#define SYS_TIMER_TICK		(2)	// 2 milliseconds
#define SYS_TIMER_MIN		(1000)	// 1 second
#define SYS_TIMER_DEFAULT	SYS_TIMER_MIN


typedef struct {
  int	   timer_ticks;
  int      ntmp;
  tmp102_t tmp[TMP102_MAX_SENSORS];
  RV8803 rtc;
} sys_t;

#define SYS_TIMER_INTERVAL(sys)	(sys.timer_ticks * SYS_TIMER_TICK)

void sys_init(sys_t *sys);
bool sys_tmp_rescan(sys_t *sys);
void sys_set_timer_interval(sys_t *sys, int msec);

#endif /* __SYS__H */   
