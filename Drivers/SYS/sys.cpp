#include "sys.h"

#include "stm32f429i_discovery.h"

void sys_init(sys_t *sys) {
  I2Cx_Init();
  
  sys->ntmp = 0;
  for (int i=0; i<TMP102_MAX_SENSORS; i++) {
    tmp102_t *tmp = &sys->tmp[i];
    tmp102_init(tmp, TMP102_BASE_ADDR + i);
    if (tmp->detect) {
      sys->ntmp++;
    }
  }
}

bool sys_tmp_rescan(sys_t *sys) {
  bool change = false;

  sys->ntmp = 0;
  for (int i=0; i<TMP102_MAX_SENSORS; i++) {
    tmp102_t *tmp = &sys->tmp[i];
    if (tmp->detect != tmp102_detect(tmp)) {
      change = true;
    }
    if (tmp->detect) {
      sys->ntmp++;
    }
  }
  
  return change;
}
