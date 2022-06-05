#include "sys.h"

#include "stm32f429i_discovery.h"

void sys_init(sys_t *sys) {
  I2Cx_Init();
  for (int i=0; i<TMP102_MAX_SENSORS; i++) {
    tmp102_init(&sys->tmp[i], TMP102_BASE_ADDR + i);
  }
}

bool sys_tmp_rescan(sys_t *sys) {
  bool change = false;
  for (int i=0; i<TMP102_MAX_SENSORS; i++) {
    if (sys->tmp[i].detect != tmp102_detect(&sys->tmp[i]))
      change = true;
  }
  return change;
}
