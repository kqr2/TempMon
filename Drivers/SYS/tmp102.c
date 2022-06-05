#include <tmp102.h>

static uint16_t tmp102_readreg16(tmp102_t *tmp, uint8_t Reg) {
  uint8_t buf[2];
  buf[0] = Reg;
  I2Cx_Master_Transmit(tmp->addr, buf, 1);
  I2Cx_Master_Receive(tmp->addr, buf, 2);
  return (((uint16_t)buf[0]) << 8) |  buf[1];
}

uint16_t tmp102_read_temp(tmp102_t *tmp) {
  return tmp102_readreg16(tmp, 0) >> 4;
}

bool tmp102_detect(tmp102_t *tmp) {
  tmp->detect = (I2Cx_Detect(tmp->addr) == HAL_OK);
  return tmp->detect;
}

void tmp102_init(tmp102_t *tmp, uint8_t Addr) {
  tmp->addr = Addr << 1;
  tmp102_detect(tmp);
}

