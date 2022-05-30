#include "sys_tmp102.h"

static uint16_t sys_tmp102_readreg16(uint8_t Addr, uint8_t Reg) {
  uint8_t buf[2];
  buf[0] = Reg;
  I2Cx_Master_Transmit(Addr, buf, 1);
  I2Cx_Master_Receive(Addr, buf, 2);
  return (((uint16_t)buf[0]) << 8) |  buf[1];
}

uint16_t sys_tmp102_read_temp(uint8_t Addr) {
  return sys_tmp102_readreg16(Addr, 0) >> 4;
}

void sys_tmp102_init(void) {
  I2Cx_Init();
}

