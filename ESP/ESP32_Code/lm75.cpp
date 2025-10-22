#include <stdio.h>
#include <Wire.h>

/* PRIVATE LIBRARIES */
#include "lm75.h"

void lm75_init(lm75_t *dev, uint8_t addr, bool (*i2c_write_read)(uint8_t, uint8_t, uint8_t*, size_t)) {
    dev->i2c_addr = addr;
    dev->i2c_write_read = i2c_write_read;
}

bool lm75_read_temp_c(const lm75_t *dev, float *temp_out) {
    uint8_t data[2];
    if (!dev->i2c_write_read(dev->i2c_addr, LM75_TEMP_REG, data, 2)) return false;

    int16_t raw = ((int16_t)data[0] << 8) | data[1];
    raw >>= 5;                      // 11 significant bits
    if (raw & 0x400) raw |= 0xF800; // check for positive or neegative temperature value
    *temp_out = raw * 0.125f;       // 0.125 °C/LSB

    return true;
}

extern "C" bool lm75_i2c_write_read(uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
    for (size_t i = 0; i < len; i++) data[i] = Wire.read();
    
    return true;
}