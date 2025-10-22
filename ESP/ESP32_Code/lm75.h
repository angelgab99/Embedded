#ifndef LM75_H
#define LM75_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif


/* DEFINES */
#define LM75_SLAVE_ADDR	0x48
#define	LM75_TEMP_REG	0x00
#define	LM75_CONF_REG	0x01
#define	LM75_THYST_REG	0x02
#define	LM75_TOS_REG	0x03

/* HANDLER TYPE DEFINITIONS */
typedef struct {
  uint8_t i2c_addr; // Slave Address 
  bool (*i2c_write_read)(uint8_t addr, uint8_t reg, uint8_t *data, size_t len); // pointer to function, returns a bool data type
} lm75_t;

/* FUNCTION PROTOTYPES */
void lm75_init(lm75_t *dev, uint8_t addr, bool (*i2c_write_read)(uint8_t, uint8_t, uint8_t*, size_t));
bool lm75_read_temp_c(const lm75_t *dev, float *temp_out); 

#ifdef __cplusplus
}
#endif

#endif // LM75_H