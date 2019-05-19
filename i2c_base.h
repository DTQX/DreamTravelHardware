#include <inttypes.h>

extern int8_t writeBytes_c(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
extern int8_t readBytes_c(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
extern void delay_ms(unsigned long ms);
extern int get_ms(unsigned long *timestamp);
#define log_i(...)     do {} while (0)
#define log_e(...)     do {} while (0)