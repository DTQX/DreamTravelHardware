#include <inttypes.h>
// #include "I2Cdev.h"

// #ifdef __cplusplus 
// extern "C" {
// #endif
    // i2c 初始化
    int8_t i2c_init_my();
    // i2c 多字节写
    uint8_t i2c_write_bytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
    // i2c 多字节读
    int8_t i2c_read_bytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
    // 毫秒延时
    // void delay_ms(unsigned long ms);
    // 获取当前时间戳
    int get_ms(unsigned long *timestamp);
    
// #ifdef __cplusplus 
// }
// #endif
// #define i2c_write_bytes      I2Cdev::writeBytes
// #define i2c_read_bytes      I2Cdev::readBytes
#define delay_ms        delay
#define log_i(...)     do {} while (0)
#define log_e(...)     do {} while (0)