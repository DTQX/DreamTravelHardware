// #include <inttypes.h>

#ifdef __cplusplus 
extern "C" {
#endif
    // i2c 多字节写
    uint8_t i2c_write(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
    // i2c 多字节读
    int8_t readBytes_c(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
    // 毫秒延时
    void delay_ms(unsigned long ms);
    // 获取当前时间戳
    int get_ms(unsigned long *timestamp);
    
#ifdef __cplusplus 
}
#endif