#include "my_i2cdev.h"

#define SCL_PIN 4
#define SCL_PORT PORTE
#define SDA_PIN 5
#define SDA_PORT PORTE
// #define I2C_NOINTERRUPT 1
#define I2C_FASTMODE 1

#include "SoftI2CMaster.h"


// #ifdef __cplusplus 
// extern "C" {
// #endif

// void delay_ms(unsigned long ms){
//     return delay(ms);
// }

int get_ms(unsigned long *timestamp){
    *timestamp = 0;
    return 0;
}

// i2c 多字节写
uint8_t i2c_write_bytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data){
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.println("into i2c_write_bytes");
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") writing ");
        Serial.print(length, DEC);
        Serial.print(" bytes to 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif
    // 开始新的i2c连接，发送从机地址
    if (!i2c_start(devAddr <<1 | I2C_WRITE)){
        return -1;
    }
    // 发送读取地址
    if (!i2c_write(regAddr)){
        return -2;
    }
    // 读取数据
    for (uint8_t i = 0; i < length; i++) {
        #ifdef I2CDEV_SERIAL_DEBUG
            Serial.print(data[i], HEX);
            if (i + 1 < length) Serial.print(" ");
        #endif
        i2c_write(data[i]);
    }
    // 发送停止信号
    i2c_stop();
    return 0;
}
// i2c 多字节读
int8_t i2c_read_bytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data){
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.println("into i2c_read_bytes");
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") reading ");
        Serial.print(length, DEC);
        Serial.print(" bytes from 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif
    // 开始新的i2c连接，发送从机地址
    if (!i2c_start(devAddr <<1 | I2C_WRITE)){
        return -1;
    }
    // 发送读取地址
    i2c_write(regAddr);
    // 发送停止信号
    i2c_stop();
    // 开始新的i2c连接，读取数据
    if (!i2c_rep_start((devAddr <<1 | I2C_READ))){
        return -2;
    }
    uint8_t cnt;
    for (cnt=0; cnt < length - 1; cnt++){
        // 读取数据
        data[cnt] = i2c_read(false);
        #ifdef I2CDEV_SERIAL_DEBUG
            Serial.print(data[cnt], HEX);
            if (cnt + 1 < length) Serial.print(" ");
        #endif
    }
    data[cnt] = i2c_read(true);
    i2c_stop();
    return 0;
}

// i2c初始化, 如果SCL为低电平直接返回false，返回true表示成功
int8_t i2c_init_my(){
    return i2c_init();
}

// #ifdef __cplusplus 
// }
// #endif
