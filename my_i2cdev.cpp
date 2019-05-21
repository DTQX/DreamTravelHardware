#include "my_i2cdev.h"
// #include <Wire.h>
// #include <Arduino.h>
#include "I2Cdev.h"

#ifdef __cplusplus 
extern "C" {
#endif
uint16_t readTimeout = 1000;

//  #define I2CDEV_SERIAL_DEBUG

// TwoWire Wire;
/** Write multiple bytes to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register address to write to
 * @param length Number of bytes to write
 * @param data Buffer to copy new data from
 * @return Status of operation (0 = success)
 */
uint8_t writeBytes_c(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data){
    if(I2Cdev::writeBytes( devAddr,  regAddr,  length,  data)){
        return 0;
    }else{
        return 1;
    }
}


/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @param readTimeout Optional read readTimeout in milliseconds (0 to disable, leave off to use default class value in I2Cdev::readTimeout)
 * @return 0 success
 */
int8_t readBytes_c(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {
    
    if(I2Cdev::readBytes( devAddr,  regAddr,  length,  data) != -1){
        return 0;
    }else{
        return 1;
    }
    
}

void delay_ms(unsigned long ms){
    return delay(ms);
}

int get_ms(unsigned long *timestamp){
    *timestamp = 0;
    return 0;
}

#ifdef __cplusplus 
}
#endif
