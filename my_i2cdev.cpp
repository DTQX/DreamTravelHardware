#include "my_i2cdev.h"
#include <Wire.h>
#include <Arduino.h>
// #include "I2Cdev.h"

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
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") writing ");
        Serial.print(length, DEC);
        Serial.print(" bytes to 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif
    uint8_t status = 0;
        Wire.beginTransmission(devAddr);
        Wire.write((uint8_t) regAddr); // send address
    for (uint8_t i = 0; i < length; i++) {
        #ifdef I2CDEV_SERIAL_DEBUG
            Serial.print(data[i], HEX);
            if (i + 1 < length) Serial.print(" ");
        #endif
			Wire.write((uint8_t) data[i]);
    }
        Wire.endTransmission();
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.println(". Done.");
    #endif
    return status;
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
    
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") reading ");
        Serial.print(length, DEC);
        Serial.print(" bytes from 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif

    int8_t count = 0;
    uint32_t t1 = millis();
    // Arduino v1.0.1+, Wire library
    // Adds official support for repeated start condition, yay!

    // I2C/TWI subsystem uses internal buffer that breaks with large data requests
    // so if user requests more than BUFFER_LENGTH bytes, we have to do it in
    // smaller chunks instead of all at once
    
    // for (uint8_t k = 0; k < length; k += min((int)length, BUFFER_LENGTH)) {
    //     Wire.beginTransmission(devAddr);
    //     Wire.write(regAddr);
    //     Wire.endTransmission();
    //     Wire.beginTransmission(devAddr);
    //     Wire.requestFrom(devAddr, (uint8_t)min(length - k, BUFFER_LENGTH));

    //     for (; Wire.available() && (readTimeout == 0 || millis() - t1 < readTimeout); count++) {
    //         data[count] = Wire.read();
    //         #ifdef I2CDEV_SERIAL_DEBUG
    //             Serial.print(data[count], HEX);
    //             if (count + 1 < length) Serial.print(" ");
    //         #endif
    //     }
    // }

    for (uint8_t k = 0; k < length; k += min((int)length, BUFFER_LENGTH)) {
        Wire.beginTransmission(devAddr);
        Wire.write(regAddr);
        Wire.endTransmission();
        // Wire.beginTransmission(devAddr);
        Wire.requestFrom(devAddr, (uint8_t)min(length - k, BUFFER_LENGTH));

        for (; Wire.available() && (readTimeout == 0 || millis() - t1 < readTimeout); count++) {
            data[count] = Wire.read();
            #ifdef I2CDEV_SERIAL_DEBUG
                Serial.print(data[count], HEX);
                if (count + 1 < length) Serial.print(" ");
            #endif
        }
    }

    // TODO加入定时中断，超时则退出
    // check for readTimeout
    if (readTimeout > 0 && millis() - t1 >= readTimeout && count < length) count = -1; // readTimeout

    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.print(". Done (");
        Serial.print(count, DEC);
        Serial.println(" read).");
    #endif
    if(count != -1){
        return 0;
    }else
    {
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
