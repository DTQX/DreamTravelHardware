// #include "i2cdev_c.h"
// #include "wire_c.h"

// /** Write multiple bytes to an 8-bit device register.
//  * @param devAddr I2C slave device address
//  * @param regAddr First register address to write to
//  * @param length Number of bytes to write
//  * @param data Buffer to copy new data from
//  * @return Status of operation (true = success)
//  */
// int I2Cdev_writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data) {
//     // #define I2CDEV_SERIAL_DEBUG
//     #ifdef I2CDEV_SERIAL_DEBUG
//         Serial.print("I2C (0x");
//         Serial.print(devAddr, HEX);
//         Serial.print(") writing ");
//         Serial.print(length, DEC);
//         Serial.print(" bytes to 0x");
//         Serial.print(regAddr, HEX);
//         Serial.print("...");
//     #endif
//     uint8_t status = 0;
//     Wire_beginTransmission(devAddr);
//     Wire_write((uint8_t) regAddr); // send address
//     for (uint8_t i = 0; i < length; i++) {
//         #ifdef I2CDEV_SERIAL_DEBUG
//             Serial.print(data[i], HEX);
//             if (i + 1 < length) Serial.print(" ");
//         #endif
//         Wire_write((uint8_t) data[i]);
//     }
// 	status = Wire_endTransmission();
//     #ifdef I2CDEV_SERIAL_DEBUG
//         Serial.println(". Done.");
//     #endif
//     return status == 0;
// }