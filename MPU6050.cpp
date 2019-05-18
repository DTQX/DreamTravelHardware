#include "mpu6050.h"

// #include "i2cdev_c.h"
#include "I2Cdev.h"

// #ifdef __cplusplus 
// extern "C" {
// #endif


unsigned char addc(){
    uint8_t a[10];
    return i2cdev_writeBytes_c(1,1,1, a);

}

// #ifdef __cplusplus 
// }
// #endif