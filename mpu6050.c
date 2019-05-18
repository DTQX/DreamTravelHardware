#include "mpu6050.h"

// #include "i2cdev_c.h"
// #include "I2Cdev.h"
// #include <twi.h>
#include "my_i2cdev.h"


// #ifdef __cplusplus 
// extern "C" {
// #endif


unsigned char addc(){
    uint8_t a[10];
    // return i2cdev_writeBytes_c(1,1,1, a);
    return writeBytes_c(1,1,1, a);
    // return 0;
}



// #ifdef __cplusplus 
// }
// #endif