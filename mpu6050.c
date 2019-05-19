#include "mpu6050.h"

// #include "my_i2cdev.h"

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

// test
// #include "C2CPP.h"
// #include "module.h"




/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)

#define MOTION          (0)
#define NO_MOTION       (1)

/* Starting sampling rate. */
#define DEFAULT_MPU_HZ  (100)

#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)

uint8_t packetLength = 0;

struct rx_s {
    unsigned char header[3];
    unsigned char cmd;
};
struct hal_s {
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
    unsigned short report;
    unsigned short dmp_features;
    unsigned char motion_int_mode;
    struct rx_s rx;
};
static struct hal_s hal = {0};

/* USB RX binary semaphore. Actually, it's just a flag. Not included in struct
 * because it's declared extern elsewhere.
 */
volatile unsigned char rx_new;

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from thei
 * driver(s).
 * TODO: The following matrices refer to the configuration on an internal test
 * board at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char gyro_orientation[9] = {1, 0, 0,
                                           0,1, 0,
                                           0, 0, 1};

enum packet_type_e {
    PACKET_TYPE_ACCEL,
    PACKET_TYPE_GYRO,
    PACKET_TYPE_QUAT,
    PACKET_TYPE_TAP,
    PACKET_TYPE_ANDROID_ORIENT,
    PACKET_TYPE_PEDO,
    PACKET_TYPE_MISC
};


static void tap_cb(unsigned char direction, unsigned char count)
{
    // char data[2];
    // data[0] = (char)direction;
    // data[1] = (char)count;
    // send_packet(PACKET_TYPE_TAP, data);
}

static void android_orient_cb(unsigned char orientation)
{
    // send_packet(PACKET_TYPE_ANDROID_ORIENT, &orientation);
}

/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
static inline unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

static inline unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

uint8_t my_mpu_init(int * innerResultCode){
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp;
    struct int_param_s int_param;

    /* Set up gyro.
     * Every function preceded by mpu_ is a driver function and can be found
     * in inv_mpu.h.
     */
    innerResultCode[0] = mpu_init(&int_param, innerResultCode);
    if (innerResultCode[0]){
        // TODO Arduino reset；
        
        return 1;
    }

    /* If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    innerResultCode[0] = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if (innerResultCode[0]){
        return 2;
    }
    
    /* Push both gyro and accel data into the FIFO. */
    
    innerResultCode[0] = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    if (innerResultCode[0]){
        return 3;
    }
    
    innerResultCode[0] = mpu_set_sample_rate(DEFAULT_MPU_HZ);
    if (innerResultCode[0]){
        return 4;
    }
    /* Read back configuration in case it was set improperly. */
    
    innerResultCode[0] = mpu_get_sample_rate(&gyro_rate);
    if (innerResultCode[0]){
        return 5;
    }
    
    innerResultCode[0] = mpu_get_gyro_fsr(&gyro_fsr);
    if (innerResultCode[0]){
        return 6;
    }
    
    innerResultCode[0] = mpu_get_accel_fsr(&accel_fsr);
    if (innerResultCode[0]){
        return 7;
    }

    /* Initialize HAL state variables. */
    memset(&hal, 0, sizeof(hal));
    hal.sensors = ACCEL_ON | GYRO_ON;
    hal.report = PRINT_QUAT;


    /* To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    
    innerResultCode[0] = dmp_load_motion_driver_firmware();
    if (innerResultCode[0]){
        return -8;
    }
    
    innerResultCode[0] = dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation));
    if (innerResultCode[0]){
        return -9;
    }
    
    innerResultCode[0] = dmp_register_tap_cb(tap_cb);
    if (innerResultCode[0]){
        return -10;
    }
    
    innerResultCode[0] = dmp_register_android_orient_cb(android_orient_cb);
    if (innerResultCode[0]){
        return -11;
    }

    /*
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     */
    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL;
    
    innerResultCode[0] = dmp_enable_feature(hal.dmp_features);
    if (innerResultCode[0]){
        return -12;
    }
    
    innerResultCode[0] = dmp_set_fifo_rate(DEFAULT_MPU_HZ);
    if (innerResultCode[0]){
        return -13;
    }
    
    innerResultCode[0] = mpu_set_dmp_state(1);
    if (innerResultCode[0]){
        return -14;
    }
    hal.dmp_on = 1;

}

int mpu_read_latest_fifo(unsigned char *fifo_data){
    /* Get a packet. */
    return mpu_read_latest_fifo_stream(packetLength, fifo_data);
}
