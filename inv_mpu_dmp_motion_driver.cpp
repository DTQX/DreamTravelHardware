/*
 $License:
    Copyright (C) 2011-2012 InvenSense Corporation, All Rights Reserved.
    See included License.txt for License information.
 $
 */
/**
 *  @addtogroup  DRIVERS Sensor Driver Layer
 *  @brief       Hardware drivers to communicate with sensors via I2C.
 *
 *  @{
 *      @file       inv_mpu_dmp_motion_driver.c
 *      @brief      DMP image and interface functions.
 *      @details    All functions are preceded by the dmp_ prefix to
 *                  differentiate among MPL and general driver function calls.
 */
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include <Arduino.h>


/* The following functions must be defined for this platform:
 * i2c_write_bytes(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char const *data)
 * i2c_read_bytes(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char *data)
 * delay_ms(unsigned long num_ms)
 * get_ms(unsigned long *count)
 */
#include "MyI2Cdev.h"



/* These defines are copied from dmpDefaultMPU6050.c in the general MPL
 * releases. These defines may change for each DMP image, so be sure to modify
 * these values when switching to a new image.
 */
#define CFG_LP_QUAT             (2712)
#define END_ORIENT_TEMP         (1866)
#define CFG_27                  (2742)
#define CFG_20                  (2224)
#define CFG_23                  (2745)
#define CFG_FIFO_ON_EVENT       (2690)
#define END_PREDICTION_UPDATE   (1761)
#define CGNOTICE_INTR           (2620)
#define X_GRT_Y_TMP             (1358)
#define CFG_DR_INT              (1029)
#define CFG_AUTH                (1035)
#define UPDATE_PROP_ROT         (1835)
#define END_COMPARE_Y_X_TMP2    (1455)
#define SKIP_X_GRT_Y_TMP        (1359)
#define SKIP_END_COMPARE        (1435)
#define FCFG_3                  (1088)
#define FCFG_2                  (1066)
#define FCFG_1                  (1062)
#define END_COMPARE_Y_X_TMP3    (1434)
#define FCFG_7                  (1073)
#define FCFG_6                  (1106)
#define FLAT_STATE_END          (1713)
#define SWING_END_4             (1616)
#define SWING_END_2             (1565)
#define SWING_END_3             (1587)
#define SWING_END_1             (1550)
#define CFG_8                   (2718)
#define CFG_15                  (2727)
#define CFG_16                  (2746)
#define CFG_EXT_GYRO_BIAS       (1189)
#define END_COMPARE_Y_X_TMP     (1407)
#define DO_NOT_UPDATE_PROP_ROT  (1839)
#define CFG_7                   (1205)
#define FLAT_STATE_END_TEMP     (1683)
#define END_COMPARE_Y_X         (1484)
#define SKIP_SWING_END_1        (1551)
#define SKIP_SWING_END_3        (1588)
#define SKIP_SWING_END_2        (1566)
#define TILTG75_START           (1672)
#define CFG_6                   (2753)
#define TILTL75_END             (1669)
#define END_ORIENT              (1884)
#define CFG_FLICK_IN            (2573)
#define TILTL75_START           (1643)
#define CFG_MOTION_BIAS         (1208)
#define X_GRT_Y                 (1408)
#define TEMPLABEL               (2324)
#define CFG_ANDROID_ORIENT_INT  (1853)
#define CFG_GYRO_RAW_DATA       (2722)
#define X_GRT_Y_TMP2            (1379)

#define D_0_22                  (22+512)
#define D_0_24                  (24+512)

#define D_0_36                  (36)
#define D_0_52                  (52)
#define D_0_96                  (96)
#define D_0_104                 (104)
#define D_0_108                 (108)
#define D_0_163                 (163)
#define D_0_188                 (188)
#define D_0_192                 (192)
#define D_0_224                 (224)
#define D_0_228                 (228)
#define D_0_232                 (232)
#define D_0_236                 (236)

#define D_1_2                   (256 + 2)
#define D_1_4                   (256 + 4)
#define D_1_8                   (256 + 8)
#define D_1_10                  (256 + 10)
#define D_1_24                  (256 + 24)
#define D_1_28                  (256 + 28)
#define D_1_36                  (256 + 36)
#define D_1_40                  (256 + 40)
#define D_1_44                  (256 + 44)
#define D_1_72                  (256 + 72)
#define D_1_74                  (256 + 74)
#define D_1_79                  (256 + 79)
#define D_1_88                  (256 + 88)
#define D_1_90                  (256 + 90)
#define D_1_92                  (256 + 92)
#define D_1_96                  (256 + 96)
#define D_1_98                  (256 + 98)
#define D_1_106                 (256 + 106)
#define D_1_108                 (256 + 108)
#define D_1_112                 (256 + 112)
#define D_1_128                 (256 + 144)
#define D_1_152                 (256 + 12)
#define D_1_160                 (256 + 160)
#define D_1_176                 (256 + 176)
#define D_1_178                 (256 + 178)
#define D_1_218                 (256 + 218)
#define D_1_232                 (256 + 232)
#define D_1_236                 (256 + 236)
#define D_1_240                 (256 + 240)
#define D_1_244                 (256 + 244)
#define D_1_250                 (256 + 250)
#define D_1_252                 (256 + 252)
#define D_2_12                  (512 + 12)
#define D_2_96                  (512 + 96)
#define D_2_108                 (512 + 108)
#define D_2_208                 (512 + 208)
#define D_2_224                 (512 + 224)
#define D_2_236                 (512 + 236)
#define D_2_244                 (512 + 244)
#define D_2_248                 (512 + 248)
#define D_2_252                 (512 + 252)

#define CPASS_BIAS_X            (35 * 16 + 4)
#define CPASS_BIAS_Y            (35 * 16 + 8)
#define CPASS_BIAS_Z            (35 * 16 + 12)
#define CPASS_MTX_00            (36 * 16)
#define CPASS_MTX_01            (36 * 16 + 4)
#define CPASS_MTX_02            (36 * 16 + 8)
#define CPASS_MTX_10            (36 * 16 + 12)
#define CPASS_MTX_11            (37 * 16)
#define CPASS_MTX_12            (37 * 16 + 4)
#define CPASS_MTX_20            (37 * 16 + 8)
#define CPASS_MTX_21            (37 * 16 + 12)
#define CPASS_MTX_22            (43 * 16 + 12)
#define D_EXT_GYRO_BIAS_X       (61 * 16)
#define D_EXT_GYRO_BIAS_Y       (61 * 16) + 4
#define D_EXT_GYRO_BIAS_Z       (61 * 16) + 8
#define D_ACT0                  (40 * 16)
#define D_ACSX                  (40 * 16 + 4)
#define D_ACSY                  (40 * 16 + 8)
#define D_ACSZ                  (40 * 16 + 12)

#define FLICK_MSG               (45 * 16 + 4)
#define FLICK_COUNTER           (45 * 16 + 8)
#define FLICK_LOWER             (45 * 16 + 12)
#define FLICK_UPPER             (46 * 16 + 12)

#define D_AUTH_OUT              (992)
#define D_AUTH_IN               (996)
#define D_AUTH_A                (1000)
#define D_AUTH_B                (1004)

#define D_PEDSTD_BP_B           (768 + 0x1C)
#define D_PEDSTD_HP_A           (768 + 0x78)
#define D_PEDSTD_HP_B           (768 + 0x7C)
#define D_PEDSTD_BP_A4          (768 + 0x40)
#define D_PEDSTD_BP_A3          (768 + 0x44)
#define D_PEDSTD_BP_A2          (768 + 0x48)
#define D_PEDSTD_BP_A1          (768 + 0x4C)
#define D_PEDSTD_INT_THRSH      (768 + 0x68)
#define D_PEDSTD_CLIP           (768 + 0x6C)
#define D_PEDSTD_SB             (768 + 0x28)
#define D_PEDSTD_SB_TIME        (768 + 0x2C)
#define D_PEDSTD_PEAKTHRSH      (768 + 0x98)
#define D_PEDSTD_TIML           (768 + 0x2A)
#define D_PEDSTD_TIMH           (768 + 0x2E)
#define D_PEDSTD_PEAK           (768 + 0X94)
#define D_PEDSTD_STEPCTR        (768 + 0x60)
#define D_PEDSTD_TIMECTR        (964)
#define D_PEDSTD_DECI           (768 + 0xA0)

#define D_HOST_NO_MOT           (976)
#define D_ACCEL_BIAS            (660)

#define D_ORIENT_GAP            (76)

#define D_TILT0_H               (48)
#define D_TILT0_L               (50)
#define D_TILT1_H               (52)
#define D_TILT1_L               (54)
#define D_TILT2_H               (56)
#define D_TILT2_L               (58)
#define D_TILT3_H               (60)
#define D_TILT3_L               (62)

#define DMP_CODE_SIZE           (3062)

static const unsigned char dmp_memory[DMP_CODE_SIZE] = {
    /* bank # 0 */
    0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xff, 0xef, 0x00, 0x00, 0xfa, 0x80, 0x00, 0x0b, 0x12, 0x82, 0x00, 0x01,
    0x03, 0x0c, 0x30, 0xc3, 0x0e, 0x8c, 0x8c, 0xe9, 0x14, 0xd5, 0x40, 0x02, 0x13, 0x71, 0x0f, 0x8e,
    0x38, 0x83, 0xf8, 0x83, 0x30, 0x00, 0xf8, 0x83, 0x25, 0x8e, 0xf8, 0x83, 0x30, 0x00, 0xf8, 0x83,
    0xff, 0xff, 0xff, 0xff, 0x0f, 0xfe, 0xa9, 0xd6, 0x24, 0x00, 0x04, 0x00, 0x1a, 0x82, 0x79, 0xa1,
    0x00, 0x00, 0x00, 0x3c, 0xff, 0xff, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x38, 0x83, 0x6f, 0xa2,
    0x00, 0x3e, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xca, 0xe3, 0x09, 0x3e, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x00, 0x0c, 0x00, 0x00, 0x00, 0x0c, 0x18, 0x6e, 0x00, 0x00, 0x06, 0x92, 0x0a, 0x16, 0xc0, 0xdf,
    0xff, 0xff, 0x02, 0x56, 0xfd, 0x8c, 0xd3, 0x77, 0xff, 0xe1, 0xc4, 0x96, 0xe0, 0xc5, 0xbe, 0xaa,
    0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x0b, 0x2b, 0x00, 0x00, 0x16, 0x57, 0x00, 0x00, 0x03, 0x59,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1d, 0xfa, 0x00, 0x02, 0x6c, 0x1d, 0x00, 0x00, 0x00, 0x00,
    0x3f, 0xff, 0xdf, 0xeb, 0x00, 0x3e, 0xb3, 0xb6, 0x00, 0x0d, 0x22, 0x78, 0x00, 0x00, 0x2f, 0x3c,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x19, 0x42, 0xb5, 0x00, 0x00, 0x39, 0xa2, 0x00, 0x00, 0xb3, 0x65,
    0xd9, 0x0e, 0x9f, 0xc9, 0x1d, 0xcf, 0x4c, 0x34, 0x30, 0x00, 0x00, 0x00, 0x50, 0x00, 0x00, 0x00,
    0x3b, 0xb6, 0x7a, 0xe8, 0x00, 0x64, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* bank # 1 */
    0x10, 0x00, 0x00, 0x00, 0x10, 0x00, 0xfa, 0x92, 0x10, 0x00, 0x22, 0x5e, 0x00, 0x0d, 0x22, 0x9f,
    0x00, 0x01, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0xff, 0x46, 0x00, 0x00, 0x63, 0xd4, 0x00, 0x00,
    0x10, 0x00, 0x00, 0x00, 0x04, 0xd6, 0x00, 0x00, 0x04, 0xcc, 0x00, 0x00, 0x04, 0xcc, 0x00, 0x00,
    0x00, 0x00, 0x10, 0x72, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x06, 0x00, 0x02, 0x00, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x05, 0x00, 0x64, 0x00, 0x20, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x00, 0x32, 0xf8, 0x98, 0x00, 0x00, 0xff, 0x65, 0x00, 0x00, 0x83, 0x0f, 0x00, 0x00,
    0xff, 0x9b, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0xb2, 0x6a, 0x00, 0x02, 0x00, 0x00,
    0x00, 0x01, 0xfb, 0x83, 0x00, 0x68, 0x00, 0x00, 0x00, 0xd9, 0xfc, 0x00, 0x7c, 0xf1, 0xff, 0x83,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x64, 0x03, 0xe8, 0x00, 0x64, 0x00, 0x28,
    0x00, 0x00, 0x00, 0x25, 0x00, 0x00, 0x00, 0x00, 0x16, 0xa0, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00,
    0x00, 0x00, 0x10, 0x00, 0x00, 0x2f, 0x00, 0x00, 0x00, 0x00, 0x01, 0xf4, 0x00, 0x00, 0x10, 0x00,
    /* bank # 2 */
    0x00, 0x28, 0x00, 0x00, 0xff, 0xff, 0x45, 0x81, 0xff, 0xff, 0xfa, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x44, 0x00, 0x05, 0x00, 0x05, 0xba, 0xc6, 0x00, 0x47, 0x78, 0xa2,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x25, 0x4d, 0x00, 0x2f, 0x70, 0x6d, 0x00, 0x00, 0x05, 0xae, 0x00, 0x0c, 0x02, 0xd0,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x64, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x0e,
    0x00, 0x00, 0x0a, 0xc7, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0xff, 0xff, 0xff, 0x9c,
    0x00, 0x00, 0x0b, 0x2b, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x64,
    0xff, 0xe5, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    /* bank # 3 */
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x24, 0x26, 0xd3,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x10, 0x00, 0x96, 0x00, 0x3c,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x0c, 0x0a, 0x4e, 0x68, 0xcd, 0xcf, 0x77, 0x09, 0x50, 0x16, 0x67, 0x59, 0xc6, 0x19, 0xce, 0x82,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xd7, 0x84, 0x00, 0x03, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc7, 0x93, 0x8f, 0x9d, 0x1e, 0x1b, 0x1c, 0x19,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x03, 0x18, 0x85, 0x00, 0x00, 0x40, 0x00,
    0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x67, 0x7d, 0xdf, 0x7e, 0x72, 0x90, 0x2e, 0x55, 0x4c, 0xf6, 0xe6, 0x88,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    /* bank # 4 */
    0xd8, 0xdc, 0xb4, 0xb8, 0xb0, 0xd8, 0xb9, 0xab, 0xf3, 0xf8, 0xfa, 0xb3, 0xb7, 0xbb, 0x8e, 0x9e,
    0xae, 0xf1, 0x32, 0xf5, 0x1b, 0xf1, 0xb4, 0xb8, 0xb0, 0x80, 0x97, 0xf1, 0xa9, 0xdf, 0xdf, 0xdf,
    0xaa, 0xdf, 0xdf, 0xdf, 0xf2, 0xaa, 0xc5, 0xcd, 0xc7, 0xa9, 0x0c, 0xc9, 0x2c, 0x97, 0xf1, 0xa9,
    0x89, 0x26, 0x46, 0x66, 0xb2, 0x89, 0x99, 0xa9, 0x2d, 0x55, 0x7d, 0xb0, 0xb0, 0x8a, 0xa8, 0x96,
    0x36, 0x56, 0x76, 0xf1, 0xba, 0xa3, 0xb4, 0xb2, 0x80, 0xc0, 0xb8, 0xa8, 0x97, 0x11, 0xb2, 0x83,
    0x98, 0xba, 0xa3, 0xf0, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xb2, 0xb9, 0xb4, 0x98, 0x83, 0xf1,
    0xa3, 0x29, 0x55, 0x7d, 0xba, 0xb5, 0xb1, 0xa3, 0x83, 0x93, 0xf0, 0x00, 0x28, 0x50, 0xf5, 0xb2,
    0xb6, 0xaa, 0x83, 0x93, 0x28, 0x54, 0x7c, 0xf1, 0xb9, 0xa3, 0x82, 0x93, 0x61, 0xba, 0xa2, 0xda,
    0xde, 0xdf, 0xdb, 0x81, 0x9a, 0xb9, 0xae, 0xf5, 0x60, 0x68, 0x70, 0xf1, 0xda, 0xba, 0xa2, 0xdf,
    0xd9, 0xba, 0xa2, 0xfa, 0xb9, 0xa3, 0x82, 0x92, 0xdb, 0x31, 0xba, 0xa2, 0xd9, 0xba, 0xa2, 0xf8,
    0xdf, 0x85, 0xa4, 0xd0, 0xc1, 0xbb, 0xad, 0x83, 0xc2, 0xc5, 0xc7, 0xb8, 0xa2, 0xdf, 0xdf, 0xdf,
    0xba, 0xa0, 0xdf, 0xdf, 0xdf, 0xd8, 0xd8, 0xf1, 0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35,
    0x5d, 0xb2, 0xb6, 0xba, 0xaf, 0x8c, 0x96, 0x19, 0x8f, 0x9f, 0xa7, 0x0e, 0x16, 0x1e, 0xb4, 0x9a,
    0xb8, 0xaa, 0x87, 0x2c, 0x54, 0x7c, 0xba, 0xa4, 0xb0, 0x8a, 0xb6, 0x91, 0x32, 0x56, 0x76, 0xb2,
    0x84, 0x94, 0xa4, 0xc8, 0x08, 0xcd, 0xd8, 0xb8, 0xb4, 0xb0, 0xf1, 0x99, 0x82, 0xa8, 0x2d, 0x55,
    0x7d, 0x98, 0xa8, 0x0e, 0x16, 0x1e, 0xa2, 0x2c, 0x54, 0x7c, 0x92, 0xa4, 0xf0, 0x2c, 0x50, 0x78,
    /* bank # 5 */
    0xf1, 0x84, 0xa8, 0x98, 0xc4, 0xcd, 0xfc, 0xd8, 0x0d, 0xdb, 0xa8, 0xfc, 0x2d, 0xf3, 0xd9, 0xba,
    0xa6, 0xf8, 0xda, 0xba, 0xa6, 0xde, 0xd8, 0xba, 0xb2, 0xb6, 0x86, 0x96, 0xa6, 0xd0, 0xf3, 0xc8,
    0x41, 0xda, 0xa6, 0xc8, 0xf8, 0xd8, 0xb0, 0xb4, 0xb8, 0x82, 0xa8, 0x92, 0xf5, 0x2c, 0x54, 0x88,
    0x98, 0xf1, 0x35, 0xd9, 0xf4, 0x18, 0xd8, 0xf1, 0xa2, 0xd0, 0xf8, 0xf9, 0xa8, 0x84, 0xd9, 0xc7,
    0xdf, 0xf8, 0xf8, 0x83, 0xc5, 0xda, 0xdf, 0x69, 0xdf, 0x83, 0xc1, 0xd8, 0xf4, 0x01, 0x14, 0xf1,
    0xa8, 0x82, 0x4e, 0xa8, 0x84, 0xf3, 0x11, 0xd1, 0x82, 0xf5, 0xd9, 0x92, 0x28, 0x97, 0x88, 0xf1,
    0x09, 0xf4, 0x1c, 0x1c, 0xd8, 0x84, 0xa8, 0xf3, 0xc0, 0xf9, 0xd1, 0xd9, 0x97, 0x82, 0xf1, 0x29,
    0xf4, 0x0d, 0xd8, 0xf3, 0xf9, 0xf9, 0xd1, 0xd9, 0x82, 0xf4, 0xc2, 0x03, 0xd8, 0xde, 0xdf, 0x1a,
    0xd8, 0xf1, 0xa2, 0xfa, 0xf9, 0xa8, 0x84, 0x98, 0xd9, 0xc7, 0xdf, 0xf8, 0xf8, 0xf8, 0x83, 0xc7,
    0xda, 0xdf, 0x69, 0xdf, 0xf8, 0x83, 0xc3, 0xd8, 0xf4, 0x01, 0x14, 0xf1, 0x98, 0xa8, 0x82, 0x2e,
    0xa8, 0x84, 0xf3, 0x11, 0xd1, 0x82, 0xf5, 0xd9, 0x92, 0x50, 0x97, 0x88, 0xf1, 0x09, 0xf4, 0x1c,
    0xd8, 0x84, 0xa8, 0xf3, 0xc0, 0xf8, 0xf9, 0xd1, 0xd9, 0x97, 0x82, 0xf1, 0x49, 0xf4, 0x0d, 0xd8,
    0xf3, 0xf9, 0xf9, 0xd1, 0xd9, 0x82, 0xf4, 0xc4, 0x03, 0xd8, 0xde, 0xdf, 0xd8, 0xf1, 0xad, 0x88,
    0x98, 0xcc, 0xa8, 0x09, 0xf9, 0xd9, 0x82, 0x92, 0xa8, 0xf5, 0x7c, 0xf1, 0x88, 0x3a, 0xcf, 0x94,
    0x4a, 0x6e, 0x98, 0xdb, 0x69, 0x31, 0xda, 0xad, 0xf2, 0xde, 0xf9, 0xd8, 0x87, 0x95, 0xa8, 0xf2,
    0x21, 0xd1, 0xda, 0xa5, 0xf9, 0xf4, 0x17, 0xd9, 0xf1, 0xae, 0x8e, 0xd0, 0xc0, 0xc3, 0xae, 0x82,
    /* bank # 6 */
    0xc6, 0x84, 0xc3, 0xa8, 0x85, 0x95, 0xc8, 0xa5, 0x88, 0xf2, 0xc0, 0xf1, 0xf4, 0x01, 0x0e, 0xf1,
    0x8e, 0x9e, 0xa8, 0xc6, 0x3e, 0x56, 0xf5, 0x54, 0xf1, 0x88, 0x72, 0xf4, 0x01, 0x15, 0xf1, 0x98,
    0x45, 0x85, 0x6e, 0xf5, 0x8e, 0x9e, 0x04, 0x88, 0xf1, 0x42, 0x98, 0x5a, 0x8e, 0x9e, 0x06, 0x88,
    0x69, 0xf4, 0x01, 0x1c, 0xf1, 0x98, 0x1e, 0x11, 0x08, 0xd0, 0xf5, 0x04, 0xf1, 0x1e, 0x97, 0x02,
    0x02, 0x98, 0x36, 0x25, 0xdb, 0xf9, 0xd9, 0x85, 0xa5, 0xf3, 0xc1, 0xda, 0x85, 0xa5, 0xf3, 0xdf,
    0xd8, 0x85, 0x95, 0xa8, 0xf3, 0x09, 0xda, 0xa5, 0xfa, 0xd8, 0x82, 0x92, 0xa8, 0xf5, 0x78, 0xf1,
    0x88, 0x1a, 0x84, 0x9f, 0x26, 0x88, 0x98, 0x21, 0xda, 0xf4, 0x1d, 0xf3, 0xd8, 0x87, 0x9f, 0x39,
    0xd1, 0xaf, 0xd9, 0xdf, 0xdf, 0xfb, 0xf9, 0xf4, 0x0c, 0xf3, 0xd8, 0xfa, 0xd0, 0xf8, 0xda, 0xf9,
    0xf9, 0xd0, 0xdf, 0xd9, 0xf9, 0xd8, 0xf4, 0x0b, 0xd8, 0xf3, 0x87, 0x9f, 0x39, 0xd1, 0xaf, 0xd9,
    0xdf, 0xdf, 0xf4, 0x1d, 0xf3, 0xd8, 0xfa, 0xfc, 0xa8, 0x69, 0xf9, 0xf9, 0xaf, 0xd0, 0xda, 0xde,
    0xfa, 0xd9, 0xf8, 0x8f, 0x9f, 0xa8, 0xf1, 0xcc, 0xf3, 0x98, 0xdb, 0x45, 0xd9, 0xaf, 0xdf, 0xd0,
    0xf8, 0xd8, 0xf1, 0x8f, 0x9f, 0xa8, 0xca, 0xf3, 0x88, 0x09, 0xda, 0xaf, 0x8f, 0xcb, 0xf8, 0xd8,
    0xf2, 0xad, 0x97, 0x8d, 0x0c, 0xd9, 0xa5, 0xdf, 0xf9, 0xba, 0xa6, 0xf3, 0xfa, 0xf4, 0x12, 0xf2,
    0xd8, 0x95, 0x0d, 0xd1, 0xd9, 0xba, 0xa6, 0xf3, 0xfa, 0xda, 0xa5, 0xf2, 0xc1, 0xba, 0xa6, 0xf3,
    0xdf, 0xd8, 0xf1, 0xba, 0xb2, 0xb6, 0x86, 0x96, 0xa6, 0xd0, 0xca, 0xf3, 0x49, 0xda, 0xa6, 0xcb,
    0xf8, 0xd8, 0xb0, 0xb4, 0xb8, 0xd8, 0xad, 0x84, 0xf2, 0xc0, 0xdf, 0xf1, 0x8f, 0xcb, 0xc3, 0xa8,
    /* bank # 7 */
    0xb2, 0xb6, 0x86, 0x96, 0xc8, 0xc1, 0xcb, 0xc3, 0xf3, 0xb0, 0xb4, 0x88, 0x98, 0xa8, 0x21, 0xdb,
    0x71, 0x8d, 0x9d, 0x71, 0x85, 0x95, 0x21, 0xd9, 0xad, 0xf2, 0xfa, 0xd8, 0x85, 0x97, 0xa8, 0x28,
    0xd9, 0xf4, 0x08, 0xd8, 0xf2, 0x8d, 0x29, 0xda, 0xf4, 0x05, 0xd9, 0xf2, 0x85, 0xa4, 0xc2, 0xf2,
    0xd8, 0xa8, 0x8d, 0x94, 0x01, 0xd1, 0xd9, 0xf4, 0x11, 0xf2, 0xd8, 0x87, 0x21, 0xd8, 0xf4, 0x0a,
    0xd8, 0xf2, 0x84, 0x98, 0xa8, 0xc8, 0x01, 0xd1, 0xd9, 0xf4, 0x11, 0xd8, 0xf3, 0xa4, 0xc8, 0xbb,
    0xaf, 0xd0, 0xf2, 0xde, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xd8, 0xf1, 0xb8, 0xf6,
    0xb5, 0xb9, 0xb0, 0x8a, 0x95, 0xa3, 0xde, 0x3c, 0xa3, 0xd9, 0xf8, 0xd8, 0x5c, 0xa3, 0xd9, 0xf8,
    0xd8, 0x7c, 0xa3, 0xd9, 0xf8, 0xd8, 0xf8, 0xf9, 0xd1, 0xa5, 0xd9, 0xdf, 0xda, 0xfa, 0xd8, 0xb1,
    0x85, 0x30, 0xf7, 0xd9, 0xde, 0xd8, 0xf8, 0x30, 0xad, 0xda, 0xde, 0xd8, 0xf2, 0xb4, 0x8c, 0x99,
    0xa3, 0x2d, 0x55, 0x7d, 0xa0, 0x83, 0xdf, 0xdf, 0xdf, 0xb5, 0x91, 0xa0, 0xf6, 0x29, 0xd9, 0xfb,
    0xd8, 0xa0, 0xfc, 0x29, 0xd9, 0xfa, 0xd8, 0xa0, 0xd0, 0x51, 0xd9, 0xf8, 0xd8, 0xfc, 0x51, 0xd9,
    0xf9, 0xd8, 0x79, 0xd9, 0xfb, 0xd8, 0xa0, 0xd0, 0xfc, 0x79, 0xd9, 0xfa, 0xd8, 0xa1, 0xf9, 0xf9,
    0xf9, 0xf9, 0xf9, 0xa0, 0xda, 0xdf, 0xdf, 0xdf, 0xd8, 0xa1, 0xf8, 0xf8, 0xf8, 0xf8, 0xf8, 0xac,
    0xde, 0xf8, 0xad, 0xde, 0x83, 0x93, 0xac, 0x2c, 0x54, 0x7c, 0xf1, 0xa8, 0xdf, 0xdf, 0xdf, 0xf6,
    0x9d, 0x2c, 0xda, 0xa0, 0xdf, 0xd9, 0xfa, 0xdb, 0x2d, 0xf8, 0xd8, 0xa8, 0x50, 0xda, 0xa0, 0xd0,
    0xde, 0xd9, 0xd0, 0xf8, 0xf8, 0xf8, 0xdb, 0x55, 0xf8, 0xd8, 0xa8, 0x78, 0xda, 0xa0, 0xd0, 0xdf,
    /* bank # 8 */
    0xd9, 0xd0, 0xfa, 0xf8, 0xf8, 0xf8, 0xf8, 0xdb, 0x7d, 0xf8, 0xd8, 0x9c, 0xa8, 0x8c, 0xf5, 0x30,
    0xdb, 0x38, 0xd9, 0xd0, 0xde, 0xdf, 0xa0, 0xd0, 0xde, 0xdf, 0xd8, 0xa8, 0x48, 0xdb, 0x58, 0xd9,
    0xdf, 0xd0, 0xde, 0xa0, 0xdf, 0xd0, 0xde, 0xd8, 0xa8, 0x68, 0xdb, 0x70, 0xd9, 0xdf, 0xdf, 0xa0,
    0xdf, 0xdf, 0xd8, 0xf1, 0xa8, 0x88, 0x90, 0x2c, 0x54, 0x7c, 0x98, 0xa8, 0xd0, 0x5c, 0x38, 0xd1,
    0xda, 0xf2, 0xae, 0x8c, 0xdf, 0xf9, 0xd8, 0xb0, 0x87, 0xa8, 0xc1, 0xc1, 0xb1, 0x88, 0xa8, 0xc6,
    0xf9, 0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xa8,
    0xf9, 0xda, 0x36, 0xd8, 0xa8, 0xf9, 0xda, 0x36, 0xd8, 0xf7, 0x8d, 0x9d, 0xad, 0xf8, 0x18, 0xda,
    0xf2, 0xae, 0xdf, 0xd8, 0xf7, 0xad, 0xfa, 0x30, 0xd9, 0xa4, 0xde, 0xf9, 0xd8, 0xf2, 0xae, 0xde,
    0xfa, 0xf9, 0x83, 0xa7, 0xd9, 0xc3, 0xc5, 0xc7, 0xf1, 0x88, 0x9b, 0xa7, 0x7a, 0xad, 0xf7, 0xde,
    0xdf, 0xa4, 0xf8, 0x84, 0x94, 0x08, 0xa7, 0x97, 0xf3, 0x00, 0xae, 0xf2, 0x98, 0x19, 0xa4, 0x88,
    0xc6, 0xa3, 0x94, 0x88, 0xf6, 0x32, 0xdf, 0xf2, 0x83, 0x93, 0xdb, 0x09, 0xd9, 0xf2, 0xaa, 0xdf,
    0xd8, 0xd8, 0xae, 0xf8, 0xf9, 0xd1, 0xda, 0xf3, 0xa4, 0xde, 0xa7, 0xf1, 0x88, 0x9b, 0x7a, 0xd8,
    0xf3, 0x84, 0x94, 0xae, 0x19, 0xf9, 0xda, 0xaa, 0xf1, 0xdf, 0xd8, 0xa8, 0x81, 0xc0, 0xc3, 0xc5,
    0xc7, 0xa3, 0x92, 0x83, 0xf6, 0x28, 0xad, 0xde, 0xd9, 0xf8, 0xd8, 0xa3, 0x50, 0xad, 0xd9, 0xf8,
    0xd8, 0xa3, 0x78, 0xad, 0xd9, 0xf8, 0xd8, 0xf8, 0xf9, 0xd1, 0xa1, 0xda, 0xde, 0xc3, 0xc5, 0xc7,
    0xd8, 0xa1, 0x81, 0x94, 0xf8, 0x18, 0xf2, 0xb0, 0x89, 0xac, 0xc3, 0xc5, 0xc7, 0xf1, 0xd8, 0xb8,
    /* bank # 9 */
    0xb4, 0xb0, 0x97, 0x86, 0xa8, 0x31, 0x9b, 0x06, 0x99, 0x07, 0xab, 0x97, 0x28, 0x88, 0x9b, 0xf0,
    0x0c, 0x20, 0x14, 0x40, 0xb0, 0xb4, 0xb8, 0xf0, 0xa8, 0x8a, 0x9a, 0x28, 0x50, 0x78, 0xb7, 0x9b,
    0xa8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xf1, 0xbb, 0xab,
    0x88, 0x00, 0x2c, 0x54, 0x7c, 0xf0, 0xb3, 0x8b, 0xb8, 0xa8, 0x04, 0x28, 0x50, 0x78, 0xf1, 0xb0,
    0x88, 0xb4, 0x97, 0x26, 0xa8, 0x59, 0x98, 0xbb, 0xab, 0xb3, 0x8b, 0x02, 0x26, 0x46, 0x66, 0xb0,
    0xb8, 0xf0, 0x8a, 0x9c, 0xa8, 0x29, 0x51, 0x79, 0x8b, 0x29, 0x51, 0x79, 0x8a, 0x24, 0x70, 0x59,
    0x8b, 0x20, 0x58, 0x71, 0x8a, 0x44, 0x69, 0x38, 0x8b, 0x39, 0x40, 0x68, 0x8a, 0x64, 0x48, 0x31,
    0x8b, 0x30, 0x49, 0x60, 0x88, 0xf1, 0xac, 0x00, 0x2c, 0x54, 0x7c, 0xf0, 0x8c, 0xa8, 0x04, 0x28,
    0x50, 0x78, 0xf1, 0x88, 0x97, 0x26, 0xa8, 0x59, 0x98, 0xac, 0x8c, 0x02, 0x26, 0x46, 0x66, 0xf0,
    0x89, 0x9c, 0xa8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31, 0xa9,
    0x88, 0x09, 0x20, 0x59, 0x70, 0xab, 0x11, 0x38, 0x40, 0x69, 0xa8, 0x19, 0x31, 0x48, 0x60, 0x8c,
    0xa8, 0x3c, 0x41, 0x5c, 0x20, 0x7c, 0x00, 0xf1, 0x87, 0x98, 0x19, 0x86, 0xa8, 0x6e, 0x76, 0x7e,
    0xa9, 0x99, 0x88, 0x2d, 0x55, 0x7d, 0xd8, 0xb1, 0xb5, 0xb9, 0xa3, 0xdf, 0xdf, 0xdf, 0xae, 0xd0,
    0xdf, 0xaa, 0xd0, 0xde, 0xf2, 0xab, 0xf8, 0xf9, 0xd9, 0xb0, 0x87, 0xc4, 0xaa, 0xf1, 0xdf, 0xdf,
    0xbb, 0xaf, 0xdf, 0xdf, 0xb9, 0xd8, 0xb1, 0xf1, 0xa3, 0x97, 0x8e, 0x60, 0xdf, 0xb0, 0x84, 0xf2,
    0xc8, 0xf8, 0xf9, 0xd9, 0xde, 0xd8, 0x93, 0x85, 0xf1, 0x4a, 0xb1, 0x83, 0xa3, 0x08, 0xb5, 0x83,
    /* bank # 10 */
    0x9a, 0x08, 0x10, 0xb7, 0x9f, 0x10, 0xd8, 0xf1, 0xb0, 0xba, 0xae, 0xb0, 0x8a, 0xc2, 0xb2, 0xb6,
    0x8e, 0x9e, 0xf1, 0xfb, 0xd9, 0xf4, 0x1d, 0xd8, 0xf9, 0xd9, 0x0c, 0xf1, 0xd8, 0xf8, 0xf8, 0xad,
    0x61, 0xd9, 0xae, 0xfb, 0xd8, 0xf4, 0x0c, 0xf1, 0xd8, 0xf8, 0xf8, 0xad, 0x19, 0xd9, 0xae, 0xfb,
    0xdf, 0xd8, 0xf4, 0x16, 0xf1, 0xd8, 0xf8, 0xad, 0x8d, 0x61, 0xd9, 0xf4, 0xf4, 0xac, 0xf5, 0x9c,
    0x9c, 0x8d, 0xdf, 0x2b, 0xba, 0xb6, 0xae, 0xfa, 0xf8, 0xf4, 0x0b, 0xd8, 0xf1, 0xae, 0xd0, 0xf8,
    0xad, 0x51, 0xda, 0xae, 0xfa, 0xf8, 0xf1, 0xd8, 0xb9, 0xb1, 0xb6, 0xa3, 0x83, 0x9c, 0x08, 0xb9,
    0xb1, 0x83, 0x9a, 0xb5, 0xaa, 0xc0, 0xfd, 0x30, 0x83, 0xb7, 0x9f, 0x10, 0xb5, 0x8b, 0x93, 0xf2,
    0x02, 0x02, 0xd1, 0xab, 0xda, 0xde, 0xd8, 0xf1, 0xb0, 0x80, 0xba, 0xab, 0xc0, 0xc3, 0xb2, 0x84,
    0xc1, 0xc3, 0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9, 0xab, 0xde, 0xb0,
    0x87, 0x9c, 0xb9, 0xa3, 0xdd, 0xf1, 0xb3, 0x8b, 0x8b, 0x8b, 0x8b, 0x8b, 0xb0, 0x87, 0xa3, 0xa3,
    0xa3, 0xa3, 0xb2, 0x8b, 0xb6, 0x9b, 0xf2, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3,
    0xa3, 0xf1, 0xb0, 0x87, 0xb5, 0x9a, 0xa3, 0xf3, 0x9b, 0xa3, 0xa3, 0xdc, 0xba, 0xac, 0xdf, 0xb9,
    0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3, 0xa3,
    0xd8, 0xd8, 0xd8, 0xbb, 0xb3, 0xb7, 0xf1, 0xaa, 0xf9, 0xda, 0xff, 0xd9, 0x80, 0x9a, 0xaa, 0x28,
    0xb4, 0x80, 0x98, 0xa7, 0x20, 0xb7, 0x97, 0x87, 0xa8, 0x66, 0x88, 0xf0, 0x79, 0x51, 0xf1, 0x90,
    0x2c, 0x87, 0x0c, 0xa7, 0x81, 0x97, 0x62, 0x93, 0xf0, 0x71, 0x71, 0x60, 0x85, 0x94, 0x01, 0x29,
    /* bank # 11 */
    0x51, 0x79, 0x90, 0xa5, 0xf1, 0x28, 0x4c, 0x6c, 0x87, 0x0c, 0x95, 0x18, 0x85, 0x78, 0xa3, 0x83,
    0x90, 0x28, 0x4c, 0x6c, 0x88, 0x6c, 0xd8, 0xf3, 0xa2, 0x82, 0x00, 0xf2, 0x10, 0xa8, 0x92, 0x19,
    0x80, 0xa2, 0xf2, 0xd9, 0x26, 0xd8, 0xf1, 0x88, 0xa8, 0x4d, 0xd9, 0x48, 0xd8, 0x96, 0xa8, 0x39,
    0x80, 0xd9, 0x3c, 0xd8, 0x95, 0x80, 0xa8, 0x39, 0xa6, 0x86, 0x98, 0xd9, 0x2c, 0xda, 0x87, 0xa7,
    0x2c, 0xd8, 0xa8, 0x89, 0x95, 0x19, 0xa9, 0x80, 0xd9, 0x38, 0xd8, 0xa8, 0x89, 0x39, 0xa9, 0x80,
    0xda, 0x3c, 0xd8, 0xa8, 0x2e, 0xa8, 0x39, 0x90, 0xd9, 0x0c, 0xd8, 0xa8, 0x95, 0x31, 0x98, 0xd9,
    0x0c, 0xd8, 0xa8, 0x09, 0xd9, 0xff, 0xd8, 0x01, 0xda, 0xff, 0xd8, 0x95, 0x39, 0xa9, 0xda, 0x26,
    0xff, 0xd8, 0x90, 0xa8, 0x0d, 0x89, 0x99, 0xa8, 0x10, 0x80, 0x98, 0x21, 0xda, 0x2e, 0xd8, 0x89,
    0x99, 0xa8, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8, 0x86, 0x96, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8,
    0x87, 0x31, 0x80, 0xda, 0x2e, 0xd8, 0xa8, 0x82, 0x92, 0xf3, 0x41, 0x80, 0xf1, 0xd9, 0x2e, 0xd8,
    0xa8, 0x82, 0xf3, 0x19, 0x80, 0xf1, 0xd9, 0x2e, 0xd8, 0x82, 0xac, 0xf3, 0xc0, 0xa2, 0x80, 0x22,
    0xf1, 0xa6, 0x2e, 0xa7, 0x2e, 0xa9, 0x22, 0x98, 0xa8, 0x29, 0xda, 0xac, 0xde, 0xff, 0xd8, 0xa2,
    0xf2, 0x2a, 0xf1, 0xa9, 0x2e, 0x82, 0x92, 0xa8, 0xf2, 0x31, 0x80, 0xa6, 0x96, 0xf1, 0xd9, 0x00,
    0xac, 0x8c, 0x9c, 0x0c, 0x30, 0xac, 0xde, 0xd0, 0xde, 0xff, 0xd8, 0x8c, 0x9c, 0xac, 0xd0, 0x10,
    0xac, 0xde, 0x80, 0x92, 0xa2, 0xf2, 0x4c, 0x82, 0xa8, 0xf1, 0xca, 0xf2, 0x35, 0xf1, 0x96, 0x88,
    0xa6, 0xd9, 0x00, 0xd8, 0xf1, 0xff
};

static const unsigned short sStartAddress = 0x0400;

/* END OF SECTION COPIED FROM dmpDefaultMPU6050.c */

#define INT_SRC_TAP             (0x01)
#define INT_SRC_ANDROID_ORIENT  (0x08)

#define DMP_FEATURE_SEND_ANY_GYRO   (DMP_FEATURE_SEND_RAW_GYRO | \
                                     DMP_FEATURE_SEND_CAL_GYRO)

#define MAX_PACKET_LENGTH   (32)

#define DMP_SAMPLE_RATE     (200)
#define GYRO_SF             (46850825LL * 200 / DMP_SAMPLE_RATE)

#define FIFO_CORRUPTION_CHECK
#ifdef FIFO_CORRUPTION_CHECK
#define QUAT_ERROR_THRESH       (1L<<24)
#define QUAT_MAG_SQ_NORMALIZED  (1L<<28)
#define QUAT_MAG_SQ_MIN         (QUAT_MAG_SQ_NORMALIZED - QUAT_ERROR_THRESH)
#define QUAT_MAG_SQ_MAX         (QUAT_MAG_SQ_NORMALIZED + QUAT_ERROR_THRESH)
#endif

struct dmp_s {
    void (*tap_cb)(unsigned char count, unsigned char direction);
    void (*android_orient_cb)(unsigned char orientation);
    unsigned short orient;
    unsigned short feature_mask;
    unsigned short fifo_rate;
    unsigned char packet_length;
};

static struct dmp_s dmp;

int dmp_init_struct(){
    dmp.tap_cb = NULL;
    dmp.android_orient_cb = NULL;
    dmp.orient = 0;
    dmp.feature_mask = 0;
    dmp.fifo_rate = 0;
    dmp.packet_length = 0;
}

/**
 *  @brief  Load the DMP with this image.
 *  @return 0 if successful.
 */
int dmp_load_motion_driver_firmware(void)
{
    return mpu_load_firmware(DMP_CODE_SIZE, dmp_memory, sStartAddress,
        DMP_SAMPLE_RATE);
}

/**
 *  @brief      Push gyro and accel orientation to the DMP.
 *  The orientation is represented here as the output of
 *  @e inv_orientation_matrix_to_scalar.
 *  @param[in]  orient  Gyro and accel orientation in body frame.
 *  @return     0 if successful.
 */
int dmp_set_orientation(unsigned short orient)
{
    unsigned char gyro_regs[3], accel_regs[3];
    const unsigned char gyro_axes[3] = {DINA4C, DINACD, DINA6C};
    const unsigned char accel_axes[3] = {DINA0C, DINAC9, DINA2C};
    const unsigned char gyro_sign[3] = {DINA36, DINA56, DINA76};
    const unsigned char accel_sign[3] = {DINA26, DINA46, DINA66};

    gyro_regs[0] = gyro_axes[orient & 3];
    gyro_regs[1] = gyro_axes[(orient >> 3) & 3];
    gyro_regs[2] = gyro_axes[(orient >> 6) & 3];
    accel_regs[0] = accel_axes[orient & 3];
    accel_regs[1] = accel_axes[(orient >> 3) & 3];
    accel_regs[2] = accel_axes[(orient >> 6) & 3];

    /* Chip-to-body, axes only. */
    if (mpu_write_mem(FCFG_1, 3, gyro_regs))
        return -1;
    if (mpu_write_mem(FCFG_2, 3, accel_regs))
        return -1;

    memcpy(gyro_regs, gyro_sign, 3);
    memcpy(accel_regs, accel_sign, 3);
    if (orient & 4) {
        gyro_regs[0] |= 1;
        accel_regs[0] |= 1;
    }
    if (orient & 0x20) {
        gyro_regs[1] |= 1;
        accel_regs[1] |= 1;
    }
    if (orient & 0x100) {
        gyro_regs[2] |= 1;
        accel_regs[2] |= 1;
    }

    /* Chip-to-body, sign only. */
    if (mpu_write_mem(FCFG_3, 3, gyro_regs))
        return -1;
    if (mpu_write_mem(FCFG_7, 3, accel_regs))
        return -1;
    dmp.orient = orient;
    return 0;
}

/**
 *  @brief      Push gyro biases to the DMP.
 *  Because the gyro integration is handled in the DMP, any gyro biases
 *  calculated by the MPL should be pushed down to DMP memory to remove
 *  3-axis quaternion drift.
 *  \n NOTE: If the DMP-based gyro calibration is enabled, the DMP will
 *  overwrite the biases written to this location once a new one is computed.
 *  @param[in]  bias    Gyro biases in q16.
 *  @return     0 if successful.
 */
int dmp_set_gyro_bias(long *bias)
{
    long gyro_bias_body[3];
    unsigned char regs[4];

    gyro_bias_body[0] = bias[dmp.orient & 3];
    if (dmp.orient & 4)
        gyro_bias_body[0] *= -1;
    gyro_bias_body[1] = bias[(dmp.orient >> 3) & 3];
    if (dmp.orient & 0x20)
        gyro_bias_body[1] *= -1;
    gyro_bias_body[2] = bias[(dmp.orient >> 6) & 3];
    if (dmp.orient & 0x100)
        gyro_bias_body[2] *= -1;

#ifdef EMPL_NO_64BIT
    gyro_bias_body[0] = (long)(((float)gyro_bias_body[0] * GYRO_SF) / 1073741824.f);
    gyro_bias_body[1] = (long)(((float)gyro_bias_body[1] * GYRO_SF) / 1073741824.f);
    gyro_bias_body[2] = (long)(((float)gyro_bias_body[2] * GYRO_SF) / 1073741824.f);
#else
    gyro_bias_body[0] = (long)(((long long)gyro_bias_body[0] * GYRO_SF) >> 30);
    gyro_bias_body[1] = (long)(((long long)gyro_bias_body[1] * GYRO_SF) >> 30);
    gyro_bias_body[2] = (long)(((long long)gyro_bias_body[2] * GYRO_SF) >> 30);
#endif

    regs[0] = (unsigned char)((gyro_bias_body[0] >> 24) & 0xFF);
    regs[1] = (unsigned char)((gyro_bias_body[0] >> 16) & 0xFF);
    regs[2] = (unsigned char)((gyro_bias_body[0] >> 8) & 0xFF);
    regs[3] = (unsigned char)(gyro_bias_body[0] & 0xFF);
    if (mpu_write_mem(D_EXT_GYRO_BIAS_X, 4, regs))
        return -1;

    regs[0] = (unsigned char)((gyro_bias_body[1] >> 24) & 0xFF);
    regs[1] = (unsigned char)((gyro_bias_body[1] >> 16) & 0xFF);
    regs[2] = (unsigned char)((gyro_bias_body[1] >> 8) & 0xFF);
    regs[3] = (unsigned char)(gyro_bias_body[1] & 0xFF);
    if (mpu_write_mem(D_EXT_GYRO_BIAS_Y, 4, regs))
        return -1;

    regs[0] = (unsigned char)((gyro_bias_body[2] >> 24) & 0xFF);
    regs[1] = (unsigned char)((gyro_bias_body[2] >> 16) & 0xFF);
    regs[2] = (unsigned char)((gyro_bias_body[2] >> 8) & 0xFF);
    regs[3] = (unsigned char)(gyro_bias_body[2] & 0xFF);
    return mpu_write_mem(D_EXT_GYRO_BIAS_Z, 4, regs);
}

/**
 *  @brief      Push accel biases to the DMP.
 *  These biases will be removed from the DMP 6-axis quaternion.
 *  @param[in]  bias    Accel biases in q16.
 *  @return     0 if successful.
 */
int dmp_set_accel_bias(long *bias)
{
    long accel_bias_body[3];
    unsigned char regs[12];
    long long accel_sf;
    unsigned short accel_sens;

    mpu_get_accel_sens(&accel_sens);
    accel_sf = (long long)accel_sens << 15;
    // __no_operation();

    accel_bias_body[0] = bias[dmp.orient & 3];
    if (dmp.orient & 4)
        accel_bias_body[0] *= -1;
    accel_bias_body[1] = bias[(dmp.orient >> 3) & 3];
    if (dmp.orient & 0x20)
        accel_bias_body[1] *= -1;
    accel_bias_body[2] = bias[(dmp.orient >> 6) & 3];
    if (dmp.orient & 0x100)
        accel_bias_body[2] *= -1;

#ifdef EMPL_NO_64BIT
    accel_bias_body[0] = (long)(((float)accel_bias_body[0] * accel_sf) / 1073741824.f);
    accel_bias_body[1] = (long)(((float)accel_bias_body[1] * accel_sf) / 1073741824.f);
    accel_bias_body[2] = (long)(((float)accel_bias_body[2] * accel_sf) / 1073741824.f);
#else
    accel_bias_body[0] = (long)(((long long)accel_bias_body[0] * accel_sf) >> 30);
    accel_bias_body[1] = (long)(((long long)accel_bias_body[1] * accel_sf) >> 30);
    accel_bias_body[2] = (long)(((long long)accel_bias_body[2] * accel_sf) >> 30);
#endif

    regs[0] = (unsigned char)((accel_bias_body[0] >> 24) & 0xFF);
    regs[1] = (unsigned char)((accel_bias_body[0] >> 16) & 0xFF);
    regs[2] = (unsigned char)((accel_bias_body[0] >> 8) & 0xFF);
    regs[3] = (unsigned char)(accel_bias_body[0] & 0xFF);
    regs[4] = (unsigned char)((accel_bias_body[1] >> 24) & 0xFF);
    regs[5] = (unsigned char)((accel_bias_body[1] >> 16) & 0xFF);
    regs[6] = (unsigned char)((accel_bias_body[1] >> 8) & 0xFF);
    regs[7] = (unsigned char)(accel_bias_body[1] & 0xFF);
    regs[8] = (unsigned char)((accel_bias_body[2] >> 24) & 0xFF);
    regs[9] = (unsigned char)((accel_bias_body[2] >> 16) & 0xFF);
    regs[10] = (unsigned char)((accel_bias_body[2] >> 8) & 0xFF);
    regs[11] = (unsigned char)(accel_bias_body[2] & 0xFF);
    return mpu_write_mem(D_ACCEL_BIAS, 12, regs);
}

/**
 *  @brief      Set DMP output rate.
 *  Only used when DMP is on.
 *  @param[in]  rate    Desired fifo rate (Hz).
 *  @return     0 if successful.
 */
int dmp_set_fifo_rate(unsigned short rate)
{
    const unsigned char regs_end[12] = {DINAFE, DINAF2, DINAAB,
        0xc4, DINAAA, DINAF1, DINADF, DINADF, 0xBB, 0xAF, DINADF, DINADF};
    unsigned short div;
    unsigned char tmp[8];

    if (rate > DMP_SAMPLE_RATE)
        return -1;
    div = DMP_SAMPLE_RATE / rate - 1;
    tmp[0] = (unsigned char)((div >> 8) & 0xFF);
    tmp[1] = (unsigned char)(div & 0xFF);
    if (mpu_write_mem(D_0_22, 2, tmp))
        return -1;
    if (mpu_write_mem(CFG_6, 12, (unsigned char*)regs_end))
        return -1;

    dmp.fifo_rate = rate;
    return 0;
}

/**
 *  @brief      Get DMP output rate.
 *  @param[out] rate    Current fifo rate (Hz).
 *  @return     0 if successful.
 */
int dmp_get_fifo_rate(unsigned short *rate)
{
    rate[0] = dmp.fifo_rate;
    return 0;
}

/**
 *  @brief      Set tap threshold for a specific axis.
 *  @param[in]  axis    1, 2, and 4 for XYZ accel, respectively.
 *  @param[in]  thresh  Tap threshold, in mg/ms.
 *  @return     0 if successful.
 */
int dmp_set_tap_thresh(unsigned char axis, unsigned short thresh)
{
    unsigned char tmp[4], accel_fsr;
    float scaled_thresh;
    unsigned short dmp_thresh, dmp_thresh_2;
    if (!(axis & TAP_XYZ) || thresh > 1600)
        return -1;

    scaled_thresh = (float)thresh / DMP_SAMPLE_RATE;

    mpu_get_accel_fsr(&accel_fsr);
    switch (accel_fsr) {
    case 2:
        dmp_thresh = (unsigned short)(scaled_thresh * 16384);
        /* dmp_thresh * 0.75 */
        dmp_thresh_2 = (unsigned short)(scaled_thresh * 12288);
        break;
    case 4:
        dmp_thresh = (unsigned short)(scaled_thresh * 8192);
        /* dmp_thresh * 0.75 */
        dmp_thresh_2 = (unsigned short)(scaled_thresh * 6144);
        break;
    case 8:
        dmp_thresh = (unsigned short)(scaled_thresh * 4096);
        /* dmp_thresh * 0.75 */
        dmp_thresh_2 = (unsigned short)(scaled_thresh * 3072);
        break;
    case 16:
        dmp_thresh = (unsigned short)(scaled_thresh * 2048);
        /* dmp_thresh * 0.75 */
        dmp_thresh_2 = (unsigned short)(scaled_thresh * 1536);
        break;
    default:
        return -1;
    }
    tmp[0] = (unsigned char)(dmp_thresh >> 8);
    tmp[1] = (unsigned char)(dmp_thresh & 0xFF);
    tmp[2] = (unsigned char)(dmp_thresh_2 >> 8);
    tmp[3] = (unsigned char)(dmp_thresh_2 & 0xFF);

    if (axis & TAP_X) {
        if (mpu_write_mem(DMP_TAP_THX, 2, tmp))
            return -1;
        if (mpu_write_mem(D_1_36, 2, tmp+2))
            return -1;
    }
    if (axis & TAP_Y) {
        if (mpu_write_mem(DMP_TAP_THY, 2, tmp))
            return -1;
        if (mpu_write_mem(D_1_40, 2, tmp+2))
            return -1;
    }
    if (axis & TAP_Z) {
        if (mpu_write_mem(DMP_TAP_THZ, 2, tmp))
            return -1;
        if (mpu_write_mem(D_1_44, 2, tmp+2))
            return -1;
    }
    return 0;
}

/**
 *  @brief      Set which axes will register a tap.
 *  @param[in]  axis    1, 2, and 4 for XYZ, respectively.
 *  @return     0 if successful.
 */
int dmp_set_tap_axes(unsigned char axis)
{
    unsigned char tmp = 0;

    if (axis & TAP_X)
        tmp |= 0x30;
    if (axis & TAP_Y)
        tmp |= 0x0C;
    if (axis & TAP_Z)
        tmp |= 0x03;
    return mpu_write_mem(D_1_72, 1, &tmp);
}

/**
 *  @brief      Set minimum number of taps needed for an interrupt.
 *  @param[in]  min_taps    Minimum consecutive taps (1-4).
 *  @return     0 if successful.
 */
int dmp_set_tap_count(unsigned char min_taps)
{
    unsigned char tmp;

    if (min_taps < 1)
        min_taps = 1;
    else if (min_taps > 4)
        min_taps = 4;

    tmp = min_taps - 1;
    return mpu_write_mem(D_1_79, 1, &tmp);
}

/**
 *  @brief      Set length between valid taps.
 *  @param[in]  time    Milliseconds between taps.
 *  @return     0 if successful.
 */
int dmp_set_tap_time(unsigned short time)
{
    unsigned short dmp_time;
    unsigned char tmp[2];

    dmp_time = time / (1000 / DMP_SAMPLE_RATE);
    tmp[0] = (unsigned char)(dmp_time >> 8);
    tmp[1] = (unsigned char)(dmp_time & 0xFF);
    return mpu_write_mem(DMP_TAPW_MIN, 2, tmp);
}

/**
 *  @brief      Set max time between taps to register as a multi-tap.
 *  @param[in]  time    Max milliseconds between taps.
 *  @return     0 if successful.
 */
int dmp_set_tap_time_multi(unsigned short time)
{
    unsigned short dmp_time;
    unsigned char tmp[2];

    dmp_time = time / (1000 / DMP_SAMPLE_RATE);
    tmp[0] = (unsigned char)(dmp_time >> 8);
    tmp[1] = (unsigned char)(dmp_time & 0xFF);
    return mpu_write_mem(D_1_218, 2, tmp);
}

/**
 *  @brief      Set shake rejection threshold.
 *  If the DMP detects a gyro sample larger than @e thresh, taps are rejected.
 *  @param[in]  sf      Gyro scale factor.
 *  @param[in]  thresh  Gyro threshold in dps.
 *  @return     0 if successful.
 */
int dmp_set_shake_reject_thresh(long sf, unsigned short thresh)
{
    unsigned char tmp[4];
    long thresh_scaled = sf / 1000 * thresh;
    tmp[0] = (unsigned char)(((long)thresh_scaled >> 24) & 0xFF);
    tmp[1] = (unsigned char)(((long)thresh_scaled >> 16) & 0xFF);
    tmp[2] = (unsigned char)(((long)thresh_scaled >> 8) & 0xFF);
    tmp[3] = (unsigned char)((long)thresh_scaled & 0xFF);
    return mpu_write_mem(D_1_92, 4, tmp);
}

/**
 *  @brief      Set shake rejection time.
 *  Sets the length of time that the gyro must be outside of the threshold set
 *  by @e gyro_set_shake_reject_thresh before taps are rejected. A mandatory
 *  60 ms is added to this parameter.
 *  @param[in]  time    Time in milliseconds.
 *  @return     0 if successful.
 */
int dmp_set_shake_reject_time(unsigned short time)
{
    unsigned char tmp[2];

    time /= (1000 / DMP_SAMPLE_RATE);
    tmp[0] = time >> 8;
    tmp[1] = time & 0xFF;
    return mpu_write_mem(D_1_90,2,tmp);
}

/**
 *  @brief      Set shake rejection timeout.
 *  Sets the length of time after a shake rejection that the gyro must stay
 *  inside of the threshold before taps can be detected again. A mandatory
 *  60 ms is added to this parameter.
 *  @param[in]  time    Time in milliseconds.
 *  @return     0 if successful.
 */
int dmp_set_shake_reject_timeout(unsigned short time)
{
    unsigned char tmp[2];

    time /= (1000 / DMP_SAMPLE_RATE);
    tmp[0] = time >> 8;
    tmp[1] = time & 0xFF;
    return mpu_write_mem(D_1_88,2,tmp);
}

/**
 *  @brief      Get current step count.
 *  @param[out] count   Number of steps detected.
 *  @return     0 if successful.
 */
int dmp_get_pedometer_step_count(unsigned long *count)
{
    unsigned char tmp[4];
    if (!count)
        return -1;

    if (mpu_read_mem(D_PEDSTD_STEPCTR, 4, tmp))
        return -1;

    count[0] = ((unsigned long)tmp[0] << 24) | ((unsigned long)tmp[1] << 16) |
        ((unsigned long)tmp[2] << 8) | tmp[3];
    return 0;
}

/**
 *  @brief      Overwrite current step count.
 *  WARNING: This function writes to DMP memory and could potentially encounter
 *  a race condition if called while the pedometer is enabled.
 *  @param[in]  count   New step count.
 *  @return     0 if successful.
 */
int dmp_set_pedometer_step_count(unsigned long count)
{
    unsigned char tmp[4];

    tmp[0] = (unsigned char)((count >> 24) & 0xFF);
    tmp[1] = (unsigned char)((count >> 16) & 0xFF);
    tmp[2] = (unsigned char)((count >> 8) & 0xFF);
    tmp[3] = (unsigned char)(count & 0xFF);
    return mpu_write_mem(D_PEDSTD_STEPCTR, 4, tmp);
}

/**
 *  @brief      Get duration of walking time.
 *  @param[in]  time    Walk time in milliseconds.
 *  @return     0 if successful.
 */
int dmp_get_pedometer_walk_time(unsigned long *time)
{
    unsigned char tmp[4];
    if (!time)
        return -1;

    if (mpu_read_mem(D_PEDSTD_TIMECTR, 4, tmp))
        return -1;

    time[0] = (((unsigned long)tmp[0] << 24) | ((unsigned long)tmp[1] << 16) |
        ((unsigned long)tmp[2] << 8) | tmp[3]) * 20;
    return 0;
}

/**
 *  @brief      Overwrite current walk time.
 *  WARNING: This function writes to DMP memory and could potentially encounter
 *  a race condition if called while the pedometer is enabled.
 *  @param[in]  time    New walk time in milliseconds.
 */
int dmp_set_pedometer_walk_time(unsigned long time)
{
    unsigned char tmp[4];

    time /= 20;

    tmp[0] = (unsigned char)((time >> 24) & 0xFF);
    tmp[1] = (unsigned char)((time >> 16) & 0xFF);
    tmp[2] = (unsigned char)((time >> 8) & 0xFF);
    tmp[3] = (unsigned char)(time & 0xFF);
    return mpu_write_mem(D_PEDSTD_TIMECTR, 4, tmp);
}

/**
 *  @brief      Enable DMP features.
 *  The following \#define's are used in the input mask:
 *  \n DMP_FEATURE_TAP
 *  \n DMP_FEATURE_ANDROID_ORIENT
 *  \n DMP_FEATURE_LP_QUAT
 *  \n DMP_FEATURE_6X_LP_QUAT
 *  \n DMP_FEATURE_GYRO_CAL
 *  \n DMP_FEATURE_SEND_RAW_ACCEL
 *  \n DMP_FEATURE_SEND_RAW_GYRO
 *  \n NOTE: DMP_FEATURE_LP_QUAT and DMP_FEATURE_6X_LP_QUAT are mutually
 *  exclusive.
 *  \n NOTE: DMP_FEATURE_SEND_RAW_GYRO and DMP_FEATURE_SEND_CAL_GYRO are also
 *  mutually exclusive.
 *  @param[in]  mask    Mask of features to enable.
 *  @return     0 if successful.
 */
int dmp_enable_feature(unsigned short mask)
{
    unsigned char tmp[10];

    /* TODO: All of these settings can probably be integrated into the default
     * DMP image.
     */
    /* Set integration scale factor. */
    tmp[0] = (unsigned char)((GYRO_SF >> 24) & 0xFF);
    tmp[1] = (unsigned char)((GYRO_SF >> 16) & 0xFF);
    tmp[2] = (unsigned char)((GYRO_SF >> 8) & 0xFF);
    tmp[3] = (unsigned char)(GYRO_SF & 0xFF);
    int result;
    result = mpu_write_mem(D_0_104, 4, tmp);
    if(result){
        return -1;
    }
    

    /* Send sensor data to the FIFO. */
    tmp[0] = 0xA3;
    if (mask & DMP_FEATURE_SEND_RAW_ACCEL) {
        tmp[1] = 0xC0;
        tmp[2] = 0xC8;
        tmp[3] = 0xC2;
    } else {
        tmp[1] = 0xA3;
        tmp[2] = 0xA3;
        tmp[3] = 0xA3;
    }
    if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
        tmp[4] = 0xC4;
        tmp[5] = 0xCC;
        tmp[6] = 0xC6;
    } else {
        tmp[4] = 0xA3;
        tmp[5] = 0xA3;
        tmp[6] = 0xA3;
    }
    tmp[7] = 0xA3;
    tmp[8] = 0xA3;
    tmp[9] = 0xA3;
    result = mpu_write_mem(CFG_15,10,tmp);
    if(result){
        return -2;
    }
    
    

    /* Send gesture data to the FIFO. */
    if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
        tmp[0] = DINA20;
    else
        tmp[0] = 0xD8;
    
    result = mpu_write_mem(CFG_27,1,tmp);
    if(result){
        return -3;
    }
    
    

    if (mask & DMP_FEATURE_GYRO_CAL){
        result = dmp_enable_gyro_cal(1);
        if(result){
            return -4;
        }
    
    }
        
    else{
        result = dmp_enable_gyro_cal(0);
        if(result){
            return -5;
        }
    
    }
        

    if (mask & DMP_FEATURE_SEND_ANY_GYRO) {
        if (mask & DMP_FEATURE_SEND_CAL_GYRO) {
            tmp[0] = 0xB2;
            tmp[1] = 0x8B;
            tmp[2] = 0xB6;
            tmp[3] = 0x9B;
        } else {
            tmp[0] = DINAC0;
            tmp[1] = DINA80;
            tmp[2] = DINAC2;
            tmp[3] = DINA90;
        }
        result = mpu_write_mem(CFG_GYRO_RAW_DATA, 4, tmp);
        if(result){
            return -6;
        }
    
        
    }

    if (mask & DMP_FEATURE_TAP) {
        /* Enable tap. */
        tmp[0] = 0xF8;
        result = mpu_write_mem(CFG_20, 1, tmp);
        if(result){
            return -7;
        }
    
        
        result = dmp_set_tap_thresh(TAP_XYZ, 250);
        if(result){
            return -8;
        }
    
        
        result = dmp_set_tap_axes(TAP_XYZ);
        if(result){
            return -9;
        }
    
        
        result = dmp_set_tap_count(1);
        if(result){
            return -10;
        }
    
        
        result = dmp_set_tap_time(100);
        if(result){
            return -11;
        }
    
        
        result = dmp_set_tap_time_multi(500);
        if(result){
            return -12;
        }
    
        
        result = dmp_set_shake_reject_thresh(GYRO_SF, 200);
        if(result){
            return -13;
        }
    

        
        result = dmp_set_shake_reject_time(40);
        if(result){
            return -14;
        }
    
        
        result = dmp_set_shake_reject_timeout(10);
        if(result){
            return -15;
        }
    
        
    } else {
        tmp[0] = 0xD8;
        result = mpu_write_mem(CFG_20, 1, tmp);
        if(result){
            return -16;
        }
    
        
    }

    if (mask & DMP_FEATURE_ANDROID_ORIENT) {
        tmp[0] = 0xD9;
    } else
        tmp[0] = 0xD8;

    result = mpu_write_mem(CFG_ANDROID_ORIENT_INT, 1, tmp);
    if(result){
        return -17;
    }
    
    

    if (mask & DMP_FEATURE_LP_QUAT){
        result = dmp_enable_lp_quat(1);
        if(result){
            return -18;
        }
    
    }
        
    else{
        result = dmp_enable_lp_quat(0);
        if(result){
            return -19;
        }
    
    }
        

    if (mask & DMP_FEATURE_6X_LP_QUAT){
        result = dmp_enable_6x_lp_quat(1);
        if(result){
            return -20;
        }
    
    }
        
    else{
        result = dmp_enable_6x_lp_quat(0);
        if(result){
            return -21;
        }
        
    }
        

    /* Pedometer is always enabled. */
    dmp.feature_mask = mask | DMP_FEATURE_PEDOMETER;
    result = mpu_reset_fifo();
    if(result){
        return -22;
    }
    
    

    dmp.packet_length = 0;
    if (mask & DMP_FEATURE_SEND_RAW_ACCEL)
        dmp.packet_length += 6;
    if (mask & DMP_FEATURE_SEND_ANY_GYRO)
        dmp.packet_length += 6;
    if (mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT))
        dmp.packet_length += 16;
    if (mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
        dmp.packet_length += 4;

    return 0;
}

/**
 *  @brief      Get list of currently enabled DMP features.
 *  @param[out] Mask of enabled features.
 *  @return     0 if successful.
 */
int dmp_get_enabled_features(unsigned short *mask)
{
    mask[0] = dmp.feature_mask;
    return 0;
}

/**
 *  @brief      Calibrate the gyro data in the DMP.
 *  After eight seconds of no motion, the DMP will compute gyro biases and
 *  subtract them from the quaternion output. If @e dmp_enable_feature is
 *  called with @e DMP_FEATURE_SEND_CAL_GYRO, the biases will also be
 *  subtracted from the gyro output.
 *  @param[in]  enable  1 to enable gyro calibration.
 *  @return     0 if successful.
 */
int dmp_enable_gyro_cal(unsigned char enable)
{
    if (enable) {
        unsigned char regs[9] = {0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d};
        return mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
    } else {
        unsigned char regs[9] = {0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7};
        return mpu_write_mem(CFG_MOTION_BIAS, 9, regs);
    }
}

/**
 *  @brief      Generate 3-axis quaternions from the DMP.
 *  In this driver, the 3-axis and 6-axis DMP quaternion features are mutually
 *  exclusive.
 *  @param[in]  enable  1 to enable 3-axis quaternion.
 *  @return     0 if successful.
 */
int dmp_enable_lp_quat(unsigned char enable)
{
    unsigned char regs[4];
    if (enable) {
        regs[0] = DINBC0;
        regs[1] = DINBC2;
        regs[2] = DINBC4;
        regs[3] = DINBC6;
    }
    else
        memset(regs, 0x8B, 4);

    if(mpu_write_mem(CFG_LP_QUAT, 4, regs)){
        return -1;
    }
    

    return mpu_reset_fifo();
}

/**
 *  @brief       Generate 6-axis quaternions from the DMP.
 *  In this driver, the 3-axis and 6-axis DMP quaternion features are mutually
 *  exclusive.
 *  @param[in]   enable  1 to enable 6-axis quaternion.
 *  @return      0 if successful.
 */
int dmp_enable_6x_lp_quat(unsigned char enable)
{
    unsigned char regs[4];
    if (enable) {
        regs[0] = DINA20;
        regs[1] = DINA28;
        regs[2] = DINA30;
        regs[3] = DINA38;
    } else
        memset(regs, 0xA3, 4);

    if(mpu_write_mem(CFG_8, 4, regs)){
        return -1;
    }
    

    return mpu_reset_fifo();
}

/**
 *  @brief      Decode the four-byte gesture data and execute any callbacks.
 *  @param[in]  gesture Gesture data from DMP packet.
 *  @return     0 if successful.
 */
static int decode_gesture(unsigned char *gesture)
{
    unsigned char tap, android_orient;

    android_orient = gesture[3] & 0xC0;
    tap = 0x3F & gesture[3];

    if (gesture[1] & INT_SRC_TAP) {
        unsigned char direction, count;
        direction = tap >> 3;
        count = (tap % 8) + 1;
        if (dmp.tap_cb)
            dmp.tap_cb(direction, count);
    }

    if (gesture[1] & INT_SRC_ANDROID_ORIENT) {
        if (dmp.android_orient_cb)
            dmp.android_orient_cb(android_orient >> 6);
    }

    return 0;
}

/**
 *  @brief      Specify when a DMP interrupt should occur.
 *  A DMP interrupt can be configured to trigger on either of the two
 *  conditions below:
 *  \n a. One FIFO period has elapsed (set by @e mpu_set_sample_rate).
 *  \n b. A tap event has been detected.
 *  @param[in]  mode    DMP_INT_GESTURE or DMP_INT_CONTINUOUS.
 *  @return     0 if successful.
 */
int dmp_set_interrupt_mode(unsigned char mode)
{
    const unsigned char regs_continuous[11] =
        {0xd8, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0x09, 0xb4, 0xd9};
    const unsigned char regs_gesture[11] =
        {0xda, 0xb1, 0xb9, 0xf3, 0x8b, 0xa3, 0x91, 0xb6, 0xda, 0xb4, 0xda};

    switch (mode) {
    case DMP_INT_CONTINUOUS:
        return mpu_write_mem(CFG_FIFO_ON_EVENT, 11,
            (unsigned char*)regs_continuous);
    case DMP_INT_GESTURE:
        return mpu_write_mem(CFG_FIFO_ON_EVENT, 11,
            (unsigned char*)regs_gesture);
    default:
        return -1;
    }
}

/**
 *  @brief      Get one packet from the FIFO.
 *  If @e sensors does not contain a particular sensor, disregard the data
 *  returned to that pointer.
 *  \n @e sensors can contain a combination of the following flags:
 *  \n INV_X_GYRO, INV_Y_GYRO, INV_Z_GYRO
 *  \n INV_XYZ_GYRO
 *  \n INV_XYZ_ACCEL
 *  \n INV_WXYZ_QUAT
 *  \n If the FIFO has no new data, @e sensors will be zero.
 *  \n If the FIFO is disabled, @e sensors will be zero and this function will
 *  return a non-zero error code.
 *  @param[out] gyro        Gyro data in hardware units.
 *  @param[out] accel       Accel data in hardware units.
 *  @param[out] quat        3-axis quaternion data in hardware units.
 *  @param[out] timestamp   Timestamp in milliseconds.
 *  @param[out] sensors     Mask of sensors read from FIFO.
 *  @param[out] more        Number of remaining packets.
 *  @return     0 if successful.
 */
int dmp_read_fifo(short *gyro, short *accel, long *quat,
    unsigned long *timestamp, short *sensors, unsigned char *more)
{
    unsigned char fifo_data[MAX_PACKET_LENGTH];
    unsigned char ii = 0;

    /* TODO: sensors[0] only changes when dmp_enable_feature is called. We can
     * cache this value and save some cycles.
     */
    sensors[0] = 0;

    /* Get a packet. */
    if (mpu_read_fifo_stream(dmp.packet_length, fifo_data, more))
        return -1;

    /* Parse DMP packet. */
    if (dmp.feature_mask & (DMP_FEATURE_LP_QUAT | DMP_FEATURE_6X_LP_QUAT)) {
#ifdef FIFO_CORRUPTION_CHECK
        long quat_q14[4], quat_mag_sq;
#endif
        quat[0] = ((long)fifo_data[0] << 24) | ((long)fifo_data[1] << 16) |
            ((long)fifo_data[2] << 8) | fifo_data[3];
        quat[1] = ((long)fifo_data[4] << 24) | ((long)fifo_data[5] << 16) |
            ((long)fifo_data[6] << 8) | fifo_data[7];
        quat[2] = ((long)fifo_data[8] << 24) | ((long)fifo_data[9] << 16) |
            ((long)fifo_data[10] << 8) | fifo_data[11];
        quat[3] = ((long)fifo_data[12] << 24) | ((long)fifo_data[13] << 16) |
            ((long)fifo_data[14] << 8) | fifo_data[15];
        ii += 16;
#ifdef FIFO_CORRUPTION_CHECK
        /* We can detect a corrupted FIFO by monitoring the quaternion data and
         * ensuring that the magnitude is always normalized to one. This
         * shouldn't happen in normal operation, but if an I2C error occurs,
         * the FIFO reads might become misaligned.
         *
         * Let's start by scaling down the quaternion data to avoid long long
         * math.
         */
        quat_q14[0] = quat[0] >> 16;
        quat_q14[1] = quat[1] >> 16;
        quat_q14[2] = quat[2] >> 16;
        quat_q14[3] = quat[3] >> 16;
        quat_mag_sq = quat_q14[0] * quat_q14[0] + quat_q14[1] * quat_q14[1] +
            quat_q14[2] * quat_q14[2] + quat_q14[3] * quat_q14[3];
        if ((quat_mag_sq < QUAT_MAG_SQ_MIN) ||
            (quat_mag_sq > QUAT_MAG_SQ_MAX)) {
            /* Quaternion is outside of the acceptable threshold. */
            mpu_reset_fifo();
            sensors[0] = 0;
            return -1;
        }
        sensors[0] |= INV_WXYZ_QUAT;
#endif
    }

    if (dmp.feature_mask & DMP_FEATURE_SEND_RAW_ACCEL) {
        accel[0] = ((short)fifo_data[ii+0] << 8) | fifo_data[ii+1];
        accel[1] = ((short)fifo_data[ii+2] << 8) | fifo_data[ii+3];
        accel[2] = ((short)fifo_data[ii+4] << 8) | fifo_data[ii+5];
        ii += 6;
        sensors[0] |= INV_XYZ_ACCEL;
    }

    if (dmp.feature_mask & DMP_FEATURE_SEND_ANY_GYRO) {
        gyro[0] = ((short)fifo_data[ii+0] << 8) | fifo_data[ii+1];
        gyro[1] = ((short)fifo_data[ii+2] << 8) | fifo_data[ii+3];
        gyro[2] = ((short)fifo_data[ii+4] << 8) | fifo_data[ii+5];
        ii += 6;
        sensors[0] |= INV_XYZ_GYRO;
    }

    /* Gesture data is at the end of the DMP packet. Parse it and call
     * the gesture callbacks (if registered).
     */
    if (dmp.feature_mask & (DMP_FEATURE_TAP | DMP_FEATURE_ANDROID_ORIENT))
        decode_gesture(fifo_data + ii);

    get_ms(timestamp);
    return 0;
}

/**
 *  @brief      Register a function to be executed on a tap event.
 *  The tap direction is represented by one of the following:
 *  \n TAP_X_UP
 *  \n TAP_X_DOWN
 *  \n TAP_Y_UP
 *  \n TAP_Y_DOWN
 *  \n TAP_Z_UP
 *  \n TAP_Z_DOWN
 *  @param[in]  func    Callback function.
 *  @return     0 if successful.
 */
int dmp_register_tap_cb(void (*func)(unsigned char, unsigned char))
{
    dmp.tap_cb = func;
    return 0;
}


/**
 *  @brief      Register a function to be executed on a android orientation event.
 *  @param[in]  func    Callback function.
 *  @return     0 if successful.
 */
int dmp_register_android_orient_cb(void (*func)(unsigned char))
{
    dmp.android_orient_cb = func;
    return 0;
}

/**
 *  @}
 */

/**
 * 获取dmp包长
 */
unsigned char dmp_get_packet_length(){
    return dmp.packet_length;
}



static void tap_cb(unsigned char direction, unsigned char count)
{
}

static void android_orient_cb(unsigned char orientation)
{
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

/**
 * 初始化设备
 */
int init_device(){
    int result = mpu_init();
    if(result){
        Serial.print(F("mpu_init error!"));
        Serial.println(result);
        
        return -1;
    }

    /* Get/set hardware configuration. Start gyro. */
    /* Wake up all sensors. */
    if(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)){
        Serial.print(F("mpu_set_sensors error"));
        
        return -2;
    }
    /* Push both gyro and accel data into the FIFO. */
    if(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)){
        Serial.print(F("mpu_configure_fifo error"));
        
        return -3;
    }
    if(mpu_set_sample_rate(100)){
        Serial.print(F("mpu_set_sample_rate error"));
        
        return -4;
    }
        
        
    /* Read back configuration in case it was set improperly. */
    unsigned char accel_fsr;
    unsigned short gyro_rate, gyro_fsr;
    unsigned long timestamp;
    if(mpu_get_sample_rate(&gyro_rate)){
        Serial.print(F("mpu_get_sample_rate error"));
        
        return -5;
    }
    
    if(mpu_get_gyro_fsr(&gyro_fsr)){
        Serial.print(F("mpu_get_gyro_fsr error"));
        
        return -6;
    }
    
    if(mpu_get_accel_fsr(&accel_fsr)){
        Serial.print(F("mpu_get_accel_fsr error"));
        
        return -7;
    }
    // long gyro[3]={10138, 1570, -74563}, accel[3]={3294, -2827, 2038};
    // mpu_set_gyro_bias_reg(gyro);
    // mpu_set_accel_bias_6500_reg(accel);
        
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
    result = dmp_load_motion_driver_firmware();
    if(result){
            Serial.print(F("dmp_load_motion_driver_firmware error :"));
            Serial.println(result);
            
            return -8;
        }
        
    if(dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation))){
        Serial.print(F("dmp_set_orientation error"));
        
        return -9;
    }
        
    
    if(dmp_register_tap_cb(tap_cb)){
        Serial.print(F("dmp_register_tap_cb error"));
        
        return -10;
    }
        
    if(dmp_register_android_orient_cb(android_orient_cb)){
            Serial.print(F("dmp_register_android_orient_cb error"));
        
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
    // if(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
    //     DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
    //     DMP_FEATURE_GYRO_CAL)){
    //     Serial.print(F("dmp_enable_feature error"));
        
    //     return -12;
    // }
    if(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT)){
        Serial.print(F("dmp_enable_feature error"));
        
        return -12;
    }
    
    
    if(dmp_set_fifo_rate(30)){
            Serial.print(F("dmp_set_fifo_rate error"));
        
        return -13;
    }
        
    if(mpu_set_dmp_state(1)){
            Serial.print(F("mpu_set_dmp_state error"));
        
        return -14;
    }
    
    return 0;
}