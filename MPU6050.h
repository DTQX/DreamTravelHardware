#define BIT_RESET          (0x80)
#define st_hw_addr           (0x68)
#define st_reg_pwr_mgmt_1           (0x6B)
#define st_reg_gyro_cfg           (0x1B)
#define st_reg_accel_cfg           (0x1C)
#define st_reg_lpf           (0x1A)
#define st_reg_rate_div           (0x19)
#define st_reg_int_enable           (0x38)
#define INV_XYZ_COMPASS (0x01)
#define BIT_DATA_RDY_EN     (0x01)
#define BIT_DMP_INT_EN      (0x02)



/* Full scale ranges. */
enum gyro_fsr_e {
    INV_FSR_250DPS = 0,
    INV_FSR_500DPS,     //1
    INV_FSR_1000DPS,    //2
    INV_FSR_2000DPS,    //3
    NUM_GYRO_FSR
};

/* Full scale ranges. */
enum accel_fsr_e {
    INV_FSR_2G = 0,
    INV_FSR_4G,
    INV_FSR_8G,
    INV_FSR_16G,
    NUM_ACCEL_FSR
};

/* Filter configurations. */
enum lpf_e {
    INV_FILTER_256HZ_NOLPF2 = 0,
    INV_FILTER_188HZ,
    INV_FILTER_98HZ,
    INV_FILTER_42HZ,
    INV_FILTER_20HZ,
    INV_FILTER_10HZ,
    INV_FILTER_5HZ,
    INV_FILTER_2100HZ_NOLPF,
    NUM_FILTER
};


int my_mpu_init();

int reset_mpu();

int mpu_configure_fifo(unsigned char sensors);

int set_int_enable(unsigned char enable);

int mpu_set_sample_rate(unsigned short rate);

int mpu_set_lpf(unsigned short lpf);

int mpu_set_accel_fsr(unsigned char fsr);

int mpu_set_gyro_fsr(unsigned short fsr)