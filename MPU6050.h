
#include <inttypes.h>

struct gyro_reg_s {
    unsigned char who_am_i;
    unsigned char rate_div;
    unsigned char lpf;
    unsigned char prod_id;
    unsigned char user_ctrl;
    unsigned char fifo_en;
    unsigned char gyro_cfg;
    unsigned char accel_cfg;
    unsigned char accel_cfg2;
    unsigned char lp_accel_odr;
    unsigned char motion_thr;
    unsigned char motion_dur;
    unsigned char fifo_count_h;
    unsigned char fifo_r_w;
    unsigned char raw_gyro;
    unsigned char raw_accel;
    unsigned char temp;
    unsigned char int_enable;
    unsigned char dmp_int_status;
    unsigned char int_status;
    unsigned char accel_intel;
    unsigned char pwr_mgmt_1;
    unsigned char pwr_mgmt_2;
    unsigned char int_pin_cfg;
    unsigned char mem_r_w;
    unsigned char accel_offs;
    unsigned char i2c_mst;
    unsigned char bank_sel;
    unsigned char mem_start_addr;
    unsigned char prgm_start_h;
#if defined AK89xx_SECONDARY
    unsigned char s0_addr;
    unsigned char s0_reg;
    unsigned char s0_ctrl;
    unsigned char s1_addr;
    unsigned char s1_reg;
    unsigned char s1_ctrl;
    unsigned char s4_ctrl;
    unsigned char s0_do;
    unsigned char s1_do;
    unsigned char i2c_delay_ctrl;
    unsigned char raw_compass;
    /* The I2C_MST_VDDIO bit is in this register. */
    unsigned char yg_offs_tc;
#endif
};
#ifdef __cplusplus 
extern "C" {
#endif


unsigned char addc();


#ifdef __cplusplus 
}
#endif