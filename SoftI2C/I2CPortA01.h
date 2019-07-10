#define SCL_PIN_PA01 0
#define SCL_PORT_PA01 PORTA
#define SDA_PIN_PA01 1
#define SDA_PORT_PA01 PORTA

// Init function. Needs to be called once in the beginning.
// Returns false if SDA or SCL are low, which probably means 
// a I2C bus lockup or that the lines are not pulled up.
bool __attribute__ ((noinline)) i2c_init_pa01(void) __attribute__ ((used));

// Start transfer function: <addr> is the 8-bit I2C address (including the R/W
// bit). 
// Return: true if the slave replies with an "acknowledge", false otherwise
bool __attribute__ ((noinline)) i2c_start_pa01(uint8_t addr) __attribute__ ((used)); 

// Similar to start function, but wait for an ACK! Will timeout if I2C_MAXWAIT > 0.
bool  __attribute__ ((noinline)) i2c_start_wait_pa01(uint8_t addr) __attribute__ ((used));

// Repeated start function: After having claimed the bus with a start condition,
// you can address another or the same chip again without an intervening 
// stop condition.
// Return: true if the slave replies with an "acknowledge", false otherwise
bool __attribute__ ((noinline)) i2c_rep_start_pa01(uint8_t addr) __attribute__ ((used));

// Issue a stop condition, freeing the bus.
void __attribute__ ((noinline)) i2c_stop_pa01(void) asm("ass_i2c_stop_pa01") __attribute__ ((used));

// Write one byte to the slave chip that had been addressed
// by the previous start call. <value> is the byte to be sent.
// Return: true if the slave replies with an "acknowledge", false otherwise
bool __attribute__ ((noinline)) i2c_write_pa01(uint8_t value) asm("ass_i2c_write_pa01") __attribute__ ((used));


// Read one byte. If <last> is true, we send a NAK after having received 
// the byte in order to terminate the read sequence. 
uint8_t __attribute__ ((noinline)) i2c_read_pa01(bool last) __attribute__ ((used));

// map the IO register back into the IO address space
#define SDA_DDR_PA01       	(_SFR_IO_ADDR(SDA_PORT_PA01) - 1)
#define SCL_DDR_PA01       	(_SFR_IO_ADDR(SCL_PORT_PA01) - 1)
#define SDA_OUT_PA01       	_SFR_IO_ADDR(SDA_PORT_PA01)
#define SCL_OUT_PA01       	_SFR_IO_ADDR(SCL_PORT_PA01)
#define SDA_IN_PA01		(_SFR_IO_ADDR(SDA_PORT_PA01) - 2)
#define SCL_IN_PA01		(_SFR_IO_ADDR(SCL_PORT_PA01) - 2)

#ifndef __tmp_reg__
#define __tmp_reg__ 0
#endif
 
// Internal delay functions.
void __attribute__ ((noinline)) i2c_delay_half_pa01(void) asm("ass_i2c_delay_half_pa01")  __attribute__ ((used));
void __attribute__ ((noinline)) i2c_wait_scl_high_pa01(void) asm("ass_i2c_wait_scl_high_pa01")  __attribute__ ((used));
