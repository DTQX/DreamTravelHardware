#define SCL_PIN_PC45 4
#define SCL_PORT_PC45 PORTC
#define SDA_PIN_PC45 5
#define SDA_PORT_PC45 PORTC

// Init function. Needs to be called once in the beginning.
// Returns false if SDA or SCL are low, which probably means 
// a I2C bus lockup or that the lines are not pulled up.
bool __attribute__ ((noinline)) i2c_init_pc45(void) __attribute__ ((used));

// Start transfer function: <addr> is the 8-bit I2C address (including the R/W
// bit). 
// Return: true if the slave replies with an "acknowledge", false otherwise
bool __attribute__ ((noinline)) i2c_start_pc45(uint8_t addr) __attribute__ ((used)); 

// Similar to start function, but wait for an ACK! Will timeout if I2C_MAXWAIT > 0.
bool  __attribute__ ((noinline)) i2c_start_wait_pc45(uint8_t addr) __attribute__ ((used));

// Repeated start function: After having claimed the bus with a start condition,
// you can address another or the same chip again without an intervening 
// stop condition.
// Return: true if the slave replies with an "acknowledge", false otherwise
bool __attribute__ ((noinline)) i2c_rep_start_pc45(uint8_t addr) __attribute__ ((used));

// Issue a stop condition, freeing the bus.
void __attribute__ ((noinline)) i2c_stop_pc45(void) asm("ass_i2c_stop_pc45") __attribute__ ((used));

// Write one byte to the slave chip that had been addressed
// by the previous start call. <value> is the byte to be sent.
// Return: true if the slave replies with an "acknowledge", false otherwise
bool __attribute__ ((noinline)) i2c_write_pc45(uint8_t value) asm("ass_i2c_write_pc45") __attribute__ ((used));


// Read one byte. If <last> is true, we send a NAK after having received 
// the byte in order to terminate the read sequence. 
uint8_t __attribute__ ((noinline)) i2c_read_pc45(bool last) __attribute__ ((used));

#if !I2C_HARDWARE
// map the IO register back into the IO address space
#define SDA_DDR_PC45       	(_SFR_IO_ADDR(SDA_PORT_PC45) - 1)
#define SCL_DDR_PC45       	(_SFR_IO_ADDR(SCL_PORT_PC45) - 1)
#define SDA_OUT_PC45       	_SFR_IO_ADDR(SDA_PORT_PC45)
#define SCL_OUT_PC45       	_SFR_IO_ADDR(SCL_PORT_PC45)
#define SDA_IN_PC45		(_SFR_IO_ADDR(SDA_PORT_PC45) - 2)
#define SCL_IN_PC45		(_SFR_IO_ADDR(SCL_PORT_PC45) - 2)

#ifndef __tmp_reg__
#define __tmp_reg__ 0
#endif
 
// Internal delay functions.
void __attribute__ ((noinline)) i2c_delay_half_pc45(void) asm("ass_i2c_delay_half_pc45")  __attribute__ ((used));
void __attribute__ ((noinline)) i2c_wait_scl_high_pc45(void) asm("ass_i2c_wait_scl_high_pc45")  __attribute__ ((used));

void  i2c_delay_half_pc45(void)
{ // function call 3 cycles => 3C
#if I2C_DELAY_COUNTER < 1
  __asm__ __volatile__ (" ret");
  // 7 cycles for call and return
#else
  __asm__ __volatile__ 
    (
     " ldi      r25, %[DELAY]           ;load delay constant   ;; 4C \n\t"
     "_Lidelay_pc45: \n\t"
     " dec r25                          ;decrement counter     ;; 4C+xC \n\t"
     " brne _Lidelay_pc45                                           ;;5C+(x-1)2C+xC\n\t"
     " ret                                                     ;; 9C+(x-1)2C+xC = 7C+xC" 
     : : [DELAY] "M" I2C_DELAY_COUNTER : "r25");
  // 7 cycles + 3 times x cycles
#endif
}

void i2c_wait_scl_high_pc45(void)
{
#if I2C_TIMEOUT <= 0
  __asm__ __volatile__ 
    ("_Li2c_wait_stretch_pc45: \n\t"
     " sbis	%[SCLIN],%[SCLPIN]	;wait for SCL high \n\t" 
     " rjmp	_Li2c_wait_stretch_pc45 \n\t"
     " cln                              ;signal: no timeout \n\t"
     " ret "
     : : [SCLIN] "I" (SCL_IN_PC45), [SCLPIN] "I" (SCL_PIN_PC45));
#else
  __asm__ __volatile__ 
    ( " ldi     r27, %[HISTRETCH]       ;load delay counter \n\t"
      " ldi     r26, %[LOSTRETCH] \n\t"
      "_Lwait_stretch: \n\t"
      " clr     __tmp_reg__             ;do next loop 255 times \n\t"
      "_Lwait_stretch_inner_loop: \n\t"
      " rcall   _Lcheck_scl_level       ;call check function   ;; 12C \n\t"
      " brpl    _Lstretch_done          ;done if N=0           ;; +1 = 13C\n\t"
      " dec     __tmp_reg__             ;dec inner loop counter;; +1 = 14C\n\t"
      " brne    _Lwait_stretch_inner_loop                      ;; +2 = 16C\n\t"
      " sbiw    r26,1                   ;dec outer loop counter \n\t"
      " brne    _Lwait_stretch          ;continue with outer loop \n\t"
      " sen                             ;timeout -> set N-bit=1 \n\t"
      " rjmp _Lwait_return              ;and return with N=1\n\t"
      "_Lstretch_done:                  ;SCL=1 sensed \n\t"            
      " cln                             ;OK -> clear N-bit \n\t"
      " rjmp _Lwait_return              ; and return with N=0 \n\t"

      "_Lcheck_scl_level:                                      ;; call = 3C\n\t"
      " cln                                                    ;; +1C = 4C \n\t"
      " sbic	%[SCLIN],%[SCLPIN]      ;skip if SCL still low ;; +2C = 6C \n\t"
      " rjmp    _Lscl_high                                     ;; +0C = 6C \n\t"
      " sen                                                    ;; +1 = 7C\n\t "
      "_Lscl_high: "
      " nop                                                    ;; +1C = 8C \n\t"
      " ret                             ;return N-Bit=1 if low ;; +4 = 12C\n\t"

      "_Lwait_return:"
      : : [SCLIN] "I" (SCL_IN_PC45), [SCLPIN] "I" (SCL_PIN_PC45), 
	[HISTRETCH] "M" (I2C_MAX_STRETCH>>8), 
	[LOSTRETCH] "M" (I2C_MAX_STRETCH&0xFF)
      : "r26", "r27");
#endif
}
#endif // !I2C_HARDWARE

// Initialize the I2C system. Must be called once in setup. Will return false if SDA or SCL is on a low level, which means that the bus is locked. Otherwise returns true.
bool i2c_init_pc45(void)
#if I2C_HARDWARE
{
#if I2C_PULLUP
  digitalWrite(SDA, 1);
  digitalWrite(SCL, 1);
#else
  digitalWrite(SDA, 0);
  digitalWrite(SCL, 0);
#endif
#if ((I2C_CPUFREQ/SCL_CLOCK)-16)/2 < 250
  TWSR = 0;                         /* no prescaler */
  TWBR = ((I2C_CPUFREQ/SCL_CLOCK)-16)/2;  /* must be > 10 for stable operation */
#else
  TWSR = (1<<TWPS0); // prescaler is 4
  TWBR = ((I2C_CPUFREQ/SCL_CLOCK)-16)/8;
#endif
  return (digitalRead(SDA) != 0 && digitalRead(SCL) != 0);
}
#else
{
  __asm__ __volatile__ 
    (" cbi      %[SDADDR],%[SDAPIN]     ;release SDA \n\t" 
     " cbi      %[SCLDDR],%[SCLPIN]     ;release SCL \n\t" 
#if I2C_PULLUP
     " sbi      %[SDAOUT],%[SDAPIN]     ;enable SDA pull-up\n\t"
#else
     " cbi      %[SDAOUT],%[SDAPIN]     ;clear SDA output value \n\t"     
#endif
#if I2C_PULLUP
     " sbi      %[SCLOUT],%[SCLPIN]     ;enable SCL pull-up\n\t"
#else
     " cbi      %[SCLOUT],%[SCLPIN]     ;clear SCL output value \n\t"
#endif
     " clr      r24                     ;set return value to false \n\t"
     " clr      r25                     ;set return value to false \n\t"
     " sbis     %[SDAIN],%[SDAPIN]      ;check for SDA high\n\t"
     " ret                              ;if low return with false \n\t"  
     " sbis     %[SCLIN],%[SCLPIN]      ;check for SCL high \n\t"
     " ret                              ;if low return with false \n\t" 
     " ldi      r24,1                   ;set return value to true \n\t"
     " ret "
     : :
       [SCLDDR] "I"  (SCL_DDR_PC45), [SCLPIN] "I" (SCL_PIN_PC45), 
       [SCLIN] "I" (SCL_IN_PC45), [SCLOUT] "I" (SCL_OUT_PC45),
       [SDADDR] "I"  (SDA_DDR_PC45), [SDAPIN] "I" (SDA_PIN_PC45), 
       [SDAIN] "I" (SDA_IN_PC45), [SDAOUT] "I" (SDA_OUT_PC45)); 
  return true;
}
#endif

// 如果SCL为低电平，并不会直接返回，会一直等SCL变高
// Initiates a transfer to the slave device with the 8-bit I2C address addr. Note that this library uses the 8-bit addressing scheme different from the 7-bit scheme in the Wire library. In addition the least significant bit of addr must be specified as I2C_WRITE (=0) or I2C_READ (=1). Returns true if the addressed device replies with an ACK. Otherwise false is returned.
bool  i2c_start_pc45(uint8_t addr)
#if I2C_HARDWARE
{
  uint8_t   twst;
#if I2C_TIMEOUT
  uint32_t start = millis();
#endif

  // send START condition
  TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);

  // wait until transmission completed
  while(!(TWCR & (1<<TWINT))) {
#if I2C_TIMEOUT
    if (millis() - start > I2C_TIMEOUT) return false;
#endif
  }

  // check value of TWI Status Register. Mask prescaler bits.
  twst = TW_STATUS & 0xF8;
  if ( (twst != TW_START) && (twst != TW_REP_START)) return false;
  
  // send device address
  TWDR = addr;
  TWCR = (1<<TWINT) | (1<<TWEN);
  
  // wail until transmission completed and ACK/NACK has been received
  while(!(TWCR & (1<<TWINT))) {
#if I2C_TIMEOUT
    if (millis() - start > I2C_TIMEOUT) return false;
#endif
  }
  
  // check value of TWI Status Register. Mask prescaler bits.
  twst = TW_STATUS & 0xF8;
  if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return false;
  
  return true;
}
#else
{
  __asm__ __volatile__ 
    (
#if I2C_NOINTERRUPT
     " cli                              ;clear IRQ bit \n\t"
#endif
     // 一直等待，直到scl变高
     " sbis     %[SCLIN],%[SCLPIN]      ;check for clock stretching slave\n\t"
     " rcall    ass_i2c_wait_scl_high_pc45   ;wait until SCL=H\n\t"
     // 不等待，scl为低直接返回
     // " clr      r24                     ;set return value to false \n\t"
     // " sbis     %[SCLIN],%[SCLPIN]      ;check for SCL high \n\t"
     // " ret                              ;if low return with false \n\t" 
     // " ldi      r24,1                   ;set return value to true \n\t"
     // end
#if I2C_PULLUP
     " cbi      %[SDAOUT],%[SDAPIN]     ;disable pull-up \n\t"
#endif
     " sbi      %[SDADDR],%[SDAPIN]     ;force SDA low  \n\t" 
     " rcall    ass_i2c_delay_half_pc45      ;wait T/2 \n\t"
     " rcall    ass_i2c_write_pc45           ;now write address \n\t"
     " ret"
     : : [SDADDR] "I"  (SDA_DDR_PC45), [SDAPIN] "I" (SDA_PIN_PC45),
       [SDAOUT] "I" (SDA_OUT_PC45), [SCLOUT] "I" (SCL_OUT_PC45),
       [SCLIN] "I" (SCL_IN_PC45),[SCLPIN] "I" (SCL_PIN_PC45)); 
  return true; // we never return here!
}
#endif

bool  i2c_rep_start_pc45(uint8_t addr)
#if I2C_HARDWARE
{
  return i2c_start_pc45(addr);
}
#else
{
  __asm__ __volatile__ 

    (
#if I2C_NOINTERRUPT
     " cli \n\t"
#endif
#if I2C_PULLUP
     " cbi      %[SCLOUT],%[SCLPIN]     ;disable SCL pull-up \n\t"
#endif
     " sbi	%[SCLDDR],%[SCLPIN]	;force SCL low \n\t" 
     " rcall 	ass_i2c_delay_half_pc45	;delay  T/2 \n\t" 
     " cbi	%[SDADDR],%[SDAPIN]	;release SDA \n\t"
#if I2C_PULLUP
     " sbi      %[SDAOUT],%[SDAPIN]     ;enable SDA pull-up \n\t"
#endif
     " rcall	ass_i2c_delay_half_pc45	;delay T/2 \n\t" 
     " cbi	%[SCLDDR],%[SCLPIN]	;release SCL \n\t"
#if I2C_PULLUP
     " sbi      %[SCLOUT],%[SCLPIN]	;enable SCL pull-up \n\t"
#endif
     " rcall 	ass_i2c_delay_half_pc45	;delay  T/2 \n\t" 
     " sbis     %[SCLIN],%[SCLPIN]      ;check for clock stretching slave\n\t"
     " rcall    ass_i2c_wait_scl_high_pc45   ;wait until SCL=H\n\t"
#if I2C_PULLUP
     " cbi 	%[SDAOUT],%[SDAPIN]	;disable SDA pull-up\n\t"
#endif
     " sbi 	%[SDADDR],%[SDAPIN]	;force SDA low \n\t"
     " rcall 	ass_i2c_delay_half_pc45	;delay	T/2 \n\t" 
     " rcall    ass_i2c_write_pc45       \n\t"
     " ret"
     : : [SCLDDR] "I"  (SCL_DDR_PC45), [SCLPIN] "I" (SCL_PIN_PC45),
       [SCLIN] "I" (SCL_IN_PC45), [SCLOUT] "I" (SCL_OUT_PC45), [SDAOUT] "I" (SDA_OUT_PC45),
       [SDADDR] "I"  (SDA_DDR_PC45), [SDAPIN] "I" (SDA_PIN_PC45)); 
  return true; // just to fool the compiler
}
#endif

bool  i2c_start_wait_pc45(uint8_t addr)
#if I2C_HARDWARE
{
  uint8_t   twst;
  uint16_t maxwait = I2C_MAXWAIT;
#if I2C_TIMEOUT
  uint32_t start = millis();
#endif
  
  while (true) {
    // send START condition
    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
    
    // wait until transmission completed
    while(!(TWCR & (1<<TWINT))) {
#if I2C_TIMEOUT
    if (millis() - start > I2C_TIMEOUT) return false;
#endif
    }
    
    // check value of TWI Status Register. Mask prescaler bits.
    twst = TW_STATUS & 0xF8;
    if ( (twst != TW_START) && (twst != TW_REP_START)) continue;
    
    // send device address
    TWDR = addr;
    TWCR = (1<<TWINT) | (1<<TWEN);
    
    // wail until transmission completed
    while(!(TWCR & (1<<TWINT))) {
#if I2C_TIMEOUT
      if (millis() - start > I2C_TIMEOUT) return false;
#endif
    }
    
    // check value of TWI Status Register. Mask prescaler bits.
    twst = TW_STATUS & 0xF8;
    if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ) 
      {    	    
	/* device busy, send stop condition to terminate write operation */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while(TWCR & (1<<TWSTO)) {
#if I2C_TIMEOUT
	  if (millis() - start > I2C_TIMEOUT) return false;
#endif
	}

	if (maxwait)
	  if (--maxwait == 0)
	    return false;
	
	continue;
      }
    //if( twst != TW_MT_SLA_ACK) return 1;
    return true;
  }
}
#else
{
 __asm__ __volatile__ 
   (
    " push	r24                     ;save original parameter \n\t"
#if I2C_MAXWAIT
    " ldi     r31, %[HIMAXWAIT]         ;load max wait counter \n\t"
    " ldi     r30, %[LOMAXWAIT]         ;load low byte \n\t"
#endif
    "_Li2c_start_wait1_pc45: \n\t"
    " pop       r24                     ;restore original parameter\n\t"
    " push      r24                     ;and save again \n\t"
#if I2C_NOINTERRUPT
    " cli                               ;disable interrupts \n\t"
#endif
    " sbis     %[SCLIN],%[SCLPIN]      ;check for clock stretching slave\n\t"
    " rcall    ass_i2c_wait_scl_high_pc45   ;wait until SCL=H\n\t" 
#if I2C_PULLUP
     " cbi      %[SDAOUT],%[SDAPIN]     ;disable pull-up \n\t"
#endif
    " sbi 	%[SDADDR],%[SDAPIN]	;force SDA low \n\t" 
    " rcall 	ass_i2c_delay_half_pc45	;delay T/2 \n\t" 
    " rcall 	ass_i2c_write_pc45	        ;write address \n\t" 
    " tst	r24		        ;if device not busy -> done \n\t" 
    " brne	_Li2c_start_wait_done_pc45 \n\t" 
    " rcall	ass_i2c_stop_pc45	        ;terminate write & enable IRQ \n\t"
#if I2C_MAXWAIT
    " sbiw      r30,1                   ;decrement max wait counter\n\t"
    " breq       _Li2c_start_wait_done_pc45  ;if zero reached, exit with false -> r24 already zero!\n\t"
#endif
    " rjmp	_Li2c_start_wait1_pc45	;device busy, poll ack again \n\t" 
    "_Li2c_start_wait_done_pc45: \n\t"
    " clr       r25                     ;clear high byte of return value\n\t"
    " pop       __tmp_reg__             ;pop off orig argument \n\t"
    " ret "
    : : [SDADDR] "I"  (SDA_DDR_PC45), [SDAPIN] "I" (SDA_PIN_PC45), [SDAOUT] "I" (SDA_OUT_PC45),
      [SCLIN] "I" (SCL_IN_PC45), [SCLPIN] "I" (SCL_PIN_PC45),
      [HIMAXWAIT] "M" (I2C_MAXWAIT>>8), 
      [LOMAXWAIT] "M" (I2C_MAXWAIT&0xFF)
    : "r30", "r31" ); 
}
#endif

void  i2c_stop_pc45(void)
#if I2C_HARDWARE
{
#if I2C_TIMEOUT
  uint32_t start = millis();
#endif
  /* send stop condition */
  TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
  
  // wait until stop condition is executed and bus released
  while(TWCR & (1<<TWSTO)) {
#if I2C_TIMEOUT
    if (millis() - start > I2C_TIMEOUT) return;
#endif
  }
}
#else
{
  __asm__ __volatile__ 
    (
#if I2C_PULLUP
     " cbi      %[SCLOUT],%[SCLPIN]     ;disable SCL pull-up \n\t"
#endif
     " sbi      %[SCLDDR],%[SCLPIN]     ;force SCL low \n\t"
#if I2C_PULLUP
     " cbi      %[SDAOUT],%[SDAPIN]     ;disable pull-up \n\t"
#endif     
     " sbi      %[SDADDR],%[SDAPIN]     ;force SDA low \n\t" 
     " rcall    ass_i2c_delay_half_pc45      ;T/2 delay \n\t"
     " cbi      %[SCLDDR],%[SCLPIN]     ;release SCL \n\t" 
#if I2C_PULLUP
     " sbi      %[SCLOUT],%[SCLPIN]	;enable SCL pull-up \n\t"
#endif
     " rcall    ass_i2c_delay_half_pc45      ;T/2 delay \n\t"
     " sbis     %[SCLIN],%[SCLPIN]      ;check for clock stretching slave\n\t"
     " rcall    ass_i2c_wait_scl_high_pc45   ;wait until SCL=H\n\t" 
     " cbi      %[SDADDR],%[SDAPIN]     ;release SDA \n\t"
#if I2C_PULLUP
     " sbi      %[SDAOUT],%[SDAPIN]     ;enable SDA pull-up \n\t"
#endif
     " rcall    ass_i2c_delay_half_pc45 \n\t"
#if I2C_NOINTERRUPT
     " sei                              ;enable interrupts again!\n\t"
#endif
     : : [SCLDDR] "I"  (SCL_DDR_PC45), [SCLPIN] "I" (SCL_PIN_PC45), [SCLIN] "I" (SCL_IN_PC45),
       [SDAOUT] "I" (SDA_OUT_PC45), [SCLOUT] "I" (SCL_OUT_PC45),
       [SDADDR] "I"  (SDA_DDR_PC45), [SDAPIN] "I" (SDA_PIN_PC45)); 
}
#endif

// Sends a byte to the previously addressed device. Returns true if the device replies with an ACK, otherwise false.
bool i2c_write_pc45(uint8_t value)
#if I2C_HARDWARE
{	
  uint8_t   twst;
#if I2C_TIMEOUT
  uint32_t start = millis();
#endif

    
  // send data to the previously addressed device
  TWDR = value;
  TWCR = (1<<TWINT) | (1<<TWEN);
  
  // wait until transmission completed
  while(!(TWCR & (1<<TWINT))) {
#if I2C_TIMEOUT
    if (millis() - start > I2C_TIMEOUT) return false;
#endif
  }
  
  // check value of TWI Status Register. Mask prescaler bits
  twst = TW_STATUS & 0xF8;
  if( twst != TW_MT_DATA_ACK) return false;
  return true;
}
#else
{
  __asm__ __volatile__ 
    (
     " sec                              ;set carry flag \n\t"
     " rol      r24                     ;shift in carry and shift out MSB \n\t"
     " rjmp _Li2c_write_first_pc45 \n\t"
     "_Li2c_write_bit_pc45:\n\t"
     " lsl      r24                     ;left shift into carry ;; 1C\n\t"
     "_Li2c_write_first_pc45:\n\t"
     " breq     _Li2c_get_ack_pc45           ;jump if TXreg is empty;; +1 = 2C \n\t"
#if I2C_PULLUP
     " cbi      %[SCLOUT],%[SCLPIN]     ;disable SCL pull-up \n\t"
#endif
     " sbi      %[SCLDDR],%[SCLPIN]     ;force SCL low         ;; +2 = 4C \n\t"
     " nop \n\t"
     " nop \n\t"
     " nop \n\t"
     " brcc     _Li2c_write_low_pc45                                ;;+1/+2=5/6C\n\t"
     " nop                                                     ;; +1 = 7C \n\t"
     " cbi %[SDADDR],%[SDAPIN]	        ;release SDA           ;; +2 = 9C \n\t"
#if I2C_PULLUP
     " sbi      %[SDAOUT],%[SDAPIN]     ;enable SDA pull-up \n\t"
#endif
     " rjmp      _Li2c_write_high_pc45                              ;; +2 = 11C \n\t"
     "_Li2c_write_low_pc45: \n\t"
#if I2C_PULLUP
     " cbi      %[SDAOUT],%[SDAPIN]     ;disable pull-up \n\t"
#endif
     " sbi	%[SDADDR],%[SDAPIN]	;force SDA low         ;; +2 = 9C \n\t"
     " rjmp	_Li2c_write_high_pc45                               ;;+2 = 11C \n\t"
     "_Li2c_write_high_pc45: \n\t"
#if I2C_DELAY_COUNTER >= 1
     " rcall 	ass_i2c_delay_half_pc45	;delay T/2             ;;+X = 11C+X\n\t"
#endif
     " cbi	%[SCLDDR],%[SCLPIN]	;release SCL           ;;+2 = 13C+X\n\t"
#if I2C_PULLUP
     " sbi      %[SCLOUT],%[SCLPIN]	;enable SCL pull-up \n\t"
#endif
     " cln                              ;clear N-bit           ;;+1 = 14C+X\n\t"
     " nop \n\t"
     " nop \n\t"
     " nop \n\t"
     " sbis	%[SCLIN],%[SCLPIN]	;check for SCL high    ;;+2 = 16C+X\n\t"
     " rcall    ass_i2c_wait_scl_high_pc45 \n\t"
     " brpl     _Ldelay_scl_high_pc45                              ;;+2 = 18C+X\n\t"
     "_Li2c_write_return_false_pc45: \n\t"
     " clr      r24                     ; return false because of timeout \n\t"
     " rjmp     _Li2c_write_return_pc45 \n\t"
     "_Ldelay_scl_high_pc45: \n\t"
#if I2C_DELAY_COUNTER >= 1
     " rcall	ass_i2c_delay_half_pc45	;delay T/2             ;;+X= 18C+2X\n\t"
#endif
     " rjmp	_Li2c_write_bit_pc45 \n\t"
     "              ;; +2 = 20C +2X for one bit-loop \n\t"
     "_Li2c_get_ack_pc45: \n\t"
#if I2C_PULLUP
     " cbi      %[SCLOUT],%[SCLPIN]     ;disable SCL pull-up \n\t"
#endif
     " sbi	%[SCLDDR],%[SCLPIN]	;force SCL low ;; +2 = 5C \n\t"
     " nop \n\t"
     " nop \n\t"
     " cbi	%[SDADDR],%[SDAPIN]	;release SDA ;;+2 = 7C \n\t"
#if I2C_PULLUP
     " sbi      %[SDAOUT],%[SDAPIN]     ;enable SDA pull-up \n\t"
#endif
#if I2C_DELAY_COUNTER >= 1
     " rcall	ass_i2c_delay_half_pc45	;delay T/2 ;; +X = 7C+X \n\t"
#endif
     " clr	r25                                            ;; 17C+2X \n\t"
     " clr	r24		        ;return 0              ;; 14C + X \n\t"
     " cbi	%[SCLDDR],%[SCLPIN]	;release SCL ;; +2 = 9C+X\n\t"
#if I2C_PULLUP
     " sbi      %[SCLOUT],%[SCLPIN]	;enable SCL pull-up \n\t"
#endif
     "_Li2c_ack_wait_pc45: \n\t"
     " cln                              ; clear N-bit          ;; 10C + X\n\t" 
     " nop \n\t"
     " sbis	%[SCLIN],%[SCLPIN]	;wait SCL high         ;; 12C + X \n\t"
     " rcall    ass_i2c_wait_scl_high_pc45 \n\t"
     " brmi     _Li2c_write_return_false_pc45                       ;; 13C + X \n\t "
     " sbis	%[SDAIN],%[SDAPIN]      ;if SDA hi -> return 0 ;; 15C + X \n\t"
     " ldi	r24,1                   ;return true           ;; 16C + X \n\t"
#if I2C_DELAY_COUNTER >= 1
     " rcall	ass_i2c_delay_half_pc45	;delay T/2             ;; 16C + 2X \n\t"
#endif
     "_Li2c_write_return_pc45: \n\t"
     " nop \n\t "
     " nop \n\t "
#if I2C_PULLUP
     " cbi      %[SCLOUT],%[SCLPIN]     ;disable SCL pull-up \n\t"
#endif
     " sbi	%[SCLDDR],%[SCLPIN]	;force SCL low so SCL=H is short\n\t"
     " ret \n\t"
     "              ;; + 4 = 17C + 2X for acknowldge bit"
     ::
      [SCLDDR] "I"  (SCL_DDR_PC45), [SCLPIN] "I" (SCL_PIN_PC45), [SCLIN] "I" (SCL_IN_PC45),
      [SDAOUT] "I" (SDA_OUT_PC45), [SCLOUT] "I" (SCL_OUT_PC45),
      [SDADDR] "I"  (SDA_DDR_PC45), [SDAPIN] "I" (SDA_PIN_PC45), [SDAIN] "I" (SDA_IN_PC45)); 
  return true; // fooling the compiler
}
#endif

uint8_t i2c_read_pc45(bool last)
#if I2C_HARDWARE
{
#if I2C_TIMEOUT
  uint32_t start = millis();
#endif
    
  TWCR = (1<<TWINT) | (1<<TWEN) | (last ? 0 : (1<<TWEA));
  while(!(TWCR & (1<<TWINT))) {
#if I2C_TIMEOUT
    if (millis() - start > I2C_TIMEOUT) return 0xFF;
#endif
  }  
  return TWDR;
}
#else
{
  __asm__ __volatile__ 
    (
     " ldi	r23,0x01 \n\t"
     "_Li2c_read_bit_pc45: \n\t"
#if I2C_PULLUP
     " cbi      %[SCLOUT],%[SCLPIN]     ;disable SCL pull-up \n\t"
#endif
     " sbi	%[SCLDDR],%[SCLPIN]	;force SCL low         ;; 2C \n\t" 
     " cbi	%[SDADDR],%[SDAPIN]	;release SDA(prev. ACK);; 4C \n\t"
#if I2C_PULLUP
     " sbi      %[SDAOUT],%[SDAPIN]     ;enable SDA pull-up \n\t"
#endif
     " nop \n\t"
     " nop \n\t"
     " nop \n\t"
#if I2C_DELAY_COUNTER >= 1
     " rcall	ass_i2c_delay_half_pc45	;delay T/2             ;; 4C+X \n\t" 
#endif
     " cbi	%[SCLDDR],%[SCLPIN]	;release SCL           ;; 6C + X \n\t" 
#if I2C_PULLUP
     " sbi      %[SCLOUT],%[SCLPIN]	;enable SCL pull-up \n\t"
#endif
#if I2C_DELAY_COUNTER >= 1
     " rcall	ass_i2c_delay_half_pc45	;delay T/2             ;; 6C + 2X \n\t" 
#endif
     " cln                              ; clear N-bit          ;; 7C + 2X \n\t"
     " nop \n\t "
     " nop \n\t "
     " nop \n\t "
     " sbis     %[SCLIN], %[SCLPIN]     ;check for SCL high    ;; 9C +2X \n\t" 
     " rcall    ass_i2c_wait_scl_high_pc45 \n\t"
     " brmi     _Li2c_read_return_pc45       ;return if timeout     ;; 10C + 2X\n\t"
     " clc		  	        ;clear carry flag      ;; 11C + 2X\n\t" 
     " sbic	%[SDAIN],%[SDAPIN]	;if SDA is high        ;; 11C + 2X\n\t" 
     " sec			        ;set carry flag        ;; 12C + 2X\n\t" 
     " rol	r23		        ;store bit             ;; 13C + 2X\n\t" 
     " brcc	_Li2c_read_bit_pc45	        ;while receiv reg not full \n\t"
     "                         ;; 15C + 2X for one bit loop \n\t" 
     
     "_Li2c_put_ack_pc45: \n\t"
#if I2C_PULLUP
     " cbi      %[SCLOUT],%[SCLPIN]     ;disable SCL pull-up \n\t"
#endif
     " sbi	%[SCLDDR],%[SCLPIN]	;force SCL low         ;; 2C \n\t" 
     " cpi	r24,0                                          ;; 3C \n\t" 
     " breq	_Li2c_put_ack_low_pc45	;if (ack=0) ;; 5C \n\t" 
     " cbi	%[SDADDR],%[SDAPIN]	;release SDA \n\t" 
#if I2C_PULLUP
     " sbi      %[SDAOUT],%[SDAPIN]     ;enable SDA pull-up \n\t"
#endif
     " rjmp	_Li2c_put_ack_high_pc45 \n\t" 
     "_Li2c_put_ack_low_pc45:                ;else \n\t" 
#if I2C_PULLUP
     " cbi      %[SDAOUT],%[SDAPIN]     ;disable pull-up \n\t"
#endif
     " sbi	%[SDADDR],%[SDAPIN]	;force SDA low         ;; 7C \n\t" 
     "_Li2c_put_ack_high_pc45: \n\t" 
     " nop \n\t "
     " nop \n\t "
     " nop \n\t "
#if I2C_DELAY_COUNTER >= 1
     " rcall	ass_i2c_delay_half_pc45	;delay T/2             ;; 7C + X \n\t" 
#endif
     " cbi	%[SCLDDR],%[SCLPIN]	;release SCL           ;; 9C +X \n\t" 
#if I2C_PULLUP
     " sbi      %[SCLOUT],%[SCLPIN]	;enable SCL pull-up \n\t"
#endif
     " cln                              ;clear N               ;; +1 = 10C\n\t"
     " nop \n\t "
     " nop \n\t "
     " sbis	%[SCLIN],%[SCLPIN]	;wait SCL high         ;; 12C + X\n\t" 
     " rcall    ass_i2c_wait_scl_high_pc45 \n\t"
#if I2C_DELAY_COUNTER >= 1
     " rcall	ass_i2c_delay_half_pc45	;delay T/2             ;; 11C + 2X\n\t" 
#endif
     "_Li2c_read_return_pc45: \n\t"
     " nop \n\t "
     " nop \n\t "
#if I2C_PULLUP
     " cbi      %[SCLOUT],%[SCLPIN]     ;disable SCL pull-up \n\t"
#endif
     "sbi	%[SCLDDR],%[SCLPIN]	;force SCL low so SCL=H is short\n\t"
     " mov	r24,r23                                        ;; 12C + 2X \n\t"
     " clr	r25                                            ;; 13 C + 2X\n\t"
     " ret                                                     ;; 17C + X"
     ::
     // I 代表数据处理指令中的立即数，[L1] means the memory contents at address L1
      [SCLDDR] "I"  (SCL_DDR_PC45), [SCLPIN] "I" (SCL_PIN_PC45), [SCLIN] "I" (SCL_IN_PC45),
      [SDAOUT] "I" (SDA_OUT_PC45), [SCLOUT] "I" (SCL_OUT_PC45),
      [SDADDR] "I"  (SDA_DDR_PC45), [SDAPIN] "I" (SDA_PIN_PC45), [SDAIN] "I" (SDA_IN_PC45) 
     ); 
  return ' '; // fool the compiler!
}
#endif