#include "MyI2Cdev.h"

// #define I2C_NOINTERRUPT 1
#define I2C_FASTMODE 1

#include "SoftI2C/SoftI2CMaster.h"
/**
 * portA 0 ~ 7 : 22 ~ 29
 * portC 0 ~ 7 : 37 ~ 30
 * portF 0 ~ 7 : A0 ~ A7
 * portG 0,1,2, portD 7 : 41 ~ 38
 * 
 */

uint8_t fifoBuffer[1024];


// 当前i2c总线
uint8_t currentPort = 0;
bool (*i2c_init[I2C_NUM])(void) = {
  i2c_init_pa01, i2c_init_pa23, i2c_init_pa45, i2c_init_pa67
  ,i2c_init_pb01, i2c_init_pb23, i2c_init_pb45, i2c_init_pb67
//   ,i2c_init_pc01, i2c_init_pc23, i2c_init_pc45, i2c_init_pc67
//   ,i2c_init_pd01
//   ,i2c_init_pf01, i2c_init_pf23, i2c_init_pf45, i2c_init_pf67
};
bool (*i2c_start[I2C_NUM])(uint8_t addr) = {
  i2c_start_pa01, i2c_start_pa23,i2c_start_pa45, i2c_start_pa67
  ,i2c_start_pb01, i2c_start_pb23,i2c_start_pb45, i2c_start_pb67
//   ,i2c_start_pc01, i2c_start_pc23,i2c_start_pc45, i2c_start_pc67
//   ,i2c_start_pd01
//   ,i2c_start_pf01, i2c_start_pf23,i2c_start_pf45, i2c_start_pf67
  };
bool (*i2c_rep_start[I2C_NUM])(uint8_t addr) = {
  i2c_rep_start_pa01, i2c_rep_start_pa23,i2c_rep_start_pa45, i2c_rep_start_pa67
  ,i2c_rep_start_pb01, i2c_rep_start_pb23,i2c_rep_start_pb45, i2c_rep_start_pb67
//   ,i2c_rep_start_pc01, i2c_rep_start_pc23,i2c_rep_start_pc45, i2c_rep_start_pc67
//   ,i2c_rep_start_pd01
//   ,i2c_rep_start_pf01, i2c_rep_start_pf23,i2c_rep_start_pf45, i2c_rep_start_pf67
};
uint8_t (*i2c_read[I2C_NUM])(bool last) = {
  i2c_read_pa01, i2c_read_pa23,i2c_read_pa45, i2c_read_pa67
  ,i2c_read_pb01, i2c_read_pb23,i2c_read_pb45, i2c_read_pb67
//   ,i2c_read_pc01, i2c_read_pc23,i2c_read_pc45, i2c_read_pc67
//   ,i2c_read_pd01
//   ,i2c_read_pf01, i2c_read_pf23,i2c_read_pf45, i2c_read_pf67
};
bool (*i2c_write[I2C_NUM])(uint8_t value) = {
  i2c_write_pa01, i2c_write_pa23,i2c_write_pa45, i2c_write_pa67
  ,i2c_write_pb01, i2c_write_pb23,i2c_write_pb45, i2c_write_pb67
//   ,i2c_write_pc01, i2c_write_pc23,i2c_write_pc45, i2c_write_pc67
//   ,i2c_write_pd01
//   ,i2c_write_pf01, i2c_write_pf23,i2c_write_pf45, i2c_write_pf67
};
void (*i2c_stop[I2C_NUM])(void) = {
  i2c_stop_pa01, i2c_stop_pa23, i2c_stop_pa45, i2c_stop_pa67
  ,i2c_stop_pb01, i2c_stop_pb23, i2c_stop_pb45, i2c_stop_pb67
//   ,i2c_stop_pc01, i2c_stop_pc23, i2c_stop_pc45, i2c_stop_pc67
//   ,i2c_stop_pd01
//   ,i2c_stop_pf01, i2c_stop_pf23, i2c_stop_pf45, i2c_stop_pf67
};

// i2c 多字节写
uint8_t i2c_write_bytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data){
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.println("into i2c_write_bytes");
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") writing ");
        Serial.print(length, DEC);
        Serial.print(" bytes to 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif
    // 开始新的i2c连接，发送从机地址
    if (!i2c_start[currentPort](devAddr <<1 | I2C_WRITE)){
        return -1;
    }
    // 发送读取地址
    if (!i2c_write[currentPort](regAddr)){
        return -2;
    }
    // 读取数据
    for (uint8_t i = 0; i < length; i++) {
        #ifdef I2CDEV_SERIAL_DEBUG
            Serial.print(data[i], HEX);
            if (i + 1 < length) Serial.print(" ");
        #endif
        i2c_write[currentPort](data[i]);
    }
    // 发送停止信号
    i2c_stop[currentPort]();
    return 0;
}
// i2c 多字节读
int8_t i2c_read_bytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data){
    #ifdef I2CDEV_SERIAL_DEBUG
        Serial.println("into i2c_read_bytes");
        Serial.print("I2C (0x");
        Serial.print(devAddr, HEX);
        Serial.print(") reading ");
        Serial.print(length, DEC);
        Serial.print(" bytes from 0x");
        Serial.print(regAddr, HEX);
        Serial.print("...");
    #endif
    // 开始新的i2c连接，发送从机地址
    if (!i2c_start[currentPort](devAddr <<1 | I2C_WRITE)){
        return -1;
    }
    // 发送读取地址
    i2c_write[currentPort](regAddr);
    // 发送停止信号
    // i2c_stop[currentPort]();
    // 开始新的i2c连接，读取数据
    if (!i2c_rep_start[currentPort]((devAddr <<1 | I2C_READ))){
        return -2;
    }
    uint8_t cnt;
    for (cnt=0; cnt < length - 1; cnt++){
        // 读取数据
        data[cnt] = i2c_read[currentPort](false);
        #ifdef I2CDEV_SERIAL_DEBUG
            Serial.print(data[cnt], HEX);
            if (cnt + 1 < length) Serial.print(" ");
        #endif
    }
    data[cnt] = i2c_read[currentPort](true);
    i2c_stop[currentPort]();
    return 0;
}

// 专门读取dmp
int8_t i2c_read_dmp(uint8_t devAddr, uint8_t packetSize, uint8_t *lastPacket, uint16_t startIndex){
    Serial.println();
    Serial.println(micros());
    
    // 开始新的i2c连接，发送从机地址
    if (!i2c_start[currentPort](devAddr <<1 | I2C_WRITE)){
        return -1;
    }
    // 发送读取地址,读取fifo_cout
    i2c_write[currentPort](0x72);
    if (!i2c_rep_start[currentPort]((devAddr <<1 | I2C_READ))){
        return -2;
    }
    
    uint8_t tmp[2];
    tmp[0] = i2c_read[currentPort](false);
    tmp[1] = i2c_read[currentPort](true);
    unsigned short fifo_count = (tmp[0] << 8) | tmp[1];

    if (fifo_count > 512) {
        /* FIFO is 50% full, better check overflow bit. */
        return -3;
    }

    uint16_t cnt;
    uint16_t length = fifo_count/packetSize * packetSize;

    if(length > 0){
        // dmp有数据
        if (!i2c_rep_start[currentPort](devAddr <<1 | I2C_WRITE)){
            return -1;
        }
        // 发送读取地址， 读取dmp fifo
        i2c_write[currentPort](0x74);
        if (!i2c_rep_start[currentPort]((devAddr <<1 | I2C_READ))){
            return -2;
        }
        for (cnt=0; cnt < length - 1; cnt++){
            // 读取数据
            fifoBuffer[cnt] = i2c_read[currentPort](false);
            #ifdef I2CDEV_SERIAL_DEBUG
                Serial.print(lastPacket[cnt], HEX);
                if (cnt + 1 < length) Serial.print(" ");
            #endif
            // Serial.print(fifoBuffer[cnt], HEX);
            // Serial.print(" ");
        }
        fifoBuffer[cnt] = i2c_read[currentPort](true);

        uint16_t index = length - packetSize;
        lastPacket[startIndex] = fifoBuffer[index];
        lastPacket[startIndex + 1] = fifoBuffer[index+1];
        lastPacket[startIndex + 2] = fifoBuffer[index+4];
        lastPacket[startIndex + 3] = fifoBuffer[index+5];
        lastPacket[startIndex + 4] = fifoBuffer[index+8];
        lastPacket[startIndex + 5] = fifoBuffer[index+9];
        lastPacket[startIndex + 6] = fifoBuffer[index+12];
        lastPacket[startIndex + 7] = fifoBuffer[index+13];

        Serial.println(micros());

    }

    // 断开连接
    i2c_stop[currentPort]();

    return 0;
}

// i2c初始化
int8_t my_i2c_init(){
    for(uint8_t i = 0; i < I2C_NUM; i++){
        // 如果SCL为低电平，并不会直接返回，会一直等SCL变高
        // i2c初始化, 如果SCL为低电平直接返回false，返回true表示成功
        i2c_init[currentPort]();
    }
}

// 设置当前i2c bus
void setCurrentPort(uint8_t port){
    currentPort = port;
}

int8_t get_ms(unsigned long *timestamp){
    *timestamp = 0;
    return 0;
}

