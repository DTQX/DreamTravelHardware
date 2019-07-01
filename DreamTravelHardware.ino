 
/** mpu采样率 Hz;  arduino数据发送频率 Hz
 *
 *  
 */
                  
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
// #include "I2Cdev.h"
#include "myLog.h"
#include "BonesMap.h"
#include "helper_3dmath.h"

// #include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "MyI2Cdev.h"

// #include "Wire.h"

// 开启调试
 #define DEBUG

// 串口相关
#define COM_RATE (115200)   // 串口通信速率

// 数据发送相关
#define MPU_DATA_SIZE 8     // 要发送的一个mpu的数据大小
// #define MPU_DATA_SIZE 16     // 要发送的一个mpu的数据大小
const uint8_t START_CODE_1=88;   // 数据包开始标志
const uint8_t START_CODE_2=44;    // 数据表介绍标志
const int intervalTime = 10;    // 数据发送间隔时间，单位ms
uint8_t lastPacket[MPU_NUM][MPU_DATA_SIZE] = {0};     //储存上一次正确的quat
unsigned long lastSendTime = 0;     // 数据上一次发送的时间
// double QUAT_SENS  = 1073741824.0;   // 对应 MPU_DATA_SIZE 16
double QUAT_SENS  = 16384.0;   // 对应 MPU_DATA_SIZE 8 

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69

// MPU control/status vars
bool dmpReady = true;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes) mpu产生的数据大小
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint16_t maxFifoCount = 1024;
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
// uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// // ================================================================
// // ===                      INITIAL SETUP                       ===
// // ================================================================

void setup() {
    // TODO 加入连接协议

    // initialize serial communication
    Serial.begin(COM_RATE);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    // 初始化i2c
    i2c_init_my();

    // initialize device
    initDevice();
    
//     // 等待开始
//     // Serial.println(F("\nSend any character to begin DMP programming: "));
//     // while (Serial.available() && Serial.read()); // empty buffer
//     // while (!Serial.available());                 // wait for data
//     // while (Serial.available() && Serial.read()); // empty buffer again

}




// // ================================================================
// // ===                    MAIN PROGRAM LOOP                     ===
// // ================================================================

void loop() {
    // if programming failed, don't try to do anything
    // if (!dmpReady) return;
    Serial.println();
    Serial.println(micros());
    for(int i = 0; i<MPU_NUM; i++){
        //  记录上一次发送时间
        // lastSendTime = millis();
        // Serial.println(mpuPins[i]);
       
        // 更新lastPacket
        // updateOneLastPacket(0);
        // Serial.println(mpuPins[i]);
        // Serial.println(millis());
        updateOneLastPacket(i);
        // Serial.println(mpuPins[i]);
        // Serial.println(millis());
        // delay(15);
        // 发送一个mpu的数据包
        // sendOneData(i);
        // 取消选中mpu
        // unselectMPU(mpuPins[i]);

        // 保证发送频率
        // while( millis() - lastSendTime < intervalTime);

    }
    Serial.println(micros());
    // Serial.println();
    // Serial.println(millis());
}

// 更新一个 mpu 的lastPacket
int updateOneLastPacket(int index){
    // DEBUG_PRINTLN("into mpu_read_latest_fifo ");
    // Serial.print("-----");
    // Serial.println(millis());
    // if(index == 2){
    //     return -3;
    // }
    
    int result = mpu_read_latest_fifo_stream(dmp_get_packet_length(), fifoBuffer);;
    
    
    
    if(result){
        DEBUG_PRINT("mpu_read_latest_fifo_stream error result: ");
        DEBUG_PRINTLN(result);
        return -1;
    }
    // long quat[4];
    // quat[0] = ((long)fifoBuffer[0] << 8) | ((long)fifoBuffer[1]) ;
    // quat[1] = ((long)fifoBuffer[4] << 8) | ((long)fifoBuffer[5]);
    // quat[2] = ((long)fifoBuffer[8] << 8) | ((long)fifoBuffer[9]);
    // quat[3] = ((long)fifoBuffer[12] << 8) | ((long)fifoBuffer[13]) ;
    // q.w = quat[0] / QUAT_SENS;
    // q.x = quat[1] / QUAT_SENS;
    // q.y = quat[2] / QUAT_SENS;
    // q.z = quat[3] / QUAT_SENS;
    // dmpGetEuler(euler, &q);

    // Serial.print(index);
    // Serial.print(" --- euler:\t");
    // Serial.print(euler[0] * 180/M_PI);
    // Serial.print("\t");
    // Serial.print(euler[1] * 180/M_PI);
    // Serial.print("\t");
    // Serial.println(euler[2] * 180/M_PI);

    // Serial.print(" --- Quat :");
    
    // Serial.print(q.w);
    // Serial.print("  ");
    // Serial.print(q.x);
    // Serial.print("  ");
    // Serial.print(q.y);
    // Serial.print("  ");
    // Serial.print(q.z);
    // Serial.print("  ");

    // if(index == 5){
    //     Serial.println();
    // }

    // Serial.print("origin Quat :");
    
    // Serial.print(quat[0] );
    // Serial.print("  ");
    // Serial.print(quat[1]);
    // Serial.print("  ");
    // Serial.print(quat[2] );
    // Serial.print("  ");
    // Serial.print(quat[3] );
    // Serial.print("  ");
    

    // mpu数据填充到 lastPacket
    // memcpy(lastPacket + index * MPU_DATA_SIZE, fifoBuffer, MPU_DATA_SIZE * sizeof(uint8_t));
    lastPacket[index][ 0] = fifoBuffer[0];
    lastPacket[index][ 1] = fifoBuffer[1];
    lastPacket[index][ 2] = fifoBuffer[4];
    lastPacket[index][ 3] = fifoBuffer[5];
    lastPacket[index][ 4] = fifoBuffer[8];
    lastPacket[index][ 5] = fifoBuffer[9];
    lastPacket[index][ 6] = fifoBuffer[12];
    lastPacket[index][ 7] = fifoBuffer[13];

    return 0;
}

// 发送一个mpu的数据包
void sendOneData(int index){
    //发送数据，如果是一个数据包的开始，则发送开始标志符
    // 不管发生什么，都要发送每个mpu的数据，如果mpu出错则发送上一次正确的数据

    #ifdef DEBUG
    // Serial.print(START_CODE_1);
    // Serial.print(START_CODE_1);

    // for(int j = 0; j < MPU_DATA_SIZE; j++){
    //     Serial.print(lastPacket[index][j]);
    // }

    // Serial.print(START_CODE_2);
    // Serial.println(START_CODE_2);
    #else
    // if(index == 0){
    //     Serial.write(START_CODE_1);
    //     Serial.write(START_CODE_1);
    // }
    
    // for(int i = 0; i < MPU_DATA_SIZE; i++){
    //     Serial.write(lastPacket[index][i]);
    // }
    
    // if(index == MPU_NUM -1){
    //     Serial.write(START_CODE_2);
    //     Serial.write(START_CODE_2);
    // }
    

    #endif
}

// 初始化mpu pins
void initMpuPins(){
    //设置所有的mpu引脚为输出引脚
    for(int i = 0; i<MPU_NUM; i++){
        pinMode(mpuPins[i], OUTPUT); 
    }

    //设置所有的mpu引脚为高电平，默认不选中，低电平为选中
    for(int i = 0; i<MPU_NUM; i++){
        digitalWrite(mpuPins[i], HIGH); 
    }
}


uint8_t dmpGetEuler(float *data, Quaternion * q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi

    return 0;
}



// initialize device
void initDevice(){
    Serial.println(F("Initializing I2C devices..."));
    dmp_init_struct();
    mpu_init_struct();
    int result = 0;
    for(int i = 0; i< I2C_NUM; i++){
        // 访问第一个mpu
        set_dev_addr(0x68);
        Serial.print(mpuPins[i]);
        // Serial.print(mpuPins[0]);
        Serial.print("---");
        result = init_device();
        if(result){
            Serial.print("error:");
            Serial.println(result);

        }else
        {
            Serial.println("success");
        }

    }
}
