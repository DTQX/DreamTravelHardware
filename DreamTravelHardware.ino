 
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

// mpu电源控制pin
#define MPU_POWER 21

// set dmp bias interrupt pin
#define SetBiasIntPin 20

// 数据发送相关
#define MPU_DATA_SIZE 8     // 要发送的一个mpu的数据大小
// #define MPU_DATA_SIZE 16     // 要发送的一个mpu的数据大小
const uint8_t START_CODE_1=88;   // 数据包开始标志
const uint8_t CODE_LENGTH = 2;   // 标志符长度
const uint8_t START_CODE_2=44;    // 数据表介绍标志
const uint8_t intervalTime = 10;    // 数据发送间隔时间，单位ms
const uint16_t PACKET_BUFFER_LENGTH = (I2C_NUM * 2) * MPU_DATA_SIZE + 2 * CODE_LENGTH ;   //  TODO 替换成数字 ， 完整数据包的长度，包括标志符
uint8_t lastPacket[PACKET_BUFFER_LENGTH] = {0};     //储存上一次正确的quat
unsigned long lastSendTime = 0;     // 数据上一次发送的时间
// double QUAT_SENS  = 1073741824.0;   // 对应 MPU_DATA_SIZE 16
double QUAT_SENS  = 16384.0;   // 对应 MPU_DATA_SIZE 8 

// MPU control/status vars
bool dmpReady = true;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t packetSize;    // expected DMP packet size  dmp产生的数据大小
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[32]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================


// // ================================================================
// // ===                      INITIAL SETUP                       ===
// // ================================================================

void setup() {
    // TODO 加入连接协议

    // 重置mpu
    resetMpu();

    // initialize serial communication
    Serial.begin(COM_RATE);

    // initialize device
    initDevice();

    initInterrupt();

    // 设置开始、结束标志符
    lastPacket[0] = START_CODE_1;
    lastPacket[1] = START_CODE_1;
    lastPacket[PACKET_BUFFER_LENGTH -2] = START_CODE_2;
    lastPacket[PACKET_BUFFER_LENGTH -1] = START_CODE_2;
    
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
    // Serial.println();
    // Serial.println(micros());
    for(uint8_t i = 0; i < I2C_NUM; i++){
        // 设置当前端口 
        setCurrentPort(i);

        set_dev_addr(0x68);
        // 更新lastPacket
        updateOneLastPacket( 2 * i * MPU_DATA_SIZE + CODE_LENGTH);

        set_dev_addr(0x69);
        // 更新lastPacket
        updateOneLastPacket( (2 * i + 1) * MPU_DATA_SIZE + CODE_LENGTH);
    }
    // 发送一个完整数据包
    // sendData();

    if(digitalRead(SetBiasIntPin) == LOW){
        handleSetDmpBias();
    }

    // 保证发送频率
    // while( millis() - lastSendTime < intervalTime);

    // Serial.println(micros());
    // Serial.println();
    // Serial.println(millis());
}

// 更新一个 mpu 的lastPacket
uint8_t updateOneLastPacket(uint16_t startIndex){
    if(mpu_read_latest_fifo_stream(packetSize, fifoBuffer)){
        DEBUG_PRINT("mpu_read_latest_fifo_stream error result: ");
        DEBUG_PRINTLN(result);
        return -1;
    }
    // mpu数据填充到 lastPacket
    
    lastPacket[startIndex] = fifoBuffer[0];
    lastPacket[startIndex + 1] = fifoBuffer[1];
    lastPacket[startIndex + 2] = fifoBuffer[4];
    lastPacket[startIndex + 3] = fifoBuffer[5];
    lastPacket[startIndex + 4] = fifoBuffer[8];
    lastPacket[startIndex + 5] = fifoBuffer[9];
    lastPacket[startIndex + 6] = fifoBuffer[12];
    lastPacket[startIndex + 7] = fifoBuffer[13];

    formateOutput();

    return 0;
}

// 发送一个完整数据包
void sendData(){
    // 发送一个完整数据包
    // 不管发生什么，都要发送每个mpu的数据，如果mpu出错则发送上一次正确的数据

    #ifdef DEBUG
    for(uint8_t j = 0; j < PACKET_BUFFER_LENGTH; j++){
        Serial.print(lastPacket[j]);
    }
    Serial.println();
    #else
    Serial.write(lastPacket, PACKET_BUFFER_LENGTH);
    #endif
}

// // 初始化mpu pins
// void initMpuPins(){
//     //设置所有的mpu引脚为输出引脚
//     for(uint8_t i = 0; i<MPU_NUM; i++){
//         pinMode(mpuPins[i], OUTPUT); 
//     }

//     //设置所有的mpu引脚为高电平，默认不选中，低电平为选中
//     for(uint8_t i = 0; i<MPU_NUM; i++){
//         digitalWrite(mpuPins[i], HIGH); 
//     }
// }

// 重启mpu，防止mpu占用i2c不放
void resetMpu(){
    pinMode(MPU_POWER, OUTPUT); 
    digitalWrite(MPU_POWER, LOW);
    delay(50);
    digitalWrite(MPU_POWER, HIGH);
    delay(50);
}

// initialize device
void initDevice(){
    // 初始化i2c
    my_i2c_init();

    Serial.println(F("Initializing I2C devices..."));
    // 初始化mpu数据结构
    dmp_init_struct();
    mpu_init_struct();
    for(uint8_t i = 0; i< I2C_NUM; i++){
        // 设置当前端口 
        setCurrentPort(i);

        // 访问第一个mpu
        set_dev_addr(0x68);
        Serial.print(i);
        Serial.print("---0x68:");
        Serial.println(init_device());
        
        // 访问第二个mpu
        set_dev_addr(0x69);
        Serial.print(i);
        Serial.print("---0x69:");
        Serial.println(init_device());
    }

    // 获取dmp数据包大小
    packetSize = dmp_get_packet_length();
}

// 初始化中断
void initInterrupt(){
    pinMode(SetBiasIntPin, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(SetBiasIntPin), handleSetDmpBias, LOW);
}

void handleSetDmpBias(){
    long gyro, accel;
    Serial.println("set dmp bias");
    for(uint8_t i = 0; i< I2C_NUM; i++){
        // 设置当前端口 
        setCurrentPort(i);
        
        // 访问第一个mpu
        set_dev_addr(0x68);
        Serial.print(i);
        Serial.print("---0x68:");
        Serial.println(init_device());
        mpu_run_6500_self_test(&gyro, &accel, 0);
        dmp_set_gyro_bias(&gyro);
        dmp_set_accel_bias(&accel);

        // 访问第二个mpu
        set_dev_addr(0x69);
        Serial.print(i);
        Serial.print("---0x69:");
        Serial.println(init_device());
        mpu_run_6500_self_test(&gyro, &accel, 0);
        dmp_set_gyro_bias(&gyro);
        dmp_set_accel_bias(&accel);
    }

}

// 获取euler
uint8_t dmpGetEuler(float *data, Quaternion * q) {
    data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi

    return 0;
}

// 格式化输出，用于调试
void formateOutput(){
    int16_t quat[4];
    quat[0] = (fifoBuffer[0] << 8) | (fifoBuffer[1] ) ;
    quat[1] = (fifoBuffer[4] << 8) | (fifoBuffer[5] ) ;
    quat[2] = (fifoBuffer[8] << 8) | (fifoBuffer[9] ) ;
    quat[3] = (fifoBuffer[12] << 8) | (fifoBuffer[13] );

    q.w = quat[0] / QUAT_SENS;
    q.x = quat[1] / QUAT_SENS;
    q.y = quat[2] / QUAT_SENS;
    q.z = quat[3] / QUAT_SENS;
    dmpGetEuler(euler, &q);

    // Serial.print(index);
    Serial.print(" --- euler:\t");
    Serial.print(euler[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(euler[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(euler[2] * 180/M_PI);

    Serial.print(" --- Quat :");
    
    Serial.print(q.w);
    Serial.print("  ");
    Serial.print(q.x);
    Serial.print("  ");
    Serial.print(q.y);
    Serial.print("  ");
    Serial.print(q.z);
    Serial.print("  ");

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
    
}