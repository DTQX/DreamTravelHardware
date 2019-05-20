
/** mpu采样率 60Hz;  arduino数据发送频率 33.33Hz
 *
 *  
 */
                  
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
// #include "I2Cdev.h"

// #include "MPU6050.h" 
#include "my_log.h"
#include "Wire.h"
#include "BonesMap.h"
#include "helper_3dmath.h"

#include "mpu6050.h"
// 开启调试
 #define DEBUG

// 串口相关
#define COM_RATE (115200)   // 串口通信速率

// 数据发送相关
#define MPU_DATA_SIZE 16     // 要发送的一个mpu的数据大小
const uint8_t START_CODE_1=88;   // 数据包开始标志
const uint8_t START_CODE_2=44;    // 数据表介绍标志
const int intervalTime = 0;    // 数据发送间隔时间，单位ms
uint8_t lastPacket[MPU_NUM][MPU_DATA_SIZE] = {0};     //储存上一次正确的quat
unsigned long lastSendTime = 10;     // 数据上一次发送的时间
double QUAT_SENS  = 1073741824.0;

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

    // 初始化mpu pins
    initMpuPins();

    // initialize serial communication
    Serial.begin(COM_RATE);
    delay(50);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    initDevice();
    
//     // 等待开始
//     // Serial.println(F("\nSend any character to begin DMP programming: "));
//     // while (Serial.available() && Serial.read()); // empty buffer
//     // while (!Serial.available());                 // wait for data
//     // while (Serial.available() && Serial.read()); // empty buffer again

    // test
    
    // Serial.print(readBit_c(44));
}




// // ================================================================
// // ===                    MAIN PROGRAM LOOP                     ===
// // ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    for(int i = 0; i<MPU_NUM; i++){
        //  记录上一次发送时间
        lastSendTime = millis();

        // 选中mpu
        selectMPU(mpuPins[i]);
       
        // 更新lastPacket
        updateOneLastPacket(i);
        // Serial.print("----------------");

        // 发送一个 mpu 的数据
        sendOneData(i);

        // 取消选中mpu
        unselectMPU(mpuPins[i]);

        // 保证发送频率
        while( millis() - lastSendTime < intervalTime);

    }

}

// 更新一个 mpu 的lastPacket
int updateOneLastPacket(int index){
    // DEBUG_PRINTLN("into mpu_read_latest_fifo ");

    int readResult = mpu_read_latest_fifo(fifoBuffer);
    
    if(readResult){

        DEBUG_PRINT("mpu_read_latest_fifo error result: ");
        DEBUG_PRINTLN(readResult);
        return -1;
    }
    
    // memcpy(lastPacket[index], fifoBuffer, 16 * sizeof(uint8_t));

    // mpu数据填充到 lastPacket
    for (size_t i = 0; i < MPU_DATA_SIZE; i++)
    {
        lastPacket[index][i] = fifoBuffer[i];
    }
    
    return 1;
}

// 发送一个 mpu 的数据
void sendOneData(int index){
    //发送数据，如果是一个数据包的开始，则发送开始标志符
    // 不管发生什么，都要发送每个mpu的数据，如果mpu出错则发送上一次正确的数据
    if(index == 0){
        #ifdef DEBUG
        Serial.print(START_CODE_1,HEX);
        Serial.print(START_CODE_2,HEX);
        Serial.print(" ");
        #else
        Serial.write(START_CODE_1);
        Serial.write(START_CODE_2);
        #endif
    }
    #ifdef DEBUG
    // for(int j = 0; j < MPU_DATA_SIZE; j++){
        // Serial.print(lastPacket[index][j],HEX);

        long quat[4];
        quat[0] = ((long)lastPacket[index][0] << 24) | ((long)lastPacket[index][1] << 16) |
            ((long)lastPacket[index][2] << 8) | lastPacket[index][3];
        quat[1] = ((long)lastPacket[index][4] << 24) | ((long)lastPacket[index][5] << 16) |
            ((long)lastPacket[index][6] << 8) | lastPacket[index][7];
        quat[2] = ((long)lastPacket[index][8] << 24) | ((long)lastPacket[index][9] << 16) |
            ((long)lastPacket[index][10] << 8) | lastPacket[index][11];
        quat[3] = ((long)lastPacket[index][12] << 24) | ((long)lastPacket[index][13] << 16) |
            ((long)lastPacket[index][14] << 8) | lastPacket[index][15];
        Serial.print(quat[0] / QUAT_SENS);
        Serial.print("  ");
        Serial.print(quat[1] / QUAT_SENS);
        Serial.print("  ");
        Serial.print(quat[2] / QUAT_SENS);
        Serial.print("  ");
        Serial.print(quat[3] / QUAT_SENS);
        Serial.print("  ");
    // }
    #else
    for(int j = 0; j < MPU_DATA_SIZE; j++){
        Serial.write(lastPacket[index][j]);
    }
    // Serial.write(fifoBuffer[0]);Serial.write(fifoBuffer[1]);
    // Serial.write(fifoBuffer[4]);Serial.write(fifoBuffer[5]);
    // Serial.write(fifoBuffer[8]);Serial.write(fifoBuffer[9]);
    // Serial.write(fifoBuffer[12]);Serial.write(fifoBuffer[13]);
    #endif
    // 发送数据包的结束编码, 不再发送结束编码
    #ifdef DEBUG
    if(index == MPU_NUM - 1){
      Serial.println();
    //     #ifdef DEBUG
    //     //Serial.println(START_CODE_2,HEX);
    //     Serial.print(START_CODE_2,HEX);
    //     #else
    //     Serial.write(START_CODE_2);
    //     #endif
    }
    #endif
}

// 选中mpu
void selectMPU(int mpuPin){
    // digitalWrite(mpuPin, LOW);
    // delay(50);
}

// 取消选中mpu
void unselectMPU(int mpuPin){
    // digitalWrite(mpuPin, HIGH);
    //delay(500);
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

// initialize device
void initDevice(){
    Serial.println(F("Initializing I2C devices..."));
    int innerResultCode[2] = {0,0};
    int resultCode;
    for(int i = 0; i< MPU_NUM; i++){
        // 选中mpu
         selectMPU(mpuPins[i]);

        delay(20);

        resultCode = my_mpu_init(innerResultCode);
        if(resultCode){
            DEBUG_PRINT(F("my_mpu_init resultCode: "));
            DEBUG_PRINT(resultCode);
            DEBUG_PRINT("  ");
            DEBUG_PRINT(" innerResultCode 0: ");
            DEBUG_PRINT(innerResultCode[0]);
            DEBUG_PRINT("  ");
            DEBUG_PRINT(" innerResultCode 1: ");
            DEBUG_PRINTLN(innerResultCode[1]);
            dmpReady = false;
        }
        
        // 取消选中mpu
        unselectMPU(mpuPins[i]); 
    }
}
