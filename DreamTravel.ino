/** mpu采样率 60Hz;  arduino数据发送频率 33.33Hz
 *
 *  
 */

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050.h" 

#include "Wire.h"
#include "BonesMap.h"

// 通信相关
#define COM_RATE (115200)   // 串口通信速率
int8_t START_CODE 88;   // 数据包开始标志
int8_t END_CODE 44;    // 数据表介绍标志

// 数据发送相关
const int intervalTime = 30;    // 数据发送间隔时间
int16_t lastQuat[MPU_NUM][4] = {0};     //储存上一次正确的quat
unsigned long lastSendTime = 0;     // 数据上一次发送的时间

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = true;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes) mpu产生的数据大小
uint16_t fifoCount;     // count of all bytes currently in FIFO
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



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // TODO 加入连接协议


    //设置所有的mpu引脚为输出引脚
    for(int i = 0; i<MPU_NUM; i++){
        pinMode(mpuPins[i], OUTPUT); 
    }
    //设置所有的mpu引脚为高电平，默认不选中，低电平为选中
    for(int i = 0; i<MPU_NUM; i++){
        digitalWrite(mpuPins[i], HIGH); 
    }

    // initialize serial communication
    Serial.begin(COM_RATE);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    for(int i = 0; i<MPU_NUM; i++){
        // 开启mpu
        digitalWrite(mpuPins[i], LOW); 
        // mpu sample rate  200Hz
        mpu.initialize();
        Serial.print(mpuPins[i]);
        Serial.print("--")
        Serial.println(mpu.testConnection() ? F("cs") : F("cf"));
        // 关闭mpu
        digitalWrite(mpuPins[i], HIGH); 
    }
    
    // 等待开始
    Serial.println(F("\nSend any character to begin DMP programming: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    for(int i = 0; i<MPU_NUM; i++){
        // 开启mpu
        digitalWrite(mpuPins[i], LOW); 
        devStatus = mpu.dmpInitialize();
        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        // mpu.setXGyroOffset(0);
        // mpu.setYGyroOffset(0);
        // mpu.setZGyroOffset(0);
        mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

        // make sure it worked (returns 0 if so)
        if (devStatus == 0) {
            // turn on the DMP, now that it's ready
            Serial.println(F("Enabling DMP..."));
            Serial.print(i);
            mpu.setDMPEnabled(true);
            mpuIntStatus = mpu.getIntStatus();
            dmpReady = dmpReady && true;
            // get expected DMP packet size for later comparison
            packetSize = mpu.dmpGetFIFOPacketSize();
        } else {
            // ERROR!
            // 1 = initial memory load failed
            // 2 = DMP configuration updates failed
            // (if it's going to break, usually the code will be 1)
            dmpReady = dmpReady && false;
            Serial.print(i);
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
        }
        // 关闭mpu
        digitalWrite(mpuPins[i], HIGH);
    }
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    for(int i = 0; i<MPU_NUM; i++){
        lastSendTime = millis();

        // 开启mpu
        digitalWrite(mpuPins[i], LOW); 

        fifoCount = mpu.getFIFOCount();

        if(fifoCount < packetSize){
            // TODO 发送上一次的数据

            digitalWrite(mpuPins[i], HIGH); 
        }

        mpuIntStatus = mpu.getIntStatus();
        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            fifoCount = mpu.getFIFOCount();
            Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            // 读取最新数据
            while(fifoCount / packetSize > 0){
                // read a packet from FIFO
                mpu.getFIFOBytes(fifoBuffer, packetSize);

                // track FIFO count here in case there is > 1 packet available
                // (this lets us immediately read more without waiting for an interrupt)
                fifoCount -= packetSize;
                
                // mpu.dmpGetQuaternion(&q, fifoBuffer);
                // mpu.dmpGetQuaternion(lastQuat[i], fifoBuffer);

                DEBUG_PRINT(F("get quat i:"));
                DEBUG_PRINTLN(i);
            }
        }

        //发送数据，如果是一个数据包的开始，则发送开始标志符
        // 不管发生什么，都要发送每个mpu的数据，如果mpu出错则返回上一次正确的数据
        if(i == 0){
            Serial.write(START_CODE);
        }
        // for(int j = 0; j < 4; j++){
        Serial.write(fifoBuffer[0]);Serial.write(fifoBuffer[1]);
        Serial.write(fifoBuffer[4]);Serial.write(fifoBuffer[5]);
        Serial.write(fifoBuffer[8]);Serial.write(fifoBuffer[9]);
        Serial.write(fifoBuffer[12]);Serial.write(fifoBuffer[13]);
        // }
        // 发送数据包的结束编码
        if(i == MPU_NUM - 1){
            Serial.write(END_CODE);
        }

        // 关闭mpu
        digitalWrite(mpuPins[i], HIGH); 

        // 保证发送频率
        while(lastSendTime - millis() < intervalTime);
        

    }

        // blink LED to indicate activity
        // blinkState = !blinkState;
        // digitalWrite(LED_PIN, blinkState);
}
