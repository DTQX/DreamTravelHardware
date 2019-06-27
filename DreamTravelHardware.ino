 
/** mpu采样率 60Hz;  arduino数据发送频率 33.33Hz
 *
 *  
 */
                  
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
// #include "I2Cdev.h"
#include "my_log.h"
#include "BonesMap.h"
#include "helper_3dmath.h"

// #include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

#include "Wire.h"

// 开启调试
 #define DEBUG

// 串口相关
#define COM_RATE (115200)   // 串口通信速率

// 数据发送相关
#define MPU_DATA_SIZE 16     // 要发送的一个mpu的数据大小
const uint8_t START_CODE_1=88;   // 数据包开始标志
const uint8_t START_CODE_2=44;    // 数据表介绍标志
const int intervalTime = 10;    // 数据发送间隔时间，单位ms
uint8_t lastPacket[MPU_NUM][MPU_DATA_SIZE] = {0};     //储存上一次正确的quat
unsigned long lastSendTime = 10;     // 数据上一次发送的时间
double QUAT_SENS  = 16384.0f;

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
    // 初始化Wire
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    

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
        // lastSendTime = millis();

        // 选中mpu
        selectMPU(mpuPins[i]);
       
        // 更新lastPacket
        updateOneLastPacket(i);
        // delay(15);
        // Serial.print("----------------");

        // 发送一个 mpu 的数据
        sendOneData(i);

        // 取消选中mpu
        unselectMPU(mpuPins[i]);

        // 保证发送频率
        // while( millis() - lastSendTime < intervalTime);

    }

}

// 更新一个 mpu 的lastPacket
int updateOneLastPacket(int index){
    // DEBUG_PRINTLN("into mpu_read_latest_fifo ");
    // int readResult = 1;
    unsigned char more[1];
    // int readResult = mpu_read_latest_fifo_stream(dmp_get_packet_length(), fifoBuffer);;
    // Serial.print("-----");
    // Serial.println(millis());
    int readResult = mpu_read_fifo_stream(dmp_get_packet_length(), fifoBuffer, more);;
    
    if(readResult){

        // DEBUG_PRINT("mpu_read_latest_fifo_stream error result: ");
        // DEBUG_PRINTLN(readResult);
        return -1;
    }
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

        // Serial.print("euler\t");
        // Serial.print(euler[0] * 180/3.1415);
        // Serial.print("\t");
        // Serial.print(euler[1] * 180/3.1415);
        // Serial.print("\t");
        // Serial.println(euler[2] * 180/3.1415);

        // // ue4的不需要
        // Serial.print("euler\t");
        // Serial.print(euler[0] );
        // Serial.print("\t");
        // Serial.print(euler[1] );
        // Serial.print("\t");
        // Serial.println(euler[2] );

        Serial.print("Quat :");
        
        Serial.print(q.w);
        Serial.print("  ");
        Serial.print(q.x);
        Serial.print("  ");
        Serial.print(q.y);
        Serial.print("  ");
        Serial.println(q.z);
        
        // Serial.println("  ");

        // Serial.print("origin Quat :");
        
        // Serial.print(quat[0] );
        // Serial.print("  ");
        // Serial.print(quat[1]);
        // Serial.print("  ");
        // Serial.print(quat[2] );
        // Serial.print("  ");
        // Serial.print(quat[3] );
        // Serial.print("  ");
    
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
        // Serial.print(START_CODE_1,HEX);
        // Serial.print(START_CODE_2,HEX);
        // Serial.print(" ");
        #else
        Serial.write(START_CODE_1);
        Serial.write(START_CODE_2);
        #endif
    }
    #ifdef DEBUG
    // for(int j = 0; j < MPU_DATA_SIZE; j++){
        // Serial.print(lastPacket[index][j],HEX);

        // long quat[4];
        // quat[0] = ((long)lastPacket[index][0] << 24) | ((long)lastPacket[index][1] << 16) |
        //     ((long)lastPacket[index][2] << 8) | lastPacket[index][3];
        // quat[1] = ((long)lastPacket[index][4] << 24) | ((long)lastPacket[index][5] << 16) |
        //     ((long)lastPacket[index][6] << 8) | lastPacket[index][7];
        // quat[2] = ((long)lastPacket[index][8] << 24) | ((long)lastPacket[index][9] << 16) |
        //     ((long)lastPacket[index][10] << 8) | lastPacket[index][11];
        // quat[3] = ((long)lastPacket[index][12] << 24) | ((long)lastPacket[index][13] << 16) |
        //     ((long)lastPacket[index][14] << 8) | lastPacket[index][15];
        // q.w = quat[0] / QUAT_SENS;
        // q.x = quat[1] / QUAT_SENS;
        // q.y = quat[2] / QUAT_SENS;
        // q.z = quat[3] / QUAT_SENS;
        // dmpGetEuler(euler, q);
        // Serial.print("euler");
        // Serial.print(euler[0]);
        // Serial.print("  ");
        // Serial.print(euler[1]);
        // Serial.print("  ");
        // Serial.println(euler[2]);

        // Serial.print("Quat :");
        
        // Serial.print(quat[0] / QUAT_SENS);
        // Serial.print("  ");
        // Serial.print(quat[1] / QUAT_SENS);
        // Serial.print("  ");
        // Serial.print(quat[2] / QUAT_SENS);
        // Serial.print("  ");
        // Serial.print(quat[3] / QUAT_SENS);
        // Serial.print("  ");

        // Serial.print("origin Quat :");
        
        // Serial.print(quat[0] );
        // Serial.print("  ");
        // Serial.print(quat[1]);
        // Serial.print("  ");
        // Serial.print(quat[2] );
        // Serial.print("  ");
        // Serial.print(quat[3] );
        // Serial.print("  ");
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
    //   Serial.println();
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
    // digitalWrite(22, LOW); 
}


uint8_t dmpGetEuler(float *data, Quaternion * q) {
    // 原来的
    // data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);   // psi
    // data[1] = -asin(2*q -> x*q -> z + 2*q -> w*q -> y);                              // theta
    // data[2] = atan2(2*q -> y*q -> z - 2*q -> w*q -> x, 2*q -> w*q -> w + 2*q -> z*q -> z - 1);   // phi

    // // 网上找的
    data[0] = atan2(2*q -> w*q -> x + 2*q -> y*q -> z, 1 - 2*q -> x*q -> x - 2*q -> y*q -> y);   // psi
    data[1] = asin(2*q -> w*q -> y - 2*q -> z*q -> x);                              // theta
    data[2] = atan2(2*q -> w*q -> z + 2*q -> x*q -> y, 1 - 2*q -> y*q -> y - 2*q -> z*q -> z);   // phi

    // // ue4的
    // const float SingularityTest = q->z*q->x - q->w*q->y;
	// const float YawY = 2.f*(q->w*q->z + q->x*q->y);
	// const float YawX = (1.f-2.f*((q->y) * (q->y) + (q->z) * (q->z)));

	
	// float SINGULARITY_THRESHOLD = 0.4999995f;
	// // float RAD_TO_DEG = (180.f)/PI;

	// if (SingularityTest < -SINGULARITY_THRESHOLD)
	// {
	// 	data[0] = -90.f;    // pitch
	// 	data[1] = atan2(YawY, YawX) * RAD_TO_DEG;    // yaw
	// 	data[2] = NormalizeAxis(-data[1] - (2.f * atan2(q->x, q->w) * RAD_TO_DEG));       // roll
	// }
	// else if (SingularityTest > SINGULARITY_THRESHOLD)
	// {
	// 	data[0] = 90.f;
	// 	data[1] = atan2(YawY, YawX) * RAD_TO_DEG;
	// 	data[2] = NormalizeAxis(data[1] - (2.f * atan2(q->x, q->w) * RAD_TO_DEG));
	// }
	// else
	// {
	// 	data[0] = asin(2.f*(SingularityTest)) * RAD_TO_DEG;
	// 	data[1] = atan2(YawY, YawX) * RAD_TO_DEG;
	// 	data[2] = atan2(-2.f*(q->w*q->x+q->y*q->z), (1.f-2.f*((q->x) * (q->x) + (q->y) * (q->y)))) * RAD_TO_DEG;
	// }

    
    return 0;
}

float ClampAxis( float Angle )
{
	// returns Angle in the range (-360,360)
    Angle = Fmod(Angle, 360.f);

	if (Angle < 0.f)
	{
		// shift to [0,360) range
		Angle += 360.f;
	}

	return Angle;
}

float Fmod(float X, float Y)
	{
		if (fabsf(Y) <= 1.e-8f)
		{
			return 0.f;
		}
		const float Quotient = (int32_t)(X / Y);
		float IntPortion = Y * Quotient;

		// Rounding and imprecision could cause IntPortion to exceed X and cause the result to be outside the expected range.
		// For example Fmod(55.8, 9.3) would result in a very small negative value!
		if (fabsf(IntPortion) > fabsf(X))
		{
			IntPortion = X;
		}

		const float Result = X - IntPortion;
		return Result;
	}

float NormalizeAxis( float Angle )
{
	// returns Angle in the range [0,360)
	Angle = ClampAxis(Angle);

	if (Angle > 180.f)
	{
		// shift to (-180,180]
		Angle -= 360.f;
	}

	return Angle;
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

// initialize device
void initDevice(){
    Serial.println(F("Initializing I2C devices..."));
    int innerResultCode[2] = {0,0};
    int resultCode;
    dmp_init_struct();
    mpu_init_struct();
    // for(int i = 0; i< MPU_NUM; i++){
        // 选中mpu
        //  selectMPU(mpuPins[i]);

        // delay(20);

        int result = mpu_init();

        if(result){
            Serial.print(F("mpu_init error!"));
            Serial.println(result);
            dmpReady = false;
            return;
        }

        mpu_set_bypass(1);

        /* Get/set hardware configuration. Start gyro. */
        /* Wake up all sensors. */
        if(mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)){
            Serial.print(F("mpu_set_sensors error"));
            dmpReady = false;
            return;
        }
        /* Push both gyro and accel data into the FIFO. */
        if(mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)){
            Serial.print(F("mpu_configure_fifo error"));
            dmpReady = false;
            return;
        }
        if(mpu_set_sample_rate(100)){
            Serial.print(F("mpu_set_sample_rate error"));
            dmpReady = false;
            return;
        }
        
        
        /* Read back configuration in case it was set improperly. */
        unsigned char accel_fsr;
        unsigned short gyro_rate, gyro_fsr;
        unsigned long timestamp;
        if(mpu_get_sample_rate(&gyro_rate)){
            Serial.print(F("mpu_get_sample_rate error"));
            dmpReady = false;
            return;
        }
        
        ;
        if(mpu_get_gyro_fsr(&gyro_fsr)){
            Serial.print(F("mpu_get_gyro_fsr error"));
            dmpReady = false;
            return;
        }
        
        ;
        if(mpu_get_accel_fsr(&accel_fsr)){
            Serial.print(F("mpu_get_accel_fsr error"));
            dmpReady = false;
            return;
        }
        
        ;

        /* Initialize HAL state variables. */
        // memset(&hal, 0, sizeof(hal));
        // hal.sensors = ACCEL_ON | GYRO_ON;
        // hal.report = PRINT_QUAT;

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
            dmpReady = false;
            return;
        }
        
    ;
    if(dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_orientation))){
        Serial.print(F("dmp_set_orientation error"));
        dmpReady = false;
        return;
    }
        
    
    if(dmp_register_tap_cb(tap_cb)){
        Serial.print(F("dmp_register_tap_cb error"));
        dmpReady = false;
        return;
    }
        
    ;
    if(dmp_register_android_orient_cb(android_orient_cb)){
            Serial.print(F("dmp_register_android_orient_cb error"));
        dmpReady = false;
        return;
    }
        
    ;
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
    if(dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
        DMP_FEATURE_GYRO_CAL)){
        Serial.print(F("dmp_enable_feature error"));
        dmpReady = false;
        return;
    }
    
    
    if(dmp_set_fifo_rate(100)){
            Serial.print(F("dmp_set_fifo_rate error"));
        dmpReady = false;
        return;
    }
        
    ;
    if(mpu_set_dmp_state(1)){
            Serial.print(F("mpu_set_dmp_state error"));
        dmpReady = false;
        return;
    }
        
    ;


        // 取消选中mpu
    //     unselectMPU(mpuPins[i]); 
    // }
}
