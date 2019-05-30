// 记录与Bone相对应的Mpu的AD0引脚。 Arduino连接多个Mpu，
// AD0默认为高，即未选中，要选中某个Mpu，让它的AD0为低。

#define MPU_NUM (16)
// #define MPU_NUM (2)  // 测试用

//bone name , mpu AD0 pin
// FName("upperarm_l"),FName("lowerarm_l"),FName("hand_l")
#define upperarm_l (22)
#define lowerarm_l (23)
#define hand_l (24)

// int mpuPins[MPU_NUM] = {upperarm_l};
// int mpuPins[MPU_NUM] = {hand_l};
int mpuPins[MPU_NUM] = {22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33 , 34, 35, 36, 37};
// int mpuPins[MPU_NUM] = {upperarm_l, lowerarm_l, hand_l};
// int mpuPins[MPU_NUM] = {upperarm_l, lowerarm_l};     // 测试用