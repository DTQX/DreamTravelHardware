// #include <stdlib.h>
// #include <string.h>

// // #include "twi.h"
// #include "wire_c.h"

// #define BUFFER_LENGTH 32
// uint8_t transmitting;
// uint8_t rxBuffer[BUFFER_LENGTH];
// uint8_t rxBufferIndex;
// uint8_t rxBufferLength;

// uint8_t txAddress;
// uint8_t txBuffer[BUFFER_LENGTH];
// uint8_t txBufferIndex;
// uint8_t txBufferLength;


// void Wire_beginTransmission(uint8_t address){
//   transmitting = 1;
//   txAddress = address;
//   txBufferIndex = 0;
//   txBufferLength = 0;
// }

// size_t Wire_write(uint8_t data){
//   if(transmitting){
//     if(txBufferLength >= BUFFER_LENGTH){
//     //   setWriteError();
//       return 0;
//     }
//     txBuffer[txBufferIndex] = data;
//     ++txBufferIndex;
//     txBufferLength = txBufferIndex;
//   } else {
//     // i2c_slave_transmit(&data, 1);
//   }
//   return 1;
// }

// uint8_t Wire_endTransmission(void){
// //   return endTransmission(true);
// }
