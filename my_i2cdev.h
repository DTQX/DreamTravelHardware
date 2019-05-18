#include <inttypes.h>



#ifdef __cplusplus 
extern "C" {
#endif

    int8_t writeBytes_c(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);

    int8_t readBytes_c(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);

#ifdef __cplusplus 
}
#endif