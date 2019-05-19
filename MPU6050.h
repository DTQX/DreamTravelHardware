#include <inttypes.h>

#ifdef __cplusplus 
extern "C" {
#endif

int my_mpu_init(int * innerResultCode);
int mpu_read_latest_fifo(unsigned char *fifo_data);

#ifdef __cplusplus 
}
#endif

