#include <Arduino.h>
#include "SerialForS.h"

void serial_print(char * data){
    Serial.print(*data);
}

void serial_println(char * data){
    Serial.println(*data);
}