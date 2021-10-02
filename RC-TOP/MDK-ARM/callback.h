#ifndef CALLBACK_H
#define CALLBACK_H

#include "main.h"

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;

void can_init(void);
void can_cmd_chassis(int16_t motor1);










#endif
