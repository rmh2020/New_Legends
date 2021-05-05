#ifndef SUPER_CAP_TASK_H
#define SUPER_CAP_TASK_H

#include "main.h"
#include "stdbool.h"
#include "CAN_receive.h"

#define SUPER_CONTROL_TIME 1

//任务初始化 空闲一段时间
#define SUPER_CAP_TASK_INIT_TIME 201



void cap_update_cap_inputvot(int16_t inputvot );
void cap_update_cap_capvot(int16_t capvot );
void cap_update_cap_test_current(int16_t current );
void cap_update_cap_target_power(int16_t power );

void cap_init(void);
extern void super_cap_task(void const *pvParameters);


#endif
