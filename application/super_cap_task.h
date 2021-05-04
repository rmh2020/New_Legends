#ifndef SUPER_CAP_TASK_H
#define SUPER_CAP_TASK_H

#include "main.h"
#include "stdbool.h"
#include "CAN_receive.h"

#define SUPER_CONTROL_TIME 1

//任务初始化 空闲一段时间
#define SUPER_CAP_TASK_INIT_TIME 201

void Cap_Update_Cap_Inputvot(int16_t inputvot );
void Cap_Update_Cap_Capvot(int16_t capvot );
void Cap_Update_Cap_Test_current(int16_t current );
void Cap_Update_Cap_Target_Power(int16_t power );

void Cap_Init(void);
void Super_cap_task(void);
void CAN1_Cap_Send(uint16_t temPower);


#endif
