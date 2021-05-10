/**
  * @file       super_cap.c/h
  * @brief      超级电容控制任务
  * @note       可从超电电容控制板中读取电容电压、裁判系统输入电流、裁判系统输入电压、设定功率
								需要注意，当需要持续超功率时，应关注电容电压不要低于12V，会造成C620电调的低压保护，使电机断电
								使用方式：当底盘电机（负载）超过设定功率时，裁判系统输入不会超过设定功率，不足部分由法拉电容补偿
								由于法拉电容容量有限，补偿时间也有限，所以超功率时间也不宜太久。电容容量8F(100F,12串)
								通过can通讯设定功率，从30W-130W可调，默认功率35W，发送值3000-13000，无论负载如何变化裁判系统输出功率始终会低于130W
								控制建议：
								超级电容设定功率依靠从裁判系统读取的功率上限值自动改变，底盘运动依靠加速来实现短暂超功率
								
								特别注意：确定主控模块的版本，以及相对应的裁判系统串口协议版本，否则会导致无法到正确的裁判信息，导致超级电容失效
  *
  @verbatim
  ==============================================================================
   一下内容需添加进CAN_receive.c中
	 
	 #include "super_cap.h"
	 
	 int16_t Cap_Inputvot,Cap_Capvot,Cap_Test_current,Cap_Target_Power;
	
	 if(RxMessage.StdId == 0x211)//超级电容控制板		
	{
		Cap_Inputvot  = (fp32)((int16_t)(RxMessage.Data[1]<<8|RxMessage.Data[0]))/100.0f;  //输入电压
		Cap_Update_Cap_Inputvot(Cap_Inputvot);
		
		Cap_Capvot = (fp32)((int16_t)(RxMessage.Data[3]<<8|RxMessage.Data[2]))/100.0f;  //电容电压
		Cap_Update_Cap_Capvot(Cap_Capvot);
		
		Cap_Test_current = (fp32)((int16_t)(RxMessage.Data[5]<<8|RxMessage.Data[4]))/100.0f;	  //输入电流
		Cap_Update_Cap_Test_current(Cap_Test_current);
		
		Cap_Target_Power = (fp32)((int16_t)(RxMessage.Data[7]<<8|RxMessage.Data[6]))/100.0f;	 //输入功率
		Cap_Update_Cap_Target_Power(Cap_Target_Power);
	}	
  ==============================================================================
  @endverbatim
  */
#include "super_cap_task.h"
#include "can.h"
#include "remote_control.h"
#include "CAN_receive.h"

#include "referee.h"

fp32 input_vot;
fp32 supercap_vot;
fp32 input_current;
fp32 target_power;

extern ext_game_robot_state_t game_state; //0x0201     比赛机器人状态
uint8_t cap_change = FALSE; //超电电压过低标识符


void cap_update_cap_inputvot(int16_t inputvot)
{
	input_vot = inputvot; //获取输入电压
}

void cap_update_cap_capvot(int16_t capvot)
{
	supercap_vot = capvot; //获取电容电压
}

void cap_update_cap_test_current(int16_t current)
{
	input_current = current; //获取输入电流
}

void cap_update_cap_target_power(int16_t power)
{
	target_power = power; //获取输入功率
}

//主任务
void super_cap_task(void const *pvParameters)
{

	//等待陀螺仪任务更新陀螺仪数据
	vTaskDelay(SUPER_CAP_TASK_INIT_TIME);

	for (;;)
	{

					if( supercap_vot <= 14 || cap_change == TRUE) //电容电压不要低于12V，这可能会造成C620低压保护,电机直接断电
					{
						 if(supercap_vot >= 20)
						{
								if (game_state.chassis_power_limit <= 40 )//当前底盘最大功率限制<40  目标功率为39w
								{
									 CAN_cmd_super_cap(3900);
								}
								else if(game_state.chassis_power_limit > 40 && game_state.chassis_power_limit <= 50)//当前底盘最大功率限制40-50w  目标功率为49w
								{
									 CAN_cmd_super_cap(4900);
								}	
								else if(game_state.chassis_power_limit > 50 && game_state.chassis_power_limit <= 60)//当前底盘最大功率限制50-60w  目标功率为59w
								{
									CAN_cmd_super_cap(5900);
								}
								else if(game_state.chassis_power_limit > 60 && game_state.chassis_power_limit <= 70)//当前底盘最大功率限制60-70w  目标功率为79w
								{
									CAN_cmd_super_cap(6900);
								}
								else if(game_state.chassis_power_limit > 70 && game_state.chassis_power_limit <= 80)//当前底盘最大功率限制70-80w  目标功率为79w
								{
									CAN_cmd_super_cap(7900);
								}
								else if(game_state.chassis_power_limit > 80 && game_state.chassis_power_limit <= 100)//当前底盘最大功率限制80-100w  目标功率为99w
								{
									CAN_cmd_super_cap(9900);
								}
								else if(game_state.chassis_power_limit > 100 && game_state.chassis_power_limit < 120)//当前底盘最大功率限制100-120w  目标功率为119w
								{
									CAN_cmd_super_cap(11900);
								}
								cap_change = FALSE;
							}
						else
						{
							cap_change = TRUE;
							CAN_cmd_super_cap(13000);
						}
					}
					else
					{
						if (game_state.chassis_power_limit <= 40 )//当前底盘最大功率限制<40  目标功率为39w
						{
							 CAN_cmd_super_cap(3900);
						}
						else if(game_state.chassis_power_limit > 40 && game_state.chassis_power_limit <= 50)//当前底盘最大功率限制40-50w  目标功率为49w
						{
							 CAN_cmd_super_cap(4900);
						}	
						else if(game_state.chassis_power_limit > 50 && game_state.chassis_power_limit <= 60)//当前底盘最大功率限制50-60w  目标功率为59w
						{
							CAN_cmd_super_cap(5900);
						}
						else if(game_state.chassis_power_limit > 60 && game_state.chassis_power_limit <= 70)//当前底盘最大功率限制60-70w  目标功率为79w
						{
							CAN_cmd_super_cap(6900);
						}
						else if(game_state.chassis_power_limit > 70 && game_state.chassis_power_limit <= 80)//当前底盘最大功率限制70-80w  目标功率为79w
						{
							CAN_cmd_super_cap(7900);
						}
						else if(game_state.chassis_power_limit > 80 && game_state.chassis_power_limit <= 100)//当前底盘最大功率限制80-100w  目标功率为99w
						{
							CAN_cmd_super_cap(9900);
						}
						else if(game_state.chassis_power_limit > 100 && game_state.chassis_power_limit < 120)//当前底盘最大功率限制100-120w  目标功率为119w
						{
							CAN_cmd_super_cap(11900);
						}
					}
				}

		vTaskDelay(SUPER_CONTROL_TIME);
}



void cap_init()
{
	CAN_cmd_super_cap(13000);
}


