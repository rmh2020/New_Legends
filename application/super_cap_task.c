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
#include "remote_Control.h"
#include "CAN_receive.h"

#include "referee.h"

fp32 Input_Vot;
fp32 Supercap_Vot;
fp32 Input_Current;
fp32 Target_Power;

extern ext_game_robot_state_t robot_state; //0x0201     比赛机器人状态

void Cap_Update_Cap_Inputvot(int16_t inputvot)
{
	Input_Vot = inputvot; //获取输入电压
}

void Cap_Update_Cap_Capvot(int16_t capvot)
{
	Supercap_Vot = capvot; //获取电容电压
}

void Cap_Update_Cap_Test_current(int16_t current)
{
	Input_Current = current; //获取输入电流
}

void Cap_Update_Cap_Target_Power(int16_t power)
{
	Target_Power = power; //获取输入功率
}
//主任务
void Super_cap_task(void)
{

	//等待陀螺仪任务更新陀螺仪数据
	vTaskDelay(SUPER_CAP_TASK_INIT_TIME);

	for (;;)
	{

		if (Supercap_Vot <= 15) //电容电压不要低于12V，这可能会造成C620低压保护
		{
			CAN1_Cap_Send(13000);
		}
		else if (robot_state.chassis_power_limit <= 40) //当前底盘最大功率限制<40  目标功率为39w
		{
			CAN1_Cap_Send(3900);
		}
		else if (robot_state.chassis_power_limit > 40 && robot_state.chassis_power_limit <= 50) //当前底盘最大功率限制40-50w  目标功率为49w
		{
			CAN1_Cap_Send(4900);
		}
		else if (robot_state.chassis_power_limit > 50 && robot_state.chassis_power_limit <= 60) //当前底盘最大功率限制50-60w  目标功率为59w
		{
			CAN1_Cap_Send(5900);
		}
		else if (robot_state.chassis_power_limit > 60 && robot_state.chassis_power_limit <= 70) //当前底盘最大功率限制60-70w  目标功率为79w
		{
			CAN1_Cap_Send(6900);
		}
		else if (robot_state.chassis_power_limit > 70 && robot_state.chassis_power_limit <= 80) //当前底盘最大功率限制70-80w  目标功率为79w
		{
			CAN1_Cap_Send(7900);
		}
		else if (robot_state.chassis_power_limit > 80 && robot_state.chassis_power_limit <= 100) //当前底盘最大功率限制80-100w  目标功率为99w
		{
			CAN1_Cap_Send(9900);
		}
		else if (robot_state.chassis_power_limit > 100 && robot_state.chassis_power_limit < 120) //当前底盘最大功率限制100-120w  目标功率为119w
		{
			CAN1_Cap_Send(11900);
		}
		else if (robot_state.chassis_power_limit >= 120) //当前底盘最大功率限制>120w  目标功率为130w
		{
			CAN1_Cap_Send(13000);
		}

		vTaskDelay(SUPER_CONTROL_TIME);
	}
}

void Cap_Init()
{
	CAN1_Cap_Send(13000);
}

/*********************************************************************************************/ //仅用于测试

//void Cap_RC_Ctrl()
//{
//		if (rc_ctrl.rc.s[0] == RC_SW_UP)//s1拨到上,CAN发送数据，将超电控制功率设置到38W
//	 {
//		 CAN1_Cap_Send(3800);
//     GPIO_SetBits(GPIOG, GPIO_Pin_1);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_4);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_7);
//	 }
//
//	 if(rc_ctrl.rc.s[0] == RC_SW_MID)//s1拨到中间,CAN发送数据，将超电控制功率设置到80W
//	 {
//		 CAN1_Cap_Send(8000);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_1);
//		 GPIO_SetBits(GPIOG, GPIO_Pin_4);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_7);

//	 }
//   if(rc_ctrl.rc.s[0] == RC_SW_DOWN)//s1到下,CAN发送数据，将超电控制功率设置到100W
//	 {
//     CAN1_Cap_Send(10000);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_1);
//		 GPIO_ResetBits(GPIOG, GPIO_Pin_4);
//		 GPIO_SetBits(GPIOG, GPIO_Pin_7);
//	 }
//}
