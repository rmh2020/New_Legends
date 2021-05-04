#include "vision.h"
#include "remote_control.h"
#include "struct_typedef.h"
#include "stdio.h"
#include "referee.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

uint8_t  Vision_Buffer[2][VISION_BUFFER_LEN];  //视觉数据暂存

extern RC_ctrl_t rc_ctrl;

//角度初始化补偿
float Vision_Comps_Yaw   = COMPENSATION_YAW;
float Vision_Comps_Pitch = COMPENSATION_PITCH;//固定补偿，减小距离的影响
float Vision_Comps_Pitch_Dist = COMPENSATION_PITCH_DIST;//根据距离补偿

VisionSendHeader_t    VisionSendHeader;        //帧头

VisionActData_t       VisionActData;          //行动模式结构体

VisionRecvData_t      VisionRecvData;        //接收数据结构体

VisionSendData_t      VisionSendData;        //发送数据结构体

uint8_t Attack_Color_Choose = ATTACK_NONE;  //默认不识别


//打符是否换装甲了
uint8_t Vision_Armor = FALSE;

//视觉是否发了新数据,FALSE没有,TRUE发了新的
uint8_t Vision_Get_New_Data = FALSE;


//角度补偿,发送给视觉
float Vision_Comps_Yaw_Send   = COMPENSATION_YAW;
float Vision_Comps_Pitch_Send = COMPENSATION_PITCH;
float SB_K_comps = 3.f;

void vision_init()
{
    usart1_init(Vision_Buffer[0], Vision_Buffer[1], VISION_BUFFER_LEN);
}


/**
  * @brief  读取视觉信息
  * @param  uart1缓存数据
  * @retval void
  * @attention  IRQ执行
  */
uint8_t Vision_Time_Test[2] = {0};            //当前数据和上一次数据
uint8_t Vision_Ping = 0;                 //发送时间间隔
void vision_read_data(uint8_t *ReadFormUart)
{
				
	//判断帧头数据是否为0xA5
	if(ReadFormUart[0] == VISION_BEGIN)
	{
		//判断帧头数据是否为0xff
		if(ReadFormUart[17] == VISION_END)
		{

		//接收数据拷贝
		memcpy( &VisionRecvData, ReadFormUart, VISION_LEN_PACKED);	
		Vision_Get_New_Data = TRUE;//标记视觉数据更新了
				
		//帧计算
		Vision_Time_Test[NOW] = xTaskGetTickCount();
		Vision_Ping = Vision_Time_Test[NOW] - Vision_Time_Test[LAST];//计算时间间隔
		Vision_Time_Test[LAST] = Vision_Time_Test[NOW];
		}
	}

}


/**
  * @brief  发送视觉指令
  * @param  CmdID
  * @retval void
  * @attention  按协议打包好数据发送
  *				CmdID   0x00   关闭视觉
  *				CmdID   0x01   识别红色装甲
  *				CmdID   0x02   识别蓝色装甲
  *				CmdID   0x03   小符
  *				CmdID   0x04   大符
  */
uint8_t vision_send_pack[50] = {0};//大于22就行
void vision_send_data(uint8_t CmdID)
{
	int i;    //循环发送次数
	uint16_t id1_17mm_speed_limit;
	uint16_t bullet_speed;
	get_shooter_id1_17mm_speed_limit_and_bullet_speed(&id1_17mm_speed_limit, &bullet_speed);	

	VisionSendData.BEGIN = VISION_BEGIN;
	
	VisionSendData.CmdID   = CmdID;
	VisionSendData.speed   =bullet_speed;
	
	VisionSendData.END    = VISION_END;

	
	memcpy( vision_send_pack + 2, &VisionSendData, 1);
	
	
	memset(vision_send_pack, 0, 50);
}


void vision_error_angle(float *yaw_angle_error, float *pitch_angle_error)
{
	*yaw_angle_error = VisionRecvData.yaw_angle;
	*pitch_angle_error = VisionRecvData.pitch_angle;
	
	if(VisionRecvData.yaw_angle == 0)
	{
		*yaw_angle_error = 0;
	}
	if(VisionRecvData.pitch_angle == 0)
	{
		*pitch_angle_error = 0;
	}
}



/**
  * @brief  判断视觉数据更新了吗
  * @param  void
  * @retval TRUE更新了   FALSE没更新
  * @attention  为自瞄做准备,串口空闲中断每触发一次且通过校验,则Vision_Get_New_Data置TRUE
  */
bool_t vision_if_update(void)
{
	return Vision_Get_New_Data;
}

/**
  * @brief  视觉数据更新标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void vision_clean_update_flag(void)
{
	Vision_Get_New_Data = FALSE;
}

/**
  * @brief  判断换装甲板了吗
  * @param  void
  * @retval TRUE换了   FALSE没换
  * @attention  为自动打符做准备,串口空闲中断每触发一次且通过校验,则Vision_Armor置TRUE
  */
bool_t vision_if_armor(void)
{
	return Vision_Armor;
}

/**
  * @brief  换装甲标志位手动置0(false)
  * @param  void
  * @retval void
  * @attention  记得要清零,在哪清零自己选,调用这个函数就行
  */
void vision_clean_ammorflag(void)
{
	Vision_Armor = FALSE;
}






void USART1_IRQHandler(void)
{
    static volatile uint8_t res;
    if(USART1->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);

        static uint16_t this_time_rx_len = 0;

        if ((huart1.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = VISION_BUFFER_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, VISION_BUFFER_LEN);
            huart1.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart1.hdmarx);

            vision_read_data(Vision_Buffer[0]);		//读取视觉数据
            memset(Vision_Buffer[0], 0, 200);    

        }
        else
        {
            __HAL_DMA_DISABLE(huart1.hdmarx);
            this_time_rx_len = VISION_BUFFER_LEN - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
            __HAL_DMA_SET_COUNTER(huart1.hdmarx, VISION_BUFFER_LEN);
            huart1.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);	
            __HAL_DMA_ENABLE(huart1.hdmarx);

            vision_read_data(Vision_Buffer[1]);		//读取视觉数据
			memset(Vision_Buffer[1], 0, 200);    //对象   内容  长度
			
        }
    }
}