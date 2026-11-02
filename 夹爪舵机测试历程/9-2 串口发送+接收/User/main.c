#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "LED.h"
#include "Key.h"
#include "OLED.h"
#include "PWM.h"
#include "Servo.h"
#include "Motor.h"
#include "Serial.h"
#include "Serial2.h"
#include "Serial3.h"
#include "TCRT5000.h"

uint8_t RxData;			//定义用于接收串口数据的变量
uint8_t KeyNum = 0;
uint8_t Case = 0;		//状态0：未启动，等待按键1；状态1：等待物品；状态2：分拣
uint16_t Angle = 0;		//（修复6：改用uint16_t，角度范围0~360，uint8_t装不下）
uint8_t ID;

/*位置值表：ID1~6对应角度0/60/120/180/240/300度（360度与0度重合），
  位置值 = 500 + 角度/360*2000，数值可按需修改*/
uint16_t ServoPosTable[7] = {0, 500, 833, 1167, 1500, 1833, 2167};

/*函数声明*/
void BusServo_SetID(uint8_t ID);

int main(void)
{
	uint16_t taget = 0;		//目标角度（声明必须在所有语句之前，Keil为C90模式；需跨圈保留上次的值）

	/*模块初始化*/
	OLED_Init();		//OLED初始化
	LED_Init();			//LED初始化（PA7、PA8）
	Key_Init();			//按键初始化（PB1、PB11）
	Serial_Init();		//串口一初始化（含中断一TIM4和中断二USART1的配置）
	Serial2_Init();		//串口二初始化（PA2、PA3）
	Serial3_Init();		//串口三初始化（PB10、PB11）
	Servo_Init();		//舵机初始化（TIM2，PA1）
	Motor_Init();		//直流电机初始化（TIM3，PB0，方向脚PA4/PA5）
	TCRT_Init();		//循迹传感器初始化（DO接PA0，AO接PA6，ADC1）
	/*显示静态字符串*/
	OLED_ShowString(1, 1, "Angle:");
	//Delay_ms(500);		//等待总线舵机上电自检完成，防止清零指令丢失
	Servo_SetAngle(50);		//PWM舵机推出物品
				 Delay_s(2);			//保持1秒（问题9：保留，不修改）
				Servo_SetAngle(150);		//PWM舵机收回
				 Delay_s(2);			//保持1秒（问题9：保留，不修改）
	BusServo_SetID(1);	//上电后总线舵机清零：发送#000P0500T1000!，转到0度
	while (1)
	{
		KeyNum = Key_GetNum();		//获取按键键码
		if (KeyNum != 0)						//有按键按下才发送，避免0x00刷屏
		{
			Serial3_SendByte(KeyNum + '0');		//发送字符'1'或'2'（加'0'转成ASCII可见字符）
		}
		if (KeyNum == 1)			//按键1按下：开始工作（必须按按键1才启动）
		{
			LED1_ON();	//LED1翻转
			Case = 1;				//从状态0进入状态1，开始工作
		}

		/*总线舵机角度回读与显示（每圈执行，与状态无关，三个状态都能看到实时角度）*/
		if(Serial_GetAngleFlag()==1)	//（修复5：角度解析完成的标志位）
		{
			Angle = Serial_GetAngle();	//（修复3：每圈更新角度，0~360）
			OLED_ShowString(1, 9, "    ");	//清除原有数据
			OLED_ShowNum(1, 9, Angle, 3);	//（修复8：用数字方式显示）
			Serial3_SendNumber( Angle, 3);
			Serial3_SendNumber( ID, 3);
		}

		/*状态机（修复1：移出按键判断，每圈都执行）*/
		switch(Case)
		{
			case 0:
								//状态0：未启动，等待按键1
			Motor_SetSpeed(0);		//皮带保持停止
			break;
			case 1:
			if( TCRT_Get()==0)		//循迹未检测到物品
			{
				LED2_OFF();			//未检测到物品，LED2熄灭
				Motor_SetSpeed(80);	//皮带慢转
			}//获取循迹传感器的数字检测结果
			if( TCRT_Get()==1)		//循迹检测到物品
			{
				LED2_ON();			//检测到物品，LED2点亮
				Case=2;				//进入分拣状态
			}break;
			case 2:
			Motor_SetSpeed(0);		//（修复2：物品到位，皮带停止，你已自己改好）
			if(Serial2_GetRxFlag() == 1)	//串口二收到ID
			{
				ID = Serial2_GetRxData() - '0';
				if (ID >= 1 && ID <= 6)			//ID有效才发送
				{
					BusServo_SetID(ID);			//按ID发送总线舵机指令
					 Delay_s(3);			//保持1秒（问题9：保留，不修改）Servo_SetAngle(80);		//PWM舵机推出物品
				Servo_SetAngle(50);		//PWM舵机推出物品
				 Delay_s(2);			//保持1秒（问题9：保留，不修改）
				Servo_SetAngle(180);		//PWM舵机收回
				 Delay_s(2);			//保持1秒（问题9：保留，不修改）
				ID = 0;					//清除旧ID，防止下一件物品误用旧ID分拣
				Case=1;					//分拣完成，回到等待物品状态
					//taget = (ID - 1) * 60;	//ID1~6精确对应0/60/120/180/240/300度
				}
			}
			/*if(ID >= 1 && ID <= 6 && Angle + 3 >= taget && Angle <= taget + 3)	//总线舵机转到目标角度（±5度容差，防止精度不够对不上）
			{
				Servo_SetAngle(80);		//PWM舵机推出物品
				 Delay_s(1);			//保持1秒（问题9：保留，不修改）
				Servo_SetAngle(170);		//PWM舵机收回
				 Delay_s(1);			//保持1秒（问题9：保留，不修改）
				ID = 0;					//清除旧ID，防止下一件物品误用旧ID分拣
				Case=1;					//分拣完成，回到等待物品状态
			}*/
			break;
		}
	}
}

/**
  * 函    数：总线舵机按ID转动
  * 参    数：ID 档位，范围1~6
  * 返 回 值：无
  * 注意事项：通过查表得到位置值，拼装ZP总线舵机指令#000PxxxxT1000!
  *           并从串口一发送，T1000表示用1秒时间转到目标位置
  */
void BusServo_SetID(uint8_t ID)
{
	uint16_t Pos;
	uint8_t Cmd[15];

	if (ID < 1 || ID > 6)				//ID超出范围，直接返回，不发送
	{
		return;
	}

	Pos = ServoPosTable[ID];			//查表得到位置值500~2167

	/*拼装指令：#000PxxxxT1000!*/
	Cmd[0] = '#';						//帧头
	Cmd[1] = '0';						//ID百位
	Cmd[2] = '0';						//ID十位
	Cmd[3] = '0';						//ID个位，000即0号舵机
	Cmd[4] = 'P';						//位置指令标志
	Cmd[5] = Pos / 1000 % 10 + '0';		//位置值千位
	Cmd[6] = Pos / 100 % 10 + '0';		//位置值百位
	Cmd[7] = Pos / 10 % 10 + '0';		//位置值十位
	Cmd[8] = Pos % 10 + '0';			//位置值个位
	Cmd[9] = 'T';						//时间指令标志
	Cmd[10] = '1';						//时间千位
	Cmd[11] = '0';						//时间百位
	Cmd[12] = '0';						//时间十位
	Cmd[13] = '0';						//时间个位，T1000即1000毫秒
	Cmd[14] = '!';						//帧尾

	TIM_ITConfig(TIM4, TIM_IT_Update, DISABLE);		//（修复7：发送期间暂停100毫秒查询中断，防止两条指令字节交插）
	Serial_SendArray(Cmd, sizeof(Cmd));				//从串口一发送给总线舵机
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);		//（修复7：发送完成，恢复查询中断）
}
