#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>

uint8_t Serial_RxData;		//定义串口接收的数据变量
uint8_t Serial_RxFlag;		//定义串口接收的标志位变量

uint16_t Serial_Angle;		//定义保存舵机角度值的变量（中断二解析出回传数据并换算后存入此变量，范围0~360）
uint8_t Serial_AngleFlag;	//定义角度更新标志位变量（存入新角度后置1）

uint8_t Serial_AngleRxBuf[32];	//定义角度帧接收缓冲区（中断二攒帧用）
uint8_t Serial_RxCount;			//定义角度帧已接收的字节数

/**
  * 回读角度指令（中断一每隔100毫秒发送一次的内容）
  * ZP总线舵机指令格式：# + 3位ID + 指令 + !，例如：
  * #000PRAD!	读取0号舵机当前角度，舵机回传 #000P1500!（1500为位置值，范围500~2500）
  */
uint8_t Serial_ReadAngleCmd[] = {'#', '0', '0', '0', 'P', 'R', 'A', 'D', '!'};	//回读角度指令：读取0号舵机角度

/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
void Serial_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//开启USART1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA9引脚初始化为复用推挽输出
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA10引脚初始化为上拉输入
	
	/*USART初始化*/
	USART_InitTypeDef USART_InitStructure;					//定义结构体变量
	USART_InitStructure.USART_BaudRate = 115200;				//波特率，与ZP总线舵机默认波特率一致
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制，不需要
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//模式，发送模式和接收模式均选择
	USART_InitStructure.USART_Parity = USART_Parity_No;		//奇偶校验，不需要
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位，选择1位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长，选择8位
	USART_Init(USART1, &USART_InitStructure);				//将结构体变量交给USART_Init，配置USART1
	
	/*中断输出配置*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
	
	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//配置NVIC为分组2
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;					//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		//选择配置NVIC的USART1线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		//指定NVIC线路的抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);							//将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*定时器初始化（中断一）：TIM4每100毫秒产生一次更新中断，定时发送回读角度指令
	  注：TIM2已被舵机PWM占用，故此中断定时器改用TIM4，避免冲突*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//开启TIM4的时钟，注意TIM4在APB1总线上

	TIM_InternalClockConfig(TIM4);							//选择内部时钟作为时基单元的时钟源

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;		//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//时钟分频，选择不分频
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;	//计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 1000 - 1;		//ARR，自动重装值，定时100毫秒
	TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;		//PSC，预分频值，分频后计数频率10kHz
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;	//重复计数器，高级定时器专用，这里置0即可
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);		//将结构体变量交给TIM_TimeBaseInit，配置TIM4

	TIM_ClearFlag(TIM4, TIM_FLAG_Update);					//清除TIM4的更新中断标志位
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);				//开启TIM4的更新中断（中断一）

	/*NVIC配置（中断一）*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;			//选择配置NVIC的TIM4线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		//指定NVIC线路的抢占优先级为0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);							//将结构体变量交给NVIC_Init，配置NVIC外设
															//优先级关系：串口二（0/0）> 中断一TIM4（0/1）
															//> 串口三（1/0）> 中断二串口一（1/1）

	TIM_Cmd(TIM4, ENABLE);									//使能TIM4，定时器开始运行

	/*USART使能*/
	USART_Cmd(USART1, ENABLE);								//使能USART1，串口开始运行
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	//设置结果初值为1
	while (Y --)			//执行Y次
	{
		Result *= X;		//将X累乘到结果
	}
	return Result;
}

/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}

/**
  * 函    数：使用printf需要重定向的底层函数
  * 参    数：保持原始格式即可，无需变动
  * 返 回 值：保持原始格式即可，无需变动
  */
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
	return ch;
}

/**
  * 函    数：自己封装的prinf函数
  * 参    数：format 格式化字符串
  * 参    数：... 可变的参数列表
  * 返 回 值：无
  */
void Serial_Printf(char *format, ...)
{
	char String[100];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	Serial_SendString(String);		//串口发送字符数组（字符串）
}

/**
  * 函    数：获取串口接收标志位
  * 参    数：无
  * 返 回 值：串口接收标志位，范围：0~1，接收到数据后，标志位置1，读取后标志位自动清零
  */
uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)			//如果标志位为1
	{
		Serial_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}

/**
  * 函    数：获取串口接收的数据
  * 参    数：无
  * 返 回 值：接收的数据，范围：0~255
  */
uint8_t Serial_GetRxData(void)
{
	return Serial_RxData;			//返回接收的数据变量
}

/**
  * 函    数：从文本帧中解析角度值（内部使用，由USART1中断函数调用）
  * 功    能：在帧缓冲区内查找'P'后面连续的数字，作为位置值解析出来，
  *           再换算成角度值存入Serial_Angle
  * 例    如：ZP总线舵机回传 #000P1500! 中的 1500 即为位置值，
  *           位置值500~2500对应角度0~360度：角度 = (位置值 - 500) * 360 / 2000
  * 返 回 值：无
  */
void Serial_ParseAngle(void)
{
	uint8_t i;
	uint16_t Pos = 0;
	for (i = 0; i < Serial_RxCount; i ++)					//遍历帧缓冲区的每个字节
	{
		if (Serial_AngleRxBuf[i] == 'P')					//找到位置值标志'P'
		{
			while (i + 1 < Serial_RxCount && Serial_AngleRxBuf[i + 1] >= '0' && Serial_AngleRxBuf[i + 1] <= '9')
			{
				Pos = Pos * 10 + (Serial_AngleRxBuf[i + 1] - '0');	//累加'P'后面的数字
				i ++;
			}
			if (Pos >= 500 && Pos <= 2500)					//位置值在有效范围内，才认为解析成功
			{
				Serial_Angle = (uint16_t)((Pos - 500) * 360 / 2000);	//换算成角度值0~360并存入变量
				Serial_AngleFlag = 1;						//置角度更新标志位为1
			}
			return;											//解析完成，直接返回
		}
	}
}

/**
  * 函    数：获取角度更新标志位
  * 参    数：无
  * 返 回 值：角度更新标志位，范围：0~1，收到新角度后置1，读取后标志位自动清零
  */
uint8_t Serial_GetAngleFlag(void)
{
	if (Serial_AngleFlag == 1)		//如果标志位为1
	{
		Serial_AngleFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}

/**
  * 函    数：获取串口一收到的角度值
  * 参    数：无
  * 返 回 值：舵机角度值，范围：0~360（已由位置值500~2500换算，500对应0度，2500对应360度）
  */
uint16_t Serial_GetAngle(void)
{
	return Serial_Angle;			//返回角度值变量
}

/**
  * 函    数：USART1中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)		//判断是否是USART1的接收事件触发的中断
	{
		uint8_t Byte = USART_ReceiveData(USART1);				//读取数据寄存器（读取后自动清除RXNE）
																//此为中断二：每次收到中断一发出的回读指令
																//经串口一传回来的数据，就攒帧并解析角度值

		Serial_RxData = Byte;									//兼容原有的单字节接收：数据存入接收数据变量
		Serial_RxFlag = 1;										//并置接收标志位变量为1

		/*中断二：把传回来的数据攒成文本帧，帧结束时解析出角度值存入变量*/
		if (Byte == '!' || Byte == '\n')						//收到帧结尾（!或换行），本帧接收完成
		{
			Serial_ParseAngle();								//从帧中解析出角度值，存入Serial_Angle
			Serial_RxCount = 0;									//清零接收计数，准备接收下一帧
		}
		else if (Serial_RxCount < sizeof(Serial_AngleRxBuf) - 1)	//帧缓冲区未满
		{
			Serial_AngleRxBuf[Serial_RxCount ++] = Byte;		//字节存入帧缓冲区
		}
		else													//帧缓冲区溢出，丢弃本帧重新开始
		{
			Serial_RxCount = 0;
		}
	}
}

/**
  * 函    数：TIM4中断函数（中断一）
  * 功    能：每隔100毫秒向外（串口一）发送一次回读角度指令
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)			//判断是否是TIM4的更新事件触发的中断
	{
		Serial_SendArray(Serial_ReadAngleCmd, sizeof(Serial_ReadAngleCmd));	//发送回读角度指令
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);				//清除TIM4的更新中断标志位
	}
}
