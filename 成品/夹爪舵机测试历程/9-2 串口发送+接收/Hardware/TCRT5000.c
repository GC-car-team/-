#include "stm32f10x.h"                  // Device header

/*引脚说明（可按实际接线修改）：
  DO（模块数字输出）接PA0，上拉输入——与出厂程序循迹传感器左路位置一致
  AO（模块模拟输出）接PA6，ADC1通道6——模拟量测量反射强度/距离
  注意：PA4已被电机方向脚IN1占用，故AO改用PA6，避免引脚冲突
  其余引脚占用见工程内其他模块注释*/

/**
  * 函    数：TCRT5000红外反射传感器初始化
  * 参    数：无
  * 返 回 值：无
  */
void TCRT_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);	//开启ADC1的时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);						//ADC时钟6分频，72M/6=12MHz，在ADC允许范围内

	/*GPIO初始化：PA0接模块DO，上拉输入（读数字量）*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA0引脚初始化为上拉输入

	/*GPIO初始化：PA6接模块AO，模拟输入（读模拟量）*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA6引脚初始化为模拟输入

	/*ADC初始化*/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_55Cycles5);	//规则组通道6，采样时间55.5周期

	ADC_InitTypeDef ADC_InitStructure;						//定义结构体变量
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;		//模式，选择独立模式，即单独使用ADC1
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//数据对齐，选择右对齐
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//外部触发，使用软件触发，不需要
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;		//连续转换，失能，每测一次触发一次
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;			//扫描模式，失能，只测规则组通道1这一个位置
	ADC_InitStructure.ADC_NbrOfChannel = 1;					//通道数，为1，仅在扫描模式下，才需要指定大于1的数
	ADC_Init(ADC1, &ADC_InitStructure);						//将结构体变量交给ADC_Init，配置ADC1

	/*ADC校准*/
	ADC_Cmd(ADC1, ENABLE);									//使能ADC1
	ADC_ResetCalibration(ADC1);								//固定流程，内部有电路会自动执行校准
	while (ADC_GetResetCalibrationStatus(ADC1) == SET);		//等待校准复位完成
	ADC_StartCalibration(ADC1);								//开始校准
	while (ADC_GetCalibrationStatus(ADC1) == SET);			//等待校准完成
}

/**
  * 函    数：获取TCRT5000的数字检测结果
  * 参    数：无
  * 返 回 值：1表示检测到物体（反射面），0表示未检测到
  * 注意事项：默认按模块"检测到物体时DO输出低电平"处理，
  *           如果你的模块极性相反，把下面 == 0 改成 == 1 即可
  */
uint8_t TCRT_Get(void)
{
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) == 0)		//读PA0输入寄存器的状态，为0代表检测到物体
	{
		return 1;											//检测到物体，返回1
	}
	return 0;												//未检测到，返回0
}

/**
  * 函    数：获取TCRT5000的模拟量原始值
  * 参    数：无
  * 返 回 值：ADC原始值，范围：0~4095，数值越大说明反射越强（物体越近/表面越白）
  */
uint16_t TCRT_GetValue(void)
{
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);					//软件触发ADC1开始转换
	while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);	//等待转换完成
	return ADC_GetConversionValue(ADC1);					//返回ADC转换结果
}
