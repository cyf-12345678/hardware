/****************************************Copyright (c)****************************************************
**     
** File name:               mq3.c
** Created by:              XiaoYi
** Created date:            2020-10-16
** Version:                 v1.0
** Descriptions:            The original 
** Link address:            https://blog.csdn.net/weixin_45006076
**
*********************************************************************************************************/

#include "drv_include.h"
#include "driver/adc.h"

#define TIMES 256

static const char *TAG = "alcohol";

static void delay_us(uint32_t cnt)
{
	usleep(cnt);
}

void alcohol_init (void)
{
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_0);  //IO2
}

 // MQ3传感器数据处理
float MQ3_GetPPM(int value)
{
	float temp = 0;
	printf("raw value :%d", value);
    temp=((float)value*(5.0/4096))*0.36-1.08;
	printf("value:%f",temp);

	if (temp < 0)
	{
		temp = 0.01;
	}

    return  temp * 10;   // mg/mL
}

float alcohol_single_read(void)
{
	int val = 0x00;
	int i = 0;
	float alcohol_value = 0;

	for(i = 0; i < 10; i++)
	{
		val += adc1_get_raw(ADC1_CHANNEL_2);
	}
	
	alcohol_value = MQ3_GetPPM(val/10);

	return alcohol_value;
}

