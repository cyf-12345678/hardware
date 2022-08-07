/****************************************Copyright (c)****************************************************
**     
** File name:               humi.c
** Created by:              XiaoYi
** Created date:            2020-10-16
** Version:                 v1.0
** Descriptions:            The original 
** Link address:            https://blog.csdn.net/weixin_45006076
**
*********************************************************************************************************/

#include "drv_include.h"
#include "driver/adc.h"

static const char *TAG = "Smoke";

static void delay_us(uint32_t cnt)
{
	usleep(cnt);
}

void smoke_init (void)
{
    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
}


#define CAL_PPM     20  // 校准环境中PPM值
#define RL			5		// RL阻值
static float        R0; // 元件在洁净空气中的阻值

/********************************************
 * 1.651428	          200               *
 * 1.437143	          300               *
 * 1.257143	          400               *
 * 1.137143	          500               *
 * 1		          600               *
 * 0.928704	          700               *
 * 0.871296	          800               *
 * 0.816667	          900               *
 * 0.785714	          1000              *
 * 0.574393	          2000              *
 * 0.466047	          3000              *
 * 0.415581	          4000              *
 * 0.370478	          5000              *
 * 0.337031	          6000              *
 * 0.305119	          7000              *
 * 0.288169	          8000              *
 * 0.272727	          9000              *
 * 0.254795	          10000             *
 *                                      *
 * ppm = 613.9f * pow(RS/RL, -2.074f)   *
 ***************************************/
 
 // 传感器校准函数
void MQ2_PPM_Calibration(float RS)
{
    R0 = RS / pow(CAL_PPM / 613.9f, 1 / -2.074f);
}

 // MQ2传感器数据处理
float MQ2_GetPPM(int value)
{
    float Vrl = 3.3f * value / 4095.f;
    float RS = (3.3f - Vrl) / Vrl * RL;
    
	// 获取系统执行时间，3s前进行校准
	MQ2_PPM_Calibration(RS);

    float ppm = 613.9f * pow(RS/R0, -2.074f);
    return  ppm;
}

int smoke_single_read(void)
{
	int val = 0x00;
	int i = 0;
	float smoke_value = 0;

	for(i = 0; i < 10; i++)
	{
		val += adc1_get_raw(ADC1_CHANNEL_0);
	}
	
	smoke_value = MQ2_GetPPM(val/10);

	return smoke_value;
}

void sensor_light_init (void)
{
    adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_6);
}

int sensor_light_single_read(void)
{
	int val = 0x00;
	int i = 0;
	float average_value = 0;

	for(i = 0; i < 10; i++)
	{
		val += adc1_get_raw(ADC1_CHANNEL_1);
	}
	 average_value = val/10;

	return 100- (int)(average_value/4096 * 100);   // 1-100， 数值越大，光强越高
}

