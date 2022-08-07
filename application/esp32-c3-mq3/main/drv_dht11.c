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

static const char *TAG = "DHT11";
#define CHECK_TIME 		28
 
 
static void delay_us(uint32_t cnt)
{
	usleep(cnt);
}

void dht11_gpio_init (void)
{
    gpio_reset_pin(DHT11_1_DATA_PINS);
    gpio_set_direction(DHT11_1_DATA_PINS, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(DHT11_1_DATA_PINS, GPIO_PULLUP_ONLY);
    gpio_set_level(DHT11_1_DATA_PINS, 1);
}
 
void dht11_mode_input(uint8_t num)
{
  gpio_set_direction(DHT11_1_DATA_PINS, GPIO_MODE_INPUT);
  gpio_set_pull_mode(DHT11_1_DATA_PINS, GPIO_PULLUP_ONLY);
}
 
void dht11_mode_output(uint8_t num)
{
  gpio_set_direction(DHT11_1_DATA_PINS, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(DHT11_1_DATA_PINS, GPIO_PULLUP_ONLY);
}

void dht11_DQ_High(uint8_t num)
{
	gpio_set_level(DHT11_1_DATA_PINS, 1);
}
 
 
void dht11_DQ_Low(uint8_t num)
{
	gpio_set_level(DHT11_1_DATA_PINS, 0);
}
 
void dht11_rst(uint8_t num)
{
	dht11_mode_output(num);
	dht11_DQ_Low(num);
	delay_us(22000);
	dht11_DQ_High(num);
	delay_us(25);
}
 
uint8_t dht11_gpio_read(uint8_t num)
{
	uint8_t gpio_val = 0x00;

	gpio_val = gpio_get_level(DHT11_1_DATA_PINS);

	return gpio_val;
}
 
uint8_t dht11_Check(uint8_t num)
{
	uint8_t retry = 0x00;
	dht11_mode_input(num);
	while((dht11_gpio_read(num) == 0x01)&&retry < 100)
	{
		retry++;
		delay_us(1);
	}
	if(retry >= 100)
	{
		//ESP_LOGD(TAG, "dht11_Check=1:num=%d\n", num);
		return 1;
	}
	else
	{
		retry = 0;
	}
	while((dht11_gpio_read(num) == 0x00)&&retry < 100)
	{
		retry++;
		delay_us(1);
	}
	if(retry >= 100)
	{
		//ESP_LOGD(TAG, "dht11_Check=1:num=%d\n", num);
		return 1;
	}
	//ESP_LOGD(TAG, "dht11_Check=0:num=%d\n", num);
	return 0;
}
 
uint8_t dht11_Read_Bit(uint8_t num)
{
	uint8_t retry = 0x00;
	while((dht11_gpio_read(num) == 1)&&retry < 100)
	{
		retry++;
		delay_us(1);
	}
	retry = 0;
	while((dht11_gpio_read(num) == 0)&&retry<100)
	{
		retry++;
		delay_us(1);
	}
	delay_us(45);
	if(dht11_gpio_read(num) == 1)
	{
		//ESP_LOGD(TAG, "dht11_Read_Bit=1:num=%d\n", num);
		return 1;
	}
	else
	{
		//ESP_LOGD(TAG, "dht11_Read_Bit=0:num=%d\n", num);
		return 0;
	}
}

uint8_t dht11_Read_Byte(uint8_t num)
{
	uint8_t i = 0;
	uint8_t dat = 0;
	for(i = 0; i < 8; i++)
	{
		dat <<= 1;
		dat|=dht11_Read_Bit(num);
	}
	//ESP_LOGD(TAG, "dht11_Read_Byte=0x%02x,num=%d\n", dat, num);
	return dat;
}
 
uint8_t dht11_Read_Data(float * temp, float * humi)
{
	uint8_t buf[5] = {0};
	uint8_t i,  num = 0;

	dht11_rst(num);
	if(dht11_Check(num) == 0)
	{
		for(i=0; i<5; i++)
		{
			buf[i] = dht11_Read_Byte(num);
		}
		//ESP_LOGD(TAG, "dht11_Read_Data:%d,%d,%d,%d,%d,num=%d\n", buf[0],buf[1],buf[2],buf[3],buf[4], num);
		if((buf[0]+buf[1]+buf[2]+buf[3]) == buf[4])
		{
			*humi = (float)(buf[0] * 100 + buf[1])/100;
			*temp = (float)(buf[2] * 100 + buf[3])/100;
		}
	}
	else
	{
		return 1;
	}
	return 0;
}

