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
#include "driver/i2c.h"

static const char *TAG = "OLED";
 
user_i2c_t  user_i2c_oled = { 0x00 };

void user_bsp_delay_us(int us)
{
	ets_delay_us(us);
}

void user_mode_set_cb(int pin, int mode)
{
    if (mode)
    {
        gpio_set_direction(pin, GPIO_MODE_INPUT);
    }else
    {
        gpio_set_direction(pin, GPIO_MODE_OUTPUT);
    }
}

void user_bsp_pin_write(int pin, int enable)
{
    gpio_set_level(pin, enable);
}

int user_bsp_pin_read(int pin)
{
    return gpio_get_level(pin);
}

//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
uint8_t oled_write_cmd( uint8_t data) 				 
{ 
    user_i2c_start(&user_i2c_oled); 
    
	user_i2c_send_byte(&user_i2c_oled, USER_OLED_ADDR);//发送器件地址+写命令	
	if(user_i2c_wait_ack(&user_i2c_oled))	//等待应答
	{
		user_i2c_stop(&user_i2c_oled);	 
		return 1;		
	}
    user_i2c_send_byte(&user_i2c_oled, 0x00);	//写寄存器地址
    user_i2c_wait_ack(&user_i2c_oled);		//等待应答 
	user_i2c_send_byte(&user_i2c_oled, data);//发送数据
	if(user_i2c_wait_ack(&user_i2c_oled))	//等待ACK
	{
		user_i2c_stop(&user_i2c_oled); 
		return 1;		 
	}		 
    user_i2c_stop(&user_i2c_oled); 
	return 0;
}

uint8_t oled_write_data( uint8_t data) 				 
{ 
    user_i2c_start(&user_i2c_oled); 
    
	user_i2c_send_byte(&user_i2c_oled, USER_OLED_ADDR);//发送器件地址+写命令	
	if(user_i2c_wait_ack(&user_i2c_oled))	//等待应答
	{
		user_i2c_stop(&user_i2c_oled);	 
		return 1;		
	}
    user_i2c_send_byte(&user_i2c_oled, 0x40);	//写寄存器地址
    user_i2c_wait_ack(&user_i2c_oled);		//等待应答 
	user_i2c_send_byte(&user_i2c_oled, data);//发送数据
	if(user_i2c_wait_ack(&user_i2c_oled))	//等待ACK
	{
		user_i2c_stop(&user_i2c_oled); 
		return 1;		 
	}		 
    user_i2c_stop(&user_i2c_oled); 
	return 0;
}


int user_oled_init(void)
{
    uint8_t res = 0;

    gpio_reset_pin(OLED_SCL_PIN);
    gpio_reset_pin(OLED_SDA_PIN);

    user_i2c_oled.addr = USER_OLED_ADDR;
    user_i2c_oled.pin_scl = OLED_SCL_PIN;
    user_i2c_oled.pin_sda = OLED_SDA_PIN;
    user_i2c_oled.delay_cb = user_bsp_delay_us;
    user_i2c_oled.mode_set_cb = user_mode_set_cb;
    user_i2c_oled.pin_write_cb = user_bsp_pin_write;
    user_i2c_oled.pin_read_cb = user_bsp_pin_read;
    user_i2c_init(&user_i2c_oled);

	vTaskDelay(100 / portTICK_RATE_MS);

    oled_write_cmd(0xAE); //display off
    oled_write_cmd(0x20); //Set Memory Addressing Mode    
    oled_write_cmd(0x10); //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    oled_write_cmd(0xb0); //Set Page Start Address for Page Addressing Mode,0-7
    oled_write_cmd(0xc8); //Set COM Output Scan Direction
    oled_write_cmd(0x00); //---set low column address
    oled_write_cmd(0x10); //---set high column address
    oled_write_cmd(0x40); //--set start line address
    oled_write_cmd(0x81); //--set contrast control register
    oled_write_cmd(0xff); //亮度调节 0x00~0xff
    oled_write_cmd(0xa1); //--set segment re-map 0 to 127
    oled_write_cmd(0xa6); //--set normal display
    oled_write_cmd(0xa8); //--set multiplex ratio(1 to 64)
    oled_write_cmd(0x3F); //
    oled_write_cmd(0xa4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    oled_write_cmd(0xd3); //-set display offset
    oled_write_cmd(0x00); //-not offset
    oled_write_cmd(0xd5); //--set display clock divide ratio/oscillator frequency
    oled_write_cmd(0xf0); //--set divide ratio
    oled_write_cmd(0xd9); //--set pre-charge period
    oled_write_cmd(0x22); //
    oled_write_cmd(0xda); //--set com pins hardware configuration
    oled_write_cmd(0x12);
    oled_write_cmd(0xdb); //--set vcomh
    oled_write_cmd(0x20); //0x20,0.77xVcc
    oled_write_cmd(0x8d); //--set DC-DC enable
    oled_write_cmd(0x14); //
    oled_write_cmd(0xaf); //--turn on oled panel
	return 0;
}

 /**
  * @brief  OLED_ON，将OLED从休眠中唤醒
  * @param  无
    * @retval 无
  */
void user_oled_on(void)
{
    oled_write_cmd(0X8D);  //设置电荷泵
    oled_write_cmd(0X14);  //开启电荷泵
    oled_write_cmd(0XAF);  //OLED唤醒
}


 /**
  * @brief  oled_setPos，设置光标
  * @param  x,光标x位置
    *                   y，光标y位置
  * @retval 无
  */
void oled_setPos(uint8_t x, uint8_t y) //设置起始点坐标
{ 
    oled_write_cmd(0xb0+y);
    oled_write_cmd(((x&0xf0)>>4)|0x10);
    oled_write_cmd((x&0x0f)|0x01);
}

 /**
  * @brief  OLED_Fill，填充整个屏幕
  * @param  fill_Data:要填充的数据
    * @retval 无
  */
void user_oled_fill(uint8_t fill_Data) //全屏填充
{
    uint8_t m, n;
    for(m=0; m<8; m++)
    {
        oled_write_cmd(0xb0+m);       //page0-page1
        oled_write_cmd(0x00);     //low column start address
        oled_write_cmd(0x10);     //high column start address
        for(n=0;n<128;n++)
            {
                oled_write_data(fill_Data);
            }
    }
}

void user_oled_clear(void)//清屏
{
    user_oled_fill(0x00);
}

/**
  * @brief  user_oled_show_string，显示codetab.h中的ASCII字符,有6*8和8*16可选择
  * @param  x,y : 起始点坐标(x:0~127, y:0~7);
    *                   ch[] :- 要显示的字符串; 
    *                   TextSize : 字符大小(1:6*8 ; 2:8*16)
    * @retval 无
  */
void user_oled_show_string(uint8_t x, uint8_t y, uint8_t ch[], uint8_t TextSize)
{
    uint8_t c = 0,i = 0,j = 0;
    switch(TextSize)
    {
        case 1:
        {
            while(ch[j] != '\0')
            {
                c = ch[j] - 32;
                if(x > 126)
                {
                    x = 0;
                    y++;
                }
                oled_setPos(x,y);
                for(i=0;i<6;i++)
                    oled_write_data(F6x8[c][i]);
                x += 6;
                j++;
            }
        }break;
        case 2:
        {
            while(ch[j] != '\0')
            {
                c = ch[j] - 32;
                if(x > 120)
                {
                    x = 0;
                    y++;
                }
                oled_setPos(x,y);
                for(i=0;i<8;i++)
                    oled_write_data(F8X16[c*16+i]);
                oled_setPos(x,y+1);
                for(i=0;i<8;i++)
                    oled_write_data(F8X16[c*16+i+8]);
                x += 8;
                j++;
            }
        }break;
    }
}


//--------------------------------------------------------------
// Prototype      : void oled_show_chinese(unsigned char x, unsigned char y, unsigned char N)
// Calls          : 
// Parameters     : x,y -- 起始点坐标(x:0~127, y:0~7); N:汉字在codetab.h中的索引
// Description    : 显示codetab.h中的汉字,16*16点阵
//--------------------------------------------------------------
void user_oled_show_chinese(unsigned char x, unsigned char y, unsigned char N)
{
	unsigned char wm=0;
	unsigned int  adder=32*N;
	oled_setPos(x , y);
	for(wm = 0;wm < 16;wm++)
	{
		oled_write_data(F16x16[adder]);
		adder += 1;
	}
	oled_setPos(x,y + 1);
	for(wm = 0;wm < 16;wm++)
	{
		oled_write_data(F16x16[adder]);
		adder += 1;
	}
}

//--------------------------------------------------------------
// Prototype      : void oled_draw_bmp(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);
// Calls          : 
// Parameters     : x0,y0 -- 起始点坐标(x0:0~127, y0:0~7); x1,y1 -- 起点对角线(结束点)的坐标(x1:1~128,y1:1~8)
// Description    : 显示BMP位图
//--------------------------------------------------------------
void user_oled_draw_bmp(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char *bmp)
{
	unsigned int j=0;
	unsigned char x,y;

  if(y1%8==0)
		y = y1/8;
  else
		y = y1/8 + 1;
	for(y=y0;y<y1;y++)
	{
		oled_setPos(x0,y);
    for(x=x0;x<x1;x++)
		{
			oled_write_data(bmp[j++]);
		}
	}
}

