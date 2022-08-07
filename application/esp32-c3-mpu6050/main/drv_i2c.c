/******************************************************************************
 * 作者：xiaoyi
 * 时间：2021-08-08
 * 源码地址：https://gitee.com/kerwincui/wumei-smart
 * author: xiaoyi
 * create: 2021-08-08
 * source:https://github.com/kerwincui/wumei-smart
 ******************************************************************************/
#include "drv_include.h"


//产生IIC起始信号
void user_i2c_start(user_i2c_t  *i2c)
{
	i2c->mode_set_cb(i2c->pin_sda, 0);     //sda线输出
	i2c->pin_write_cb(i2c->pin_sda,  1);	  	  
	i2c->pin_write_cb(i2c->pin_scl,  1);
	i2c->delay_cb(3);
 	i2c->pin_write_cb(i2c->pin_sda,  0);//START:when CLK is high,DATA change form high to low 
	i2c->delay_cb(3);
	i2c->pin_write_cb(i2c->pin_scl,  0);//钳住I2C总线，准备发送或接收数据 
}

void user_i2c_stop(user_i2c_t  *i2c)
{
	i2c->mode_set_cb(i2c->pin_sda, 0); //sda线输出
	i2c->pin_write_cb(i2c->pin_scl, 0);
	i2c->pin_write_cb(i2c->pin_sda,  0); //STOP:when CLK is high DATA change form low to high
 	i2c->delay_cb(3);
	i2c->pin_write_cb(i2c->pin_scl,  1);
	i2c->pin_write_cb(i2c->pin_sda,  1);//发送I2C总线结束信号
	i2c->delay_cb(3);
}

//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t user_i2c_wait_ack(user_i2c_t  *i2c)
{
	uint8_t ucErrTime=0;
	i2c->mode_set_cb(i2c->pin_sda, 1);      //SDA设置为输入  
	i2c->pin_write_cb(i2c->pin_sda,  1);
    i2c->delay_cb(3);	   
	i2c->pin_write_cb(i2c->pin_scl,  1);
    i2c->delay_cb(3);	 
	while(i2c->pin_read_cb(i2c->pin_sda))
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			user_i2c_stop(i2c);
			return 1;
		}
	}
	i2c->pin_write_cb(i2c->pin_scl,  0);//时钟输出0 	   
	return 0;  
}

void user_i2c_ack(user_i2c_t  *i2c)
{
	i2c->pin_write_cb(i2c->pin_scl,  0);
	i2c->mode_set_cb(i2c->pin_sda, 0);
	i2c->pin_write_cb(i2c->pin_sda,  0);
	i2c->delay_cb(3);
	i2c->pin_write_cb(i2c->pin_scl,  1);
	i2c->delay_cb(3);
	i2c->pin_write_cb(i2c->pin_scl,  0);
}
    
void user_i2c_nack(user_i2c_t  *i2c)
{
	i2c->pin_write_cb(i2c->pin_scl,  0);
	i2c->mode_set_cb(i2c->pin_sda, 0);
	i2c->pin_write_cb(i2c->pin_sda,  1);
	i2c->delay_cb(3);
	i2c->pin_write_cb(i2c->pin_scl,  1);
	i2c->delay_cb(3);
	i2c->pin_write_cb(i2c->pin_scl,  0);
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void user_i2c_send_byte(user_i2c_t  *i2c, uint8_t txd)
{                        
    uint8_t t;   
	i2c->mode_set_cb(i2c->pin_sda, 0); 	    
    i2c->pin_write_cb(i2c->pin_scl,  0);//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        i2c->pin_write_cb(i2c->pin_sda,  (txd&0x80)>>7);
        txd<<=1; 	  
		i2c->pin_write_cb(i2c->pin_scl,  1);
		i2c->delay_cb(3); 
		i2c->pin_write_cb(i2c->pin_scl,  0);	
		i2c->delay_cb(3);
    }	 
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t i2c_read_byte(user_i2c_t *i2c, unsigned char ack)
{
	unsigned char i,receive=0;
	i2c->mode_set_cb(i2c->pin_sda, 1);//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        i2c->pin_write_cb(i2c->pin_scl,  0); 
        i2c->delay_cb(3);
		i2c->pin_write_cb(i2c->pin_scl,  1);
        receive<<=1;
        if(i2c->pin_read_cb(i2c->pin_sda))
        {
            receive++;
        }   
		i2c->delay_cb(3); 
    }					 
    if (!ack)
        user_i2c_nack(i2c);//发送nACK
    else
        user_i2c_ack(i2c); //发送ACK   
    return receive;
}

//初始化IIC
void user_i2c_init(user_i2c_t  *i2c)
{					     
    ETS_ASSERT(i2c->delay_cb && i2c->mode_set_cb && i2c->pin_read_cb && i2c->pin_write_cb);

    i2c->mode_set_cb(i2c->pin_scl, 0);  // output
    i2c->mode_set_cb(i2c->pin_sda, 0);
}

