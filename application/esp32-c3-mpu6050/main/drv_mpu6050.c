/******************************************************************************
 * 作者：xiaoyi
 * 时间：2021-08-08
 * 源码地址：https://gitee.com/kerwincui/wumei-smart
 * author: xiaoyi
 * create: 2021-08-08
 * source:https://github.com/kerwincui/wumei-smart
 ******************************************************************************/
#include "drv_include.h"
#include "driver/gpio.h"


user_i2c_t  i2c_mpu6050 = { 0x00 };
user_mpu6050_t user_mpu6050 = { 0x00 };

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

//设置MPU6050陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr<<3);//设置陀螺仪满量程范围  
}
//设置MPU6050加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}
//设置MPU6050的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU_CFG_REG,data);//设置数字低通滤波器  
}
//设置MPU6050的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

//得到温度值
//返回值:温度值(扩大了100倍)
short MPU_Get_Temperature(void)
{
    uint8_t buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((uint16_t)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100;;
}
//得到陀螺仪值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((uint16_t)buf[0]<<8)|buf[1];  
		*gy=((uint16_t)buf[2]<<8)|buf[3];  
		*gz=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((uint16_t)buf[0]<<8)|buf[1];  
		*ay=((uint16_t)buf[2]<<8)|buf[3];  
		*az=((uint16_t)buf[4]<<8)|buf[5];
	} 	
    return res;;
}
//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
	uint8_t i; 
    user_i2c_start(&i2c_mpu6050); 
	user_i2c_send_byte(&i2c_mpu6050, (addr<<1)|0);//发送器件地址+写命令	
	if(user_i2c_wait_ack(&i2c_mpu6050))	//等待应答
	{
		user_i2c_stop(&i2c_mpu6050);	 
		return 1;		
	}
    user_i2c_send_byte(&i2c_mpu6050, reg);	//写寄存器地址
    user_i2c_wait_ack(&i2c_mpu6050);		//等待应答
	for(i=0;i<len;i++)
	{
		user_i2c_send_byte(&i2c_mpu6050, buf[i]);	//发送数据
		if(user_i2c_wait_ack(&i2c_mpu6050))		//等待ACK
		{
			user_i2c_stop(&i2c_mpu6050); 
			return 1;		 
		}		
	}    
    user_i2c_stop(&i2c_mpu6050); 
	return 0;	
} 
//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{ 
 	user_i2c_start(&i2c_mpu6050); 
	user_i2c_send_byte(&i2c_mpu6050, (addr<<1)|0);//发送器件地址+写命令	
	if(user_i2c_wait_ack(&i2c_mpu6050))	//等待应答
	{
		user_i2c_stop(&i2c_mpu6050);	 
		return 1;		
	}
    user_i2c_send_byte(&i2c_mpu6050, reg);	//写寄存器地址
    user_i2c_wait_ack(&i2c_mpu6050);		//等待应答
    user_i2c_start(&i2c_mpu6050);
	user_i2c_send_byte(&i2c_mpu6050, (addr<<1)|1);//发送器件地址+读命令	
    user_i2c_wait_ack(&i2c_mpu6050);		//等待应答 
	while(len)
	{
		if(len==1)
            *buf=i2c_read_byte(&i2c_mpu6050, 0);//读数据,发送nACK 
		else 
            *buf=i2c_read_byte(&i2c_mpu6050, 1);		//读数据,发送ACK  
		len--;
		buf++; 
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}    
    user_i2c_stop(&i2c_mpu6050);//产生一个停止条件 
	return 0;	
}
//IIC写一个字节 
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data) 				 
{ 
    user_i2c_start(&i2c_mpu6050); 
    
	user_i2c_send_byte(&i2c_mpu6050, (MPU_ADDR<<1)|0);//发送器件地址+写命令	
	if(user_i2c_wait_ack(&i2c_mpu6050))	//等待应答
	{
		user_i2c_stop(&i2c_mpu6050);	 
		return 1;		
	}
    user_i2c_send_byte(&i2c_mpu6050, reg);	//写寄存器地址
    user_i2c_wait_ack(&i2c_mpu6050);		//等待应答 
	user_i2c_send_byte(&i2c_mpu6050, data);//发送数据
	if(user_i2c_wait_ack(&i2c_mpu6050))	//等待ACK
	{
		user_i2c_stop(&i2c_mpu6050); 
		return 1;		 
	}		 
    user_i2c_stop(&i2c_mpu6050); 
	return 0;
}
//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
uint8_t MPU_Read_Byte(uint8_t reg)
{
	uint8_t res;
    user_i2c_start(&i2c_mpu6050); 
	user_i2c_send_byte(&i2c_mpu6050, (MPU_ADDR<<1)|0);//发送器件地址+写命令	
	user_i2c_wait_ack(&i2c_mpu6050);		//等待应答 
    user_i2c_send_byte(&i2c_mpu6050, reg);	//写寄存器地址
    user_i2c_wait_ack(&i2c_mpu6050);		//等待应答
    user_i2c_start(&i2c_mpu6050);
	user_i2c_send_byte(&i2c_mpu6050, (MPU_ADDR<<1)|1);//发送器件地址+读命令	
    user_i2c_wait_ack(&i2c_mpu6050);		//等待应答 
	res=i2c_read_byte(&i2c_mpu6050, 0);//读取数据,发送nACK 
    user_i2c_stop(&i2c_mpu6050);		//产生一个停止条件 
	return res;		
}

int mpu6050_init(void)
{
    uint8_t res = 0;

    gpio_reset_pin(MPU6050_SCL);
    gpio_reset_pin(MPU6050_SDA);

    i2c_mpu6050.addr = 0x68;
    i2c_mpu6050.pin_scl = MPU6050_SCL;
    i2c_mpu6050.pin_sda = MPU6050_SDA;
    i2c_mpu6050.delay_cb = user_bsp_delay_us;
    i2c_mpu6050.mode_set_cb = user_mode_set_cb;
    i2c_mpu6050.pin_write_cb = user_bsp_pin_write;
    i2c_mpu6050.pin_read_cb = user_bsp_pin_read;
    user_i2c_init(&i2c_mpu6050);

	if (MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80))	//复位MPU6050
	{
		printf("write 0x6B error\r\n");
	}
	vTaskDelay(100 / portTICK_RATE_MS);
	MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00);	//唤醒MPU6050 
	MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	MPU_Set_Rate(50);						//设置采样率50Hz
	MPU_Write_Byte(MPU_INT_EN_REG, 0X00);	//关闭所有中断
	MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00);	//I2C主模式关闭
	MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);	//关闭FIFO
	MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80);	//INT引脚低电平有效
	res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	if(res==MPU_ADDR)//器件ID正确
	{
		MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
		MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
		MPU_Set_Rate(50);						//设置采样率为50Hz
 	}else 
	 {
		 printf("recv id is :%02x\r\n", res);
		 return 1;
	 }

	return 0;
}

void mpu6050_sample_thread(void *param)
{
    short temp = 0, count = 0;
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据


	if (mpu6050_init())
	{
		printf("init mpu6050 error!\r\n");
		return ;
	}else
	{
		printf("init mpu6050 success!\r\n");
	}

	count = 0;
	while (count < 60)
	{
		MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
		printf("aacx:%d, aacy:%d, aacz:%d\r\n", aacx, aacy, aacz);
		vTaskDelay(2000 / portTICK_RATE_MS);
		count ++;
	}

	vTaskDelete(NULL);
}

