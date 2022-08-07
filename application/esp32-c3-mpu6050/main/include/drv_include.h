/******************************************************************************
 * 作者：kerwincui
 * 时间：2021-06-08
 * 邮箱：164770707@qq.com
 * 源码地址：https://gitee.com/kerwincui/wumei-smart
 * author: kerwincui
 * create: 2021-06-08
 * email：164770707@qq.com
 * source:https://github.com/kerwincui/wumei-smart
 ******************************************************************************/
#ifndef _DRV_INCLUDE_H_ 
#define _DRV_INCLUDE_H_


#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include <freertos/semphr.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_spiffs.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "unity.h"
#include "iot_button.h"

#include "driver/i2c.h"
#include "driver/gpio.h"


#define MPU6050_SCL            GPIO_NUM_5
#define MPU6050_SDA            GPIO_NUM_4

// DHT11 连接引脚定义  
#define 	DHT11_1_DATA_PINS			GPIO_NUM_3

/************************************ drv_button ************************************/

bool is_open;

void button_start(void);
void rf_receiver_start(void);
void radar_start(void);

//获取设备温度
void get_device_temp(void);


/************************************ drv_temp ************************************/

//##被称为连接符（concatenator），用来将两个Token连接为一个Token。
#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

// #define I2C_MASTER_NUM I2C_NUMBER(0)         // I2C主机端口号
#define I2C_MASTER_FREQ_HZ 100000            // I2C主机时钟频率
#define I2C_MASTER_TX_BUF_DISABLE 0          //I2C主机不需要缓冲区
#define I2C_MASTER_RX_BUF_DISABLE 0          //I2C主机不需要缓冲区

#define WRITE_BIT I2C_MASTER_WRITE           // I2C主机写   
#define READ_BIT I2C_MASTER_READ             // I2C主机读
#define ACK_CHECK_EN 0x1                     // I2C主机将检查从机的ack
#define ACK_CHECK_DIS 0x0                    // I2C主机将不检查从机的ack 
#define ACK_VAL 0x0                          // I2C ack值
#define NACK_VAL 0x1                         // I2C nack值

//SHT30
#define SHT30_WRITE_ADDR 0x44 //地址
#define CMD_FETCH_DATA_H 0x22 //循环采样，参考sht30 datasheet
#define CMD_FETCH_DATA_L 0x36

void i2c_temp_start(void);
//获取空气温湿度
void get_temp_humi();

/************************************ drv_common.c ************************************/

#define IO_SENSOR_SCL      1   // IIC-SCL
#define IO_SENSOR_SDA      2   // IIC-SDA
#define IO_RELAY           7   // 继电器
#define IO_RELAY_MQ3       6   // 继电器


// 电源灯：红色（接通电源）、 绿色（已经联网）、蓝色闪烁（智能配网，长按配网按钮2s进入）、黄色闪烁（遥控学习，长按配置按钮5s进入）
// 接收消息和发送消息时、mqtt连接成功 蓝色灯闪烁一次
#define IO_LED_R            11
#define IO_LED_G            12
#define IO_LED_B            13

#define IO_SWITCH_BUTTON          8    // 短按开关、连续短按断网/联网、长按3s重启

#define IO_RF_RECEIVER_D          19    // 433M射频遥控接收
#define IO_RF_RECEIVER_C          20
#define IO_RF_RECEIVER_B          21
#define IO_RF_RECEIVER_A          26
#define IO_RF_LEARN_KEY           33    // RF遥控学习

#define OTA_DIAGNOSTIC            35    // ota升级回滚确认引脚

// #define BROKEN_URL               "mqtt://106.12.9.213:1883"
#define BROKEN_URL               "mqtt://192.168.31.209:1883"
// #define BROKEN_IP_HOST           "106.38.203.210"
#define BROKEN_IP_HOST           "192.168.31.209"
#define BROKEN_IP_PORT           1883
// #define BROKEN_SSL_URL           "mqtts://test.mosquitto.org:8884" 
#define BROKEN_SSL_URL           "mqtts://192.168.31.209:8883"
#define BROKEN_ADMIN             "admin"
#define BROKEN_PWD               "public"
#define VERSION                  "1.0"
#define CATEGORY                 4  //1-wifi通断器,2-智能灯,3-智能门锁,4-environment


extern char ssid[33];                // wifi的SSID
extern char pwd[65];                 // wifi的密码
extern char owner_id[64];            // 用户ID

extern char open_broken_url[128];    // 二次开发mqtt服务器地址
extern char open_account[64];        // 二次开发mqtt账号
extern char open_pwd[64];            // 二次开发mqtt密码

extern uint8_t relay_status;         // 继电器状态 0-关闭，1-打开
extern uint8_t light_status;         // 灯的状态 0-关闭 1-打开
extern uint8_t light_mode;           // 颜色模式 0-固定颜色 1-七彩渐变 2-七彩动感 3-单色渐变 4-白光 5-暖光

extern uint8_t is_radar;             // 是否进入雷达感应
extern uint8_t is_alarm;             // 是否打开报警
extern uint8_t is_wifi_offline;      // 是否离线模式
extern uint8_t is_open_certifi;      // 二次开发是否使用证书
extern uint8_t is_host;              // 是否托管
extern uint8_t is_rf_control;        // 是否使用RF遥控
extern uint8_t rf_one_func;          // 第一个遥控按键功能，1-继电器通断，2-开关灯，开关雷达，4-报警开关，5-智能配网
extern uint8_t rf_two_func;          // 第二个遥控按键功能
extern uint8_t rf_three_func;        // 第三个遥控按键功能
extern uint8_t rf_four_func;         // 第四个遥控按键功能

extern uint32_t brightness;           // 亮度0-100
extern uint32_t light_interval;      // 颜色闪烁间隔
extern uint32_t radar_interval;      // 雷达开灯持续时间,单位秒
extern uint32_t fade_time;           // 灯渐变时间

// 全局变量-不需要存储
extern char device_num[13];          // 设备编号，mac地址
extern int8_t rssi;                  // wifi信号强度(信号极好4格[-55—— 0]，信号好3格[-70—— -55]，信号一般2格[-85—— -70]，信号差1格[-100—— -85])
extern uint8_t wifi_status;          // wifi连接状态：0-未联网，1-已联网, 2-router conecting, 3-router connected, 4-cloud failed
extern uint8_t trigger_source;       // 触发源：0-无、1-按键、2.手机、3-浏览器、4-射频遥控、5-雷达、6-报警、7-定时
extern uint8_t is_reset;             // 是否重启
extern uint8_t is_ap;                // 是否打开AP
extern uint8_t is_rf_learn;          // 遥控配对
extern uint8_t is_rf_clear;          // 遥控清码
extern uint8_t is_smart_config;      // 智能配网
extern float device_temp;            // 设备温度
extern float air_temperature;        // 空气温度
extern float air_humidity;           // 空气湿度

enum{
    WIFI_STATUS_ROUTER_CONNECTING = 0,
    WIFI_STATUS_ROUTER_CONNECTED = 1,
    WIFI_STATUS_ROUTER_DISCONNECTING = 2,
    WIFI_STATUS_CLOUD_CONENCTED = 3,
    WIFI_STATUS_CLOUD_DISCONNECTED = 4
};

// 设备重启
void device_restart(void);
// 获取设备编号
void get_device_num(void);
//设备温度初始化
void device_temp_init(void);
// 读取证书
void read_cert(void);
// 打开继电器
void open_relay(void);
//关闭继电器
void close_relay(void);
// 获取rssi
void get_ap_info(void);

/************************************ drv_dht11.c ************************************/

// DHT11 数据类型定义
typedef struct
{
	uint8_t	humi_int;		//湿度的整数部分
	uint8_t	humi_deci;		//湿度的小数部分
	uint8_t	temp_int;		//温度的整数部分
	uint8_t	temp_deci;		//温度的小数部分
	uint8_t	check_sum;		//校验和
}DHT11_Data_TypeDef;
 
void dht11_gpio_init (void);
uint8_t dht11_Read_Data(float * temp, float * humi);

/************************************ drv_i2c.c ************************************/

void alcohol_init(void);

float alcohol_single_read(void);

/************************************ drv_i2c.c ************************************/

typedef void (*i2c_set_mode)(int pin, int mode);    // 0:  output,  1:  input
typedef void (*i2c_pin_write)(int pin, int enable);
typedef int (*i2c_pin_read)(int pin);
typedef void (*i2c_delay)(int);  // delay 2us

typedef struct{
    uint8_t addr;
    int pin_sda;
    int pin_scl;
    i2c_set_mode mode_set_cb;
    i2c_pin_write pin_write_cb;
    i2c_pin_read pin_read_cb;
    i2c_delay delay_cb;
}user_i2c_t;

void user_i2c_init(user_i2c_t  *i2c);

void user_i2c_start(user_i2c_t  *i2c);

void user_i2c_stop(user_i2c_t  *i2c);

uint8_t user_i2c_wait_ack(user_i2c_t  *i2c);

void user_i2c_ack(user_i2c_t  *i2c);

void user_i2c_nack(user_i2c_t  *i2c);

void user_i2c_send_byte(user_i2c_t  *i2c, uint8_t txd);

uint8_t i2c_read_byte(user_i2c_t *i2c, unsigned char ack);

/******************************** user_drv_mpu6050.c ***********************************/
typedef struct{
    short aacx;
    short aacy;
    short aacz;	
}user_mpu6050_t;

extern user_mpu6050_t user_mpu6050;

void user_bsp_delay_us(int us);

void user_mode_set_cb(int pin, int mode);

void user_bsp_pin_write(int pin, int enable);

int user_bsp_pin_read(int pin);

//#define MPU_ACCEL_OFFS_REG		0X06	//accel_offs寄存器,可读取版本号,寄存器手册未提到
//#define MPU_PROD_ID_REG			0X0C	//prod id寄存器,在寄存器手册未提到
#define MPU_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU_SELF_TESTZ_REG		0X0F	//自检寄存器Z
#define MPU_SELF_TESTA_REG		0X10	//自检寄存器A
#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU_CFG_REG				0X1A	//配置寄存器
#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG	0X67	//IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG	0X69	//运动检测控制寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2 
#define MPU_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU_DEVICE_ID_REG		0X75	//器件ID寄存器
 
//如果AD0脚(9脚)接地,IIC地址为0X68(不包含最低位).
//如果接V3.3,则IIC地址为0X69(不包含最低位).
#define MPU_ADDR				0X68


////因为模块AD0默认接GND,所以转为读写地址后,为0XD1和0XD0(如果接VCC,则为0XD3和0XD2)  
//#define MPU_READ    0XD1
//#define MPU_WRITE   0XD0

int mpu6050_init(void);
uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);//IIC连续写
uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf); //IIC连续读 
uint8_t MPU_Write_Byte(uint8_t reg,uint8_t data);				//IIC写一个字节
uint8_t MPU_Read_Byte(uint8_t reg);						//IIC读一个字节

uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr);
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr);
uint8_t MPU_Set_LPF(uint16_t lpf);
uint8_t MPU_Set_Rate(uint16_t rate);
uint8_t MPU_Set_Fifo(uint8_t sens);


short MPU_Get_Temperature(void);
uint8_t MPU_Get_Gyroscope(short *gx,short *gy,short *gz);
uint8_t MPU_Get_Accelerometer(short *ax,short *ay,short *az);

#endif

