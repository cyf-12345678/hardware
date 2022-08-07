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
#include "driver/adc.h"


// DHT11 连接引脚定义  
#define 	DHT11_1_DATA_PINS			GPIO_NUM_3

/************************************ drv_button ************************************/

bool is_open;

void button_start(void);
void rf_receiver_start(void);
void radar_start(void);

//获取设备温度
void get_device_temp(void);

/************************************ drv_common.c ************************************/

#define IO_SENSOR_SCL          GPIO_NUM_4   // IIC-SCL
#define IO_SENSOR_SDA          GPIO_NUM_5   // IIC-SDA
#define IO_RELAY               GPIO_NUM_2   // 继电器
#define IO_LED                 GPIO_NUM_18  // LED
#define IO_BUZZ                GPIO_NUM_18  // 蜂鸣器
#define SENSOR_HUMAN_PIN       GPIO_NUM_7

// 接收消息和发送消息时、mqtt连接成功 蓝色灯闪烁一次

#define IO_SWITCH_BUTTON          6    // 短按开关、连续短按断网/联网、长按3s重启

#define OTA_DIAGNOSTIC            35    // ota升级回滚确认引脚

// #define BROKEN_URL               "mqtt://wumei.live:1883"
#define BROKEN_URL               "mqtt://127.0.0.1:1883"
#define BROKEN_SSL_URL           "mqtts://test.mosquitto.org:8884"
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
// 初始化LED
void user_led_init(void);
//关闭LED
void user_led_close(void);
//打开LED
void user_led_open(void);
// 初始化继电器
void user_relay_init(void);
// 打开继电器
void user_relay_open(void);
//关闭继电器
void user_relay_close(void);
//初始化蜂鸣器
void user_buzz_init(void);
// 打开蜂鸣器
void user_buzz_open(void);
//关闭蜂鸣器
void user_buzz_close(void);
// 获取rssi
void get_ap_info(void);
//初始化人体红外传感器
void sensor_human_init(void);
// 读取人体红外传感器
int sensor_human_get(void);
// 初始化板级驱动
void bsp_init(void);

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

/************************************ drv_adc.c ************************************/
void smoke_init (void);

int smoke_single_read(void);

void sensor_light_init (void);

int sensor_light_single_read(void);

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

#endif

