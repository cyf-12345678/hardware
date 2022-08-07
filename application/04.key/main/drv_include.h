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

/********************* drv_button.c ********************/

void button_start(void);

#endif