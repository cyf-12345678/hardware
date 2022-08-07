/******************************************************************************
 * author: kerwincui
 * create: 2021-06-08
 * email：164770707@qq.com
 * source:https://github.com/kerwincui/wumei-smart
 ******************************************************************************/
#include <stdio.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "nvs.h"
#include "esp_pm.h"
#include "mqtt.h"
#include "smart_config.h"
#include "lwip_sntp.h"
#include "wifi.h"
#include "nvs_storage.h"
#include "freertos/FreeRTOS.h"

#include "drv_include.h"

// #include "mqtt_ssl.h"
// #include "native_ota.h"
// #include "flash_encrypt.h"
// #include "statistic_free_rtos.h"
// #include "statistic_perfmon.h"

static const char *TAG = "wumei-open";

void app_main()
{
    int flag = 0;

    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "[wumei-open] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    // 默认时间循环
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // 读取配置信息
    read_config_data();
    // 获取设备编号
    get_device_num();

    // 启动按钮
    button_start();

    // 初始化设备温度
    device_temp_init();

    // 初始化温湿度传感器
    dht11_gpio_init();

    // 初始化LED
    user_led_init();
    
    printf("init led and relay\r\n");

    // 初始化继电器
    user_relay_init();
    
    // 初始化OELD
    user_oled_init();

    user_oled_fill(0xFF);//全屏点亮
    vTaskDelay(200 / portTICK_RATE_MS);
    user_oled_fill(0x00);//全屏灭

    // 显示OLED屏幕信息
    user_oled_show_string(24, 0, (uint8_t *)" System", 2);
    user_oled_show_string(5, 3,  (uint8_t *)"Light Now:", 1);	
    user_oled_show_string(5, 5,  (uint8_t *)"Light MIN:", 1);	
    user_oled_show_string(5, 7,  (uint8_t *)"Fire:", 1);

    // wifi
    if(is_wifi_offline==0){
        //连接wifi
        wifi_start();
        // 启动mqtt
        mqtt_start();
        //获取SNTP时间
        // sntp_start();
    }
}

