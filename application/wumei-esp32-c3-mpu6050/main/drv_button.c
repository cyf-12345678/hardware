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

#include "drv_include.h"
#include "smart_config.h"
#include "wifi.h"
#include "mqtt.h"
#include "lwip_sntp.h"
#include "nvs_storage.h"

static const char* TAG = "button";
bool is_open=false;

//单击开关按钮(回调函数)
static void button_single_click_cb(void* arg)
{
    ESP_EARLY_LOGI(TAG, "tab switch button\n");
    if(relay_status==1)
    {
        close_relay();
        relay_status=0;
    }else{ 
        open_relay();
        relay_status=1;
    }
    write_relay_status();
    
    // mqtt发布状态
    if(is_wifi_offline==0 && wifi_status==1 ){
        publishStatus();
    }
}

//长按开关按钮
static void button_long_press_start_cb(void* arg)
{
    ESP_LOGI(TAG, "begin smargconfig\n"); 
    //蓝灯闪烁
    // led_rgb_blink(0,0,255,2,0,0,0,0);
    //智能配网        
    smart_config_start();

}

// 连续短按开关按钮
static void button_press_repeat_cb(void *arg)
{    
    ESP_LOGI(TAG, "BUTTON_PRESS_REPEAT[%d]", iot_button_get_repeat((button_handle_t)arg));
    uint8_t num=iot_button_get_repeat((button_handle_t)arg);
    ESP_LOGI(TAG, "press repeat num: %d\n",num);
    
    switch (num)
    {
    case 7:
        ESP_LOGI(TAG, "native ota...\n");
        // led_rgb_blink(255,0,0,2,0,0,0,0);
        // native_ota_start();
        break;
    case 6:
        //打开/关闭雷达        
        // led_rgb_blink(0,0,255,2,0,6,0,0);
        is_radar=is_radar==1 ? 0:1;
        write_is_radar();
        break;
    case 5:
        //遥控清码
        ESP_LOGI(TAG, "rf clear code ...\n");
        // led_rgb_blink(0,0,255,2,0,5,0,0);
        is_rf_clear=1;
        break;
    case 4:    
        //遥控学习    
        ESP_LOGI(TAG, "rf learning... \n");
        // led_rgb_blink(0,0,255,2,0,4,0,0);
        is_rf_learn=1;
        break;
    case 3:                
        if(is_ap!=1){
            // 打开ap
            ESP_LOGI(TAG, "open access point \n");
            is_ap=1;
            ap_start(); 
        }else{
            //重启
            device_restart();
        }        
        break;   
    case 2:                
        ESP_EARLY_LOGI(TAG, "switch light \n");
        light_status=light_status==0?1:0;
        //   led_status();
        //   write_light_status();
        break; 
    default:
        break;
    }

    // mqtt发布状态
    if(is_wifi_offline==0 && wifi_status==1 ){
        publishSetting();
        publishStatus();
    }
}

void button_start()
{
    printf("before btn init, heap: %d\n", esp_get_free_heap_size());    

    //初始化继电器引脚
    gpio_pad_select_gpio(IO_RELAY);
    gpio_pad_select_gpio(IO_RELAY_MQ3);
    gpio_set_direction(IO_RELAY,GPIO_MODE_OUTPUT);
    gpio_set_direction(IO_RELAY_MQ3,GPIO_MODE_OUTPUT);
    gpio_set_level(IO_RELAY, 1);
     gpio_set_level(IO_RELAY_MQ3, 1);

    //初始化按键
    button_config_t cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = IO_SWITCH_BUTTON,
            .active_level = 0,
        },
    };
    button_handle_t gpio_btn = iot_button_create(&cfg);
    if(NULL == gpio_btn) { ESP_LOGE(TAG, "Button create failed"); }
    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, button_single_click_cb);            //短按
    iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_START, button_long_press_start_cb);    //长按
    iot_button_register_cb(gpio_btn, BUTTON_PRESS_REPEAT, button_press_repeat_cb);            //连续短按
}

