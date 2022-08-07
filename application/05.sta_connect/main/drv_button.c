// Copyright 2020 Espressif Systems (Shanghai) Co. Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <string.h>
#include "esp_log.h"
#include "iot_button.h"

#define TAG "drv_button"
#define IO_SWITCH_BUTTON    6

static void button_single_click_cb(void *arg)
{
    ESP_LOGI(TAG, "button single clicked!\r\n");
}

static void button_long_press_start_cb(void *arg)
{
    ESP_LOGI(TAG, "button long pressed!\r\n");
}

void button_start(void)
{
    //初始化按键
    button_config_t cfg = {
        .type = BUTTON_TYPE_GPIO,
        .gpio_button_config = {
            .gpio_num = IO_SWITCH_BUTTON,
            .active_level = 0,
        },
    };
    button_handle_t gpio_btn = iot_button_create(&cfg);
    if(NULL == gpio_btn)
    { 
        ESP_LOGE(TAG, "Button create failed"); 
    }
    iot_button_register_cb(gpio_btn, BUTTON_SINGLE_CLICK, button_single_click_cb);            //短按
    iot_button_register_cb(gpio_btn, BUTTON_LONG_PRESS_START, button_long_press_start_cb);    //长按
}

