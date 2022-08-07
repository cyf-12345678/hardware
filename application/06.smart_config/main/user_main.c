/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "drv_include.h"

/* Can use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO GPIO_NUM_18

#define TAG "user_main"

void app_main(void)
{

    nvs_handle_t my_handle;
    char data_out[64] = { 0x00 };
    size_t data_len = 20;

    ESP_LOGE(TAG, "Error hello world");
    ESP_LOGW(TAG, "Warn hello world");
    ESP_LOGI(TAG, "Info hello world");
    ESP_LOGD(TAG, "debug hello world");
    ESP_LOGV(TAG, "verbose hello world");

    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // // open namespace
    // err = nvs_open("wifi_config", NVS_READWRITE, &my_handle);
    // if (ESP_OK != err)
    // {
    //     printf("open nvs namespace(wifi_config) error\r\n");
    // }else
    // {
    //     err = nvs_get_str(my_handle, "ssid", data_out, &data_len );
    //     switch (err) {
    //         case ESP_OK:
    //             printf("Done\n");
    //             printf("########### get ssid success(%d) %s\r\n", data_len, data_out);
    //             break;
    //         case ESP_ERR_NVS_NOT_FOUND:
    //             printf("The value is not initialized yet!\n");
    //             break;
    //         default :
    //             printf("Error (%s) reading!\n", esp_err_to_name(err));
    //     }

    // }

    // // write/read nvs_flash
    // err = nvs_set_str(my_handle, "ssid", "brown123456");
    // if (err == ESP_OK)
    // {
    //     printf("write nvs_value success\r\n");
    // }
    
    // err = nvs_commit(my_handle);
    // if (err != ESP_OK)
    // {
    //     printf("nvs commit error\r\n");
    // }


    // // close namespace
    // nvs_close(my_handle);

    /* Configure the IOMUX register for pad BLINK_GPIO (some pads are
       muxed to GPIO on reset already, but some default to other
       functions and need to be switched to GPIO. Consult the
       Technical Reference for a list of pads and their default
       functions.)
    */
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    // init iot_button
    button_start();
    
    // 连网
    wifi_start();
    
    // while(1) {
    //     /* Blink off (output low) */
    //     printf("Turning off the LED\n");
    //     gpio_set_level(BLINK_GPIO, 0);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    //     /* Blink on (output high) */
    //     printf("Turning on the LED\n");
    //     gpio_set_level(BLINK_GPIO, 1);
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
}
