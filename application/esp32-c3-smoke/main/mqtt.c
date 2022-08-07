/******************************************************************************
 * author: kerwincui
 * create: 2021-06-08
 * email：164770707@qq.com
 * source:https://github.com/kerwincui/wumei-smart
 ******************************************************************************/
#include "mqtt.h"
#include "cJSON.h"
#include "cJSON_Utils.h"
#include "nvs_storage.h"
#include "driver/temp_sensor.h"
#include "wifi.h"
#include "drv_include.h"

static const char *TAG = "MQTT";
static esp_mqtt_client_handle_t mqtt_client=NULL;
static char *update_status_topic;
static char *get_status_topic;
static char *update_setting_topic;
static char *get_setting_topic;

static TaskHandle_t report_task_handle = NULL;

SemaphoreHandle_t  sem_mqtt = NULL;             //定义一个二值信号量或者计数信号量的句柄

static int sensor_human_val = 0;
float dht11_data_humi = 0, dht11_data_temp = 0;
static int sensor_light_val = 0;
static int sensor_smoke_val = 0;

/*  @brief 发布设备状态
 *
 *  "remark":{"acc_x":15, "acc_y":10, "acc_z":20, "isHuman":1}
 */ 
void publishStatus(void)
{
    get_ap_info();                            //获取wifi信号

    cJSON *status = cJSON_CreateObject();        
    cJSON_AddStringToObject(status,"deviceNum",device_num);
    cJSON_AddNumberToObject(status,"relayStatus",relay_status);
    cJSON_AddNumberToObject(status,"lightStatus",light_status);
    cJSON_AddNumberToObject(status,"isOnline",1);
    cJSON_AddNumberToObject(status,"rssi",rssi);
    cJSON_AddNumberToObject(status,"deviceTemperature",device_temp);
    cJSON_AddNumberToObject(status,"airTemperature",dht11_data_temp);
    cJSON_AddNumberToObject(status,"airHumidity",dht11_data_humi);
    cJSON_AddNumberToObject(status,"triggerSource",trigger_source); 
    cJSON_AddNumberToObject(status,"brightness",brightness);
    cJSON_AddNumberToObject(status,"lightInterval",light_interval);
    cJSON_AddNumberToObject(status,"lightMode",light_mode);
    cJSON_AddNumberToObject(status,"fadeTime",fade_time);
    cJSON *json_env = cJSON_CreateObject();
    cJSON_AddNumberToObject(json_env, "lightValue", sensor_light_val);
    cJSON_AddNumberToObject(json_env, "smokeValue", sensor_smoke_val);
    cJSON_AddNumberToObject(json_env, "isHuman", sensor_human_val);
    cJSON_AddItemToObject(status, "remark", json_env);
    char *status_msg = cJSON_Print(status);
    ESP_LOGI(TAG,"status msg: %s",status_msg);
    int msg_id=esp_mqtt_client_publish(mqtt_client,"status",status_msg,0,1,0);
    ESP_LOGI(TAG, "sent publish device status, msg_id=%d",msg_id);

    //释放内存
    cJSON_free(status_msg);
    cJSON_Delete(status);
}

//发布设备配置
void publishSetting(void)
{
    cJSON *setting = cJSON_CreateObject();
    cJSON_AddStringToObject(setting,"deviceNum",device_num);
    cJSON_AddNumberToObject(setting,"isAlarm",is_alarm);
    cJSON_AddNumberToObject(setting,"isRadar",is_radar);
    cJSON_AddNumberToObject(setting,"isHost",is_host);
    cJSON_AddStringToObject(setting,"ownerId",owner_id);
    cJSON_AddNumberToObject(setting,"isReset",is_reset);
    cJSON_AddNumberToObject(setting,"isAp",is_ap);
    cJSON_AddNumberToObject(setting,"isRfLearn",is_rf_learn);
    cJSON_AddNumberToObject(setting,"isRfClear",is_rf_clear); 
    cJSON_AddNumberToObject(setting,"isSmartConfig",is_smart_config); 
    cJSON_AddNumberToObject(setting,"radarInterval",radar_interval); 
    cJSON_AddNumberToObject(setting,"isWifiOffline",is_wifi_offline);
    char *setting_msg = cJSON_Print(setting);            
    ESP_LOGI(TAG,"setting msg: %s",setting_msg);
    int msg_id=esp_mqtt_client_publish(mqtt_client,"setting",setting_msg,0,1,0);
    ESP_LOGI(TAG, "sent publish device setting, msg_id=%d",msg_id);
    //释放内存
    cJSON_free(setting_msg);
    cJSON_Delete(setting);
}

// 更新设备状态
static void updateStatus(esp_mqtt_event_handle_t event,char *topic)
{
    //将字符串格式的json数据转化为JSON对象格式
    cJSON *root = cJSON_Parse(event->data);
    if(root == NULL) { printf("parse error\n"); }

    cJSON *value_relay = cJSON_GetObjectItem(root, "relayStatus");
    char *relay = cJSON_Print(value_relay);     
    relay_status=atoi(relay);
    //打开关闭继电器
    if(relay_status==1){
        user_relay_open();
    }else{
        user_relay_close();
    }
    free(relay);

    cJSON *value_trigger = cJSON_GetObjectItem(root, "triggerSource");
    char *trigger = cJSON_Print(value_trigger); 
    trigger_source=atoi(trigger);
    free(trigger);

    cJSON *value_brightness = cJSON_GetObjectItem(root, "brightness");
    char *bright = cJSON_Print(value_brightness);     
    brightness=atoi(bright);    
    free(bright);

    cJSON *value_interval = cJSON_GetObjectItem(root, "lightInterval");
    char *interval = cJSON_Print(value_interval);     
    light_interval=atoi(interval); 
    free(interval);

    cJSON *value_mode = cJSON_GetObjectItem(root, "lightMode");
    char *mode = cJSON_Print(value_mode);     
    light_mode=atoi(mode);
    free(mode);

    cJSON *value_fade = cJSON_GetObjectItem(root, "fadeTime");
    char *fade = cJSON_Print(value_fade);     
    fade_time=atoi(fade);
    free(fade);

    cJSON *value_light = cJSON_GetObjectItem(root, "lightStatus");
    char *light = cJSON_Print(value_light);
    light_status=atoi(light); 
    //开关LED
    if(relay_status==1){
        user_led_open();
    }else{
        user_led_close();
    }
    free(light);
    
    //更新灯状态并存储
    //   led_status();
    // write_config_data();
    cJSON_Delete(root);
}

//更新设备配置
static void updateSetting(esp_mqtt_event_handle_t event,char *topic){
    //将字符串格式的json数据转化为JSON对象格式
    cJSON *root = cJSON_Parse(event->data);
    if(root == NULL) { printf("parse error\n"); }

    cJSON *value_alarm = cJSON_GetObjectItem(root, "isAlarm");
    char *alarm = cJSON_Print(value_alarm);     
    is_alarm=atoi(alarm);
    free(alarm);

    cJSON *value_radar = cJSON_GetObjectItem(root, "isRadar");
    char *radar = cJSON_Print(value_radar);     
    is_radar=atoi(radar);
    free(radar);

    cJSON *value_host = cJSON_GetObjectItem(root, "isHost");
    char *host = cJSON_Print(value_host);
    is_host=atoi(host);
    free(host);

    cJSON *value_ap = cJSON_GetObjectItem(root, "isAp");
    char *ap = cJSON_Print(value_ap);     
    if(strcmp(ap, "1") == 0){
        // 打开AP
        if(is_ap!=1){
            // 打开ap
            ESP_LOGI(TAG, "open access point \n");
            is_ap=1;
            ap_start(); 
        }
    }
    free(ap);   

    //写入配置
    write_config_data();

    cJSON *value_reset = cJSON_GetObjectItem(root, "isReset");
    char *reset = cJSON_Print(value_reset);     
    if(strcmp(reset, "1") == 0){
        // 设备重启
        device_restart();
    }
    free(reset);    

    cJSON_Delete(root);
}

static void mqtt_report_task(void *args)
{
    while (1)
    {
        // test drv
        temp_sensor_read_celsius(&device_temp);   //获取设备温度
    
        dht11_Read_Data(&dht11_data_temp, &dht11_data_humi);

        ESP_LOGI(TAG, "----------------------------get temp : %04f, humi : %04f\r\n", dht11_data_temp, dht11_data_humi);

        sensor_smoke_val = smoke_single_read();
        ESP_LOGI(TAG, " sensor_smoke_val :%d\r\n", sensor_smoke_val);

        sensor_human_val = sensor_human_get();
        ESP_LOGI(TAG, "sensor_human_val : %d\r\n", sensor_human_val);

        sensor_light_val = sensor_light_single_read();
        ESP_LOGI(TAG, "sensor light's value  : %d\r\n", sensor_light_val);

        if (wifi_status == WIFI_STATUS_CLOUD_CONENCTED)
        {
            publishStatus();
        }
        
        vTaskDelay(5000 / portTICK_RATE_MS);
    }
}

static void mqtt_subscribe_event(esp_mqtt_event_handle_t event)
{
    char topic[32];
    ESP_LOGI(TAG,"event topic:%.*s\r", event->topic_len, event->topic);
    ESP_LOGI(TAG,"event data:%.*s\r\n",event->data_len, event->data);
    sprintf(topic,"%.*s", event->topic_len, event->topic);
    if (strcmp(topic, update_status_topic) == 0)
    {
        //更新设备状态
        updateStatus(event,topic);
        // 发布设备状态
        publishStatus();
        
    } else if (strcmp(topic, get_status_topic) == 0)
    {
        publishStatus();
        
    }else if(strcmp(topic,update_setting_topic) == 0){
        // 更新设备配置
        updateSetting(event,topic);
        // 发布设备配置
        publishSetting();
    }
    else if(strcmp(topic, get_setting_topic) == 0)
    {
        publishSetting();
    }
}


static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    int msg_id;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
                    //-----------------------------------订阅消息----------------------------------------
            // 订阅更新设备状态            
            msg_id = esp_mqtt_client_subscribe(mqtt_client, update_status_topic, 1);
            ESP_LOGI(TAG, "sent subscribe set status successful, msg_id=%d,topic=%s", msg_id,update_status_topic);
            // 订阅获取设备状态
            msg_id = esp_mqtt_client_subscribe(mqtt_client, get_status_topic, 1);
            ESP_LOGI(TAG, "sent subscribe get status successful, msg_id=%d,topic=%s", msg_id,get_status_topic);
            // 订阅更新设备配置            
            msg_id = esp_mqtt_client_subscribe(mqtt_client, update_setting_topic, 1);
            ESP_LOGI(TAG, "sent subscribe set setting successful, msg_id=%d,topic=%s", msg_id,update_setting_topic);
            // 订阅获取设备配置            
            msg_id = esp_mqtt_client_subscribe(mqtt_client, get_setting_topic, 1);
            ESP_LOGI(TAG, "sent subscribe get setting successful, msg_id=%d,topic=%s", msg_id,get_setting_topic);  

            xSemaphoreGive( sem_mqtt );
            wifi_status = WIFI_STATUS_CLOUD_CONENCTED;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            wifi_status = WIFI_STATUS_CLOUD_DISCONNECTED;
            break;
        case MQTT_EVENT_SUBSCRIBED:            
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            //订阅事件处理
            mqtt_subscribe_event(event); 
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
    return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}

// 配置订阅主题
static void config_topic(){
    static char *update_status_path="status/set/";
    static char *get_status_path="status/get/";
    static char *update_setting_path="setting/set/";
    static char *get_setting_path="setting/get/";
    // 更新状态
    update_status_topic=(char *)malloc(strlen(update_status_path)+strlen(device_num)+1);
    if(update_status_topic==NULL){
        ESP_LOGD(TAG, "failed to apply for memory");
    }
    strcpy(update_status_topic,update_status_path);
    strcat(update_status_topic,device_num);
    ESP_LOGI(TAG,"update_status_topic:%s",update_status_topic);
    // 获取状态
    get_status_topic=(char *)malloc(strlen(get_status_path)+strlen(device_num)+1);
    if(get_status_topic==NULL){
        ESP_LOGD(TAG, "failed to apply for memory");
    }
    strcpy(get_status_topic,get_status_path);
    strcat(get_status_topic,device_num);
    ESP_LOGI(TAG,"get_status_topic:%s",get_status_topic);
    // 更新配置
    update_setting_topic=(char *)malloc(strlen(update_setting_path)+strlen(device_num)+1);
    if(update_setting_topic==NULL){
        ESP_LOGD(TAG, "failed to apply for memory");
    }
    strcpy(update_setting_topic,update_setting_path);
    strcat(update_setting_topic,device_num);
    ESP_LOGI(TAG,"update_setting_topic:%s",update_setting_topic);
    // 获取配置
    get_setting_topic=(char *)malloc(strlen(get_setting_path)+strlen(device_num)+1);
    if(get_setting_topic==NULL){
        ESP_LOGD(TAG, "failed to apply for memory");
    }
    strcpy(get_setting_topic,get_setting_path);
    strcat(get_setting_topic,device_num);  
    ESP_LOGI(TAG,"get_setting_topic:%s",get_setting_topic);
}

void mqtt_start(void)
{
    int msg_id;
    BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为 pdPASS */ 

    sem_mqtt = xSemaphoreCreateBinary();
    if (NULL != sem_mqtt) 
    {
        printf("create sem_mqtt success\r\n"); 
    }

    // esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("MQTT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    // 配置订阅的主题
    config_topic();

    // 遗嘱消息内容
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root,"deviceNum",device_num);
    cJSON_AddNumberToObject(root,"isOnline",0);
    char *will_msg = cJSON_Print(root);
    //释放内存
	cJSON_Delete(root);

    ESP_LOGI(TAG,"last will: %s", will_msg);
    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = BROKEN_URL,
        .username = BROKEN_ADMIN,
        .password = BROKEN_PWD,
        //设置遗嘱
        .lwt_topic="offline",
        .lwt_msg=will_msg,
        .lwt_msg_len=strlen(will_msg),
        .lwt_qos=1,
        .lwt_retain=0,
        //断开前等待的时间,3秒
        .keepalive=3,
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);

    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    
    esp_mqtt_client_start(mqtt_client);

    //获取二值信号量 xSemaphore,没获取到则一直等待 
    xReturn = xSemaphoreTake(sem_mqtt,/* 二值信号量句柄 */ 
                            portMAX_DELAY); /* 等待时间 */ 
    if (pdTRUE == xReturn) 
    {
        printf("Device connect to cloud success\r\n"); 
    } 

        //-----------------------------------发布消息--------------------------------------
    //发布设备信息
    cJSON *device_info = cJSON_CreateObject();
    cJSON_AddStringToObject(device_info,"deviceNum",device_num);
    cJSON_AddNumberToObject(device_info,"categoryId",CATEGORY);
    cJSON_AddStringToObject(device_info,"firmwareVersion",VERSION);
    cJSON_AddStringToObject(device_info,"ownerId",owner_id);
    char *device_msg = cJSON_Print(device_info);
    //释放内存
    cJSON_Delete(device_info);
    ESP_LOGI(TAG,"device msg: %s",device_msg);
    msg_id=esp_mqtt_client_publish(mqtt_client,"device_info",device_msg,0,1,0);
    ESP_LOGI(TAG, "sent publish device info, msg_id=%d",msg_id);
    cJSON_free(device_msg);

    // start report thread
    if (xTaskCreate(mqtt_report_task, "mqtt_report_task", 4096, NULL, 1, &report_task_handle) != pdTRUE) 
    {
        ESP_LOGE(TAG, "Error create mqtt task");
    }
}

void mqtt_stop(void){
    if(mqtt_client!=NULL){
        esp_mqtt_client_stop(mqtt_client);
    }
}
void mqtt_continue(void){
    if(mqtt_client!=NULL){
        esp_mqtt_client_start(mqtt_client);
    }
}

