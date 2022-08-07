
set(COMPONENT_SRCS "main.c"  
"drv_button.c" 
"drv_common.c" 
"drv_dht11.c"
"drv_adc.c"
"lwip_sntp.c" 
"mqtt.c" 
"nvs_storage.c" 
"smart_config.c" 
"wifi.c" 
)
set(COMPONENT_ADD_INCLUDEDIRS ". include")
set(COMPONENT_EMBED_TXTFILES ${project_dir}/server_certs/ca_cert.pem)
set(COMPONENT_EMBED_FILES "www/home.html.gz" "www/prompt.html.gz" "favicon.ico" "www/upload_script.html" "spiffs/test.pem" "main/spiffs/test.pem")
register_component()