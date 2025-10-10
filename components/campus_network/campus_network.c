#include "campus_network.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_http_client.h"
#include <string.h>

#define TAG     "campus_network"

static EventGroupHandle_t s_wifi_ev = NULL;
#define WIFI_NEED_TO_CONNECTED_EV   (BIT0)
#define WIFI_CONNECTED_SUCCESS_EV   (BIT1)

#define CAMPUS_NETWORK_NEED_TO_AUTH (BIT2)
#define CAMPUS_NETWORK_IS_CERTIFIED (BIT3)

typedef struct {
    char user_id[64];                //  校园网用户id, 一般是你的学号或者证件号, 看学校安排
    char user_password[64];          //  校园网用户密码, 一般不为空故这里没有做空密码处理
} log_info_t;

log_info_t user_info = {0};

/* wifi回调函数 */
static void can_wifi_event_handle(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if( event_base == WIFI_EVENT ) {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:  //  启动STA工作模式了
            ESP_LOGI(TAG, "esp32 is connecting to ap......");
            xEventGroupSetBits( s_wifi_ev, WIFI_NEED_TO_CONNECTED_EV); //  开启wifi连接
            break;
        case WIFI_EVENT_STA_CONNECTED:  //  wifi连接上了AP
            ESP_LOGI(TAG, "esp32 connected to ap!");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:
            /* 官方推荐记录重连次数, 延时或者退出, 防止频繁地重复连接, 这里省略, 放到后台任务中处理 */
            ESP_LOGI(TAG, "esp32 connected the ap failed! retry!");
            xEventGroupSetBits( s_wifi_ev, WIFI_NEED_TO_CONNECTED_EV); //  重新开启wifi连接
            xEventGroupWaitBits( s_wifi_ev, WIFI_CONNECTED_SUCCESS_EV | CAMPUS_NETWORK_IS_CERTIFIED, pdTRUE, pdFALSE, 0);
            break;
        default:
            break;
        }
    } else if ( event_base == IP_EVENT ) {
        switch (event_id)
        {
        case IP_EVENT_STA_GOT_IP:   //  ESP32成功从AP获取分配的IP, 即真正的连接上了AP
            ESP_LOGI(TAG, "esp32 get ip address!");
            xEventGroupSetBits( s_wifi_ev, WIFI_CONNECTED_SUCCESS_EV);
            ESP_LOGI(TAG, "esp32 is being authenticated on the campus network.......");
            xEventGroupSetBits( s_wifi_ev, CAMPUS_NETWORK_NEED_TO_AUTH); //  开启校园网认证
            break;
        default:
            break;
        }
    }
}

/* http回调函数 */
esp_err_t can_http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id){
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        /*
         *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
         *  However, event handler can also be used in case chunked encoding is used.
         */
        if (!esp_http_client_is_chunked_response(evt->client))
        {
            ESP_LOGI(TAG, "Received data (length=%d): %.*s", evt->data_len, evt->data_len, (char*)evt->data);
        }
        break;
    default:
            break;
    }
    
    return ESP_OK;
}

/* http post请求发送函数 */
uint8_t campus_network_http_post(void)
{
    /* HTTP客户端配置 */
    esp_http_client_config_t http_client_config = {
        .url = "http://210.30.0.113/eportal/InterFace.do?method=login",
        // .timeout_ms = 10 * 1000,
        // .keep_alive_enable = true,
        .method = HTTP_METHOD_POST,
        .event_handler = can_http_event_handler,
        .buffer_size = 1024,
        .buffer_size_tx = 2048,
    };

    /* 初始化HTTP客户端 */
    esp_http_client_handle_t http_client = esp_http_client_init( &http_client_config);
    if ( http_client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize HTTP client");
        return 0;
    }

    /* 准备POST数据 */
    char payload_data[1024] = {0};
    snprintf( payload_data, sizeof(payload_data), "userId=%s&password=%s", user_info.user_id, user_info.user_password);
    strcat( payload_data, "&service=&queryString=wlanuserip%253Dadc8156fa2456190a69784e5096b138b%2526wlanacname%253D53f642a552df5d0946b9667fcfcce7f6%2526ssid%253D%2526nasip%253Ddf9b315fd398ab697449e217764cff8e%2526snmpagentip%253D%2526mac%253D2325bf99f7f197a10880a0cf22c4996f%2526t%253Dwireless-v2%2526url%253D7a56d870afa3640291a5181c844d8299f1e28c9615377a1af01b0a4cfff13d484314a48ce9fcfb3b%2526apmac%253D%2526nasid%253D53f642a552df5d0946b9667fcfcce7f6%2526vid%253D3dce999186299d76%2526port%253De3a19f31d7935139%2526nasportid%253D4ff4d69af5efdbff0a7cf3c11416aea718181cf4a131378204dbc989a3c15456&operatorPwd=&operatorUserId=&validcode=&passwordEncrypt=false");
    esp_http_client_set_header(http_client, "Accept", "*/*");
    esp_http_client_set_header(http_client, "Accept-Encoding", "gzip, deflate");
    esp_http_client_set_header(http_client, "Accept-Language", "zh-CN,zh;q=0.9,en;q=0.8,en-GB;q=0.7,en-US;q=0.6");
    esp_http_client_set_header(http_client, "Connection", "keep-alive");
    esp_http_client_set_header(http_client, "Content-Length", "670");
    esp_http_client_set_header(http_client, "Content-Type", "application/x-www-form-urlencoded; charset=UTF-8");
    esp_http_client_set_header(http_client, "Cookie", "EPORTAL_COOKIE_SERVER=; EPORTAL_COOKIE_DOMAIN=; EPORTAL_COOKIE_OPERATORPWD=; EPORTAL_AUTO_LAND=; EPORTAL_COOKIE_USERNAME=; EPORTAL_COOKIE_PASSWORD=; EPORTAL_COOKIE_SERVER_NAME=; EPORTAL_COOKIE_SAVEPASSWORD=false; EPORTAL_COOKIE_NEWV=; EPORTAL_USER_GROUP=null; JSESSIONID=27FDC2C23480CD344D15610787ABAB76");
    esp_http_client_set_header(http_client, "Host", "210.30.0.113");
    esp_http_client_set_header(http_client, "Origin", "http://210.30.0.113");
    esp_http_client_set_header(http_client, "Referer", "http://210.30.0.113/eportal/index.jsp?wlanuserip=adc8156fa2456190a69784e5096b138b&wlanacname=53f642a552df5d0946b9667fcfcce7f6&ssid=&nasip=df9b315fd398ab697449e217764cff8e&snmpagentip=&mac=2325bf99f7f197a10880a0cf22c4996f&t=wireless-v2&url=7a56d870afa3640291a5181c844d8299f1e28c9615377a1af01b0a4cfff13d484314a48ce9fcfb3b&apmac=&nasid=53f642a552df5d0946b9667fcfcce7f6&vid=3dce999186299d76&port=e3a19f31d7935139&nasportid=4ff4d69af5efdbff0a7cf3c11416aea718181cf4a131378204dbc989a3c15456");
    esp_http_client_set_header(http_client, "User-Agent", "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/140.0.0.0 Safari/537.36 Edg/140.0.0.0");

    esp_http_client_set_post_field(http_client, payload_data, strlen(payload_data));

    /* 执行HTTP POST */
    esp_err_t err = esp_http_client_perform(http_client);
    uint8_t ret = 0;
    if( err == ESP_OK ){
        int status_code = esp_http_client_get_status_code(http_client);
        if (status_code >= 200 && status_code < 300) {
            ret = 1;
        } else {
            ret = 0;
        }
    } else {
        ret = 0;
    }

    /* 清除HTTP资源 */
    esp_http_client_cleanup(http_client);

    if( ret == 0 )
        ESP_LOGI(TAG, "Campus Network Auth Fail!");
    else
        ESP_LOGI(TAG, "Campus Network Auth success!");
    return ret;
}

/* 校园网后台任务 */
static void campus_network_task(void* param)
{
    const TickType_t xDelay_100ms = pdMS_TO_TICKS( 100UL );   //  每次连接或者认证之间的间隔至少为 100ms
    TickType_t xLastWakeTime;
    EventBits_t ev = 0;

    /* 获得当前的Tick Count */
	xLastWakeTime = xTaskGetTickCount();
    while(1)
    {
        ev = xEventGroupWaitBits( s_wifi_ev, WIFI_NEED_TO_CONNECTED_EV | CAMPUS_NETWORK_NEED_TO_AUTH, pdTRUE, pdFALSE, portMAX_DELAY);
        ESP_LOGI(TAG, "the can_task is running again, ev_code %lu", ev);
        if( ev & WIFI_NEED_TO_CONNECTED_EV ) {
            esp_wifi_connect(); //  重新开启wifi连接
        } else if ( ev & CAMPUS_NETWORK_NEED_TO_AUTH ) {
            uint8_t ret = campus_network_http_post();
            if( ret != 0 )
                xEventGroupSetBits( s_wifi_ev, CAMPUS_NETWORK_IS_CERTIFIED); //  认证成功
        }
        vTaskDelayUntil(&xLastWakeTime, xDelay_100ms);
    }

    /* 意外退出释放资源 */
    if( s_wifi_ev != NULL ) {
        vEventGroupDelete(s_wifi_ev);
        s_wifi_ev = NULL;
    }
    vTaskDelete(NULL);
}

void campus_network_config(campus_network_config_t* can_cfg)
{
    /* 初始化 tcp ip协议栈( LWIP 实现 )任务 */
    ESP_ERROR_CHECK( esp_netif_init() );

    /* 初始化 事件系统循环 */
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    /* 初始化 TCP/IP 堆栈的默认网络接口实例绑定 station */
    esp_netif_create_default_wifi_sta();

    /* 初始化 Wi-Fi 驱动程序任务 */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init( &cfg ) );

    /* 注册回调函数 */
    if( can_cfg->can_event_handler == NULL ) {
        esp_event_handler_register( WIFI_EVENT, ESP_EVENT_ANY_ID, can_wifi_event_handle, NULL); //  wifi事件
        esp_event_handler_register( IP_EVENT, IP_EVENT_STA_GOT_IP, can_wifi_event_handle, NULL); //  网络事件
    } else {
        esp_event_handler_register( WIFI_EVENT, ESP_EVENT_ANY_ID, can_cfg->can_event_handler, NULL); //  wifi事件
        esp_event_handler_register( IP_EVENT, IP_EVENT_STA_GOT_IP, can_cfg->can_event_handler, NULL); //  网络事件
    }

    /* 配置 Wi-Fi 驱动程序任务 */
    wifi_config_t wifi_config = {
        .sta.threshold.authmode = WIFI_AUTH_OPEN,   //  配置加密模式为开放认证
        .sta.pmf_cfg.capable = true,    
        .sta.pmf_cfg.required = false,
    };
    memset( &wifi_config.sta.ssid, 0 ,sizeof(wifi_config.sta.ssid));
    memcpy( wifi_config.sta.ssid, can_cfg->can_ssid, strlen(can_cfg->can_ssid));
    memset( &wifi_config.sta.password, 0 ,sizeof(wifi_config.sta.password));
    memcpy( wifi_config.sta.password, "", strlen(""));

    /* 初始化消息组 */
    if( s_wifi_ev == NULL ) {
        s_wifi_ev = xEventGroupCreate();
        if( s_wifi_ev == NULL ) { 
            ESP_LOGE( TAG, "create event group error! terminate connection");
            return;
        }
    }

    /* 初始化用户登录信息 */
    memset( &user_info, 0, sizeof(log_info_t));
    snprintf( user_info.user_id, sizeof(user_info.user_id), can_cfg->can_user_id);
    snprintf( user_info.user_password, sizeof(user_info.user_password), can_cfg->can_user_password);
    
    /* 添加后台任务 */
    xTaskCreatePinnedToCore( campus_network_task, "campus_network_task", 4096, NULL, can_cfg->can_task_priority, NULL, 1);
    
    /* 开启STA模式WIFI */
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config( WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

uint8_t campus_network_is_connected( TickType_t xTicksToWait)
{
    if( s_wifi_ev == NULL )
        return 0;
    // ESP_LOGI( TAG, "waiting for connection and authentication to complete");

    EventBits_t ev = 0;
    ev = xEventGroupWaitBits( s_wifi_ev, WIFI_CONNECTED_SUCCESS_EV | CAMPUS_NETWORK_IS_CERTIFIED, pdFALSE, pdTRUE, xTicksToWait);
    if( ev & ( WIFI_CONNECTED_SUCCESS_EV | CAMPUS_NETWORK_IS_CERTIFIED ) )
        return 1;
    else
        return 0;
}