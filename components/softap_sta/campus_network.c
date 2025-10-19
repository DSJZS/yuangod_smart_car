#include "campus_network.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_http_client.h"
#include <string.h>

#define TAG     "campus_network"

/* 校园网登陆配置结构体 */
typedef struct {
    char* user_id;                //  校园网用户id, 一般是你的学号或者证件号, 看学校安排
    char* user_password;          //  校园网用户密码, 一般不为空故这里没有做空密码处理
} login_info_t;

/* http回调函数 */
static esp_err_t can_http_event_handler(esp_http_client_event_t *evt)
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
uint8_t campus_network_http_post( login_info_t* user_info)
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
    snprintf( payload_data, sizeof(payload_data), "userId=%s&password=%s", user_info->user_id, user_info->user_password);
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

uint8_t campus_network_login( char* user_id, char* user_password)
{
    login_info_t login_info = {
        .user_id = user_id,
        .user_password = user_password,
    };
    return campus_network_http_post( &login_info);
}