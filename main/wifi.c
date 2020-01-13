#ifdef CONFIG_WIFI_EABLE
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wps.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/api.h"
#include "lwip/tcp.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include "sdkconfig.h"

#include "wifi.h"
#include "uart.h"

#ifdef CONFIG_WIFI_SOFTAP
#include "softap.h"
#endif

//#define USE_WPS 1

/* The examples use WiFi configuration that you can set via 'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/

#ifdef CONFIG_KISS_TCPIP_PORT
#define TNC_PORT CONFIG_KISS_TCPIP_PORT
#else
#define TNC_PORT 3105
#endif

static const char TAG[] = "wifi";

#ifndef CONFIG_WIFI_SOFTAP
/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about one event 
 * - are we connected to the AP with an IP? */
const int WIFI_CONNECTED_BIT = BIT0;


static int s_retry_num = 0;

#define WPS_TEST_MODE WPS_TYPE_PBC

#define PIN2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5], (a)[6], (a)[7] 
#define PINSTR "%c%c%c%c%c%c%c%c"

static esp_wps_config_t config = WPS_CONFIG_INIT_DEFAULT(WPS_TEST_MODE);

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
        s_retry_num = 0;
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
	ESP_LOGI(TAG, "SYSTEM_EVENT_STA_DISCONNECTED");
        ESP_LOGI(TAG,"connect to the AP fail");
	ESP_LOGI(TAG, "ssid: %s", event->event_info.disconnected.ssid);
        {
            if (s_retry_num < 10) {
                ESP_ERROR_CHECK(esp_wifi_connect());
                xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
                s_retry_num++;
                ESP_LOGI(TAG,"retry to connect to the AP");
            } else {
		ESP_ERROR_CHECK(esp_wifi_wps_enable(&config));
		ESP_ERROR_CHECK(esp_wifi_wps_start(0));
		ESP_LOGI(TAG, "retry to wps...");
                s_retry_num = 0;
	    }
            break;
        }
    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:
	ESP_LOGI(TAG, "SYSTEM_EVENT_STA_WPS_ER_SUCCESS");
	ESP_ERROR_CHECK(esp_wifi_wps_disable());
        ESP_ERROR_CHECK(esp_wifi_connect());
	break;
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:
	ESP_LOGI(TAG, "SYSTEM_EVENT_STA_WPS_ER_FAILED/TIMEOUT");
	ESP_ERROR_CHECK(esp_wifi_wps_disable());
	ESP_ERROR_CHECK(esp_wifi_wps_enable(&config));
	ESP_ERROR_CHECK(esp_wifi_wps_start(0));
	break;
    case SYSTEM_EVENT_STA_WPS_ER_PIN:
	ESP_LOGI(TAG, "SYSTEM_EVENT_STA_WPS_ER_PIN");
	ESP_LOGI(TAG, "WPS_PIN = "PINSTR, PIN2STR(event->event_info.sta_er_pin.pin_code));
	break;
    default:
        break;
    }
    return ESP_OK;
}
#endif

static void callback(struct netconn *conn, enum netconn_evt event, u16_t len)
{
    static const char *evt_name[] = {
	    "RCVPLUS",
	    "RCVMINUS",
	    "SENDPLUS",
	    "SENDMINUS",
	    "ERROR",
    };

    ESP_LOGI(TAG, "event: %s, len: %d", evt_name[event], len);
}

#define RECVTIMEOUT (60*1000)	// 60 sec

static void tcp_client(struct netconn *conn)
{
    struct netbuf *nbuf;

    //netconn_set_recvtimeout(conn, RECVTIMEOUT);
    while (netconn_recv(conn, &nbuf) == ERR_OK) {
	ESP_LOGD(TAG, "netconn_recv(): %d byte", netbuf_len(nbuf));

	// do process KISS protocol (SLIP)
	do {
	    uint8_t *data;
	    u16_t len;

	    netbuf_data(nbuf, (void **)&data, &len);

	    // do process about data[] len bytes

	} while (netbuf_next(nbuf) >= 0);
	netbuf_delete(nbuf);
    }
}

//static RingbufHandle_t tcp_ringbuf = NULL;

#define TCP_RINGBUF_SIZE (1024 * 16)

struct RINGCONN { // ringbuffer and connection
    RingbufHandle_t ringbuf;
    struct netconn *conn;
};

static void tcp_writer_task(void *p)
{
    struct RINGCONN *rcp = (struct RINGCONN *)p;
    uint8_t *item[2];
    size_t size[2];

    while (1) {
        if (xRingbufferReceiveSplit(rcp->ringbuf, (void **)&item[0], (void **)&item[1], &size[0], &size[1], portMAX_DELAY) == pdTRUE) { 

	    // do process about item[2], size[2]
	    netconn_write(rcp->conn, item[0], size[0], NETCONN_COPY);
	    if (item[1]) netconn_write(rcp->conn, item[1], size[1], NETCONN_COPY);
	    // release ringbuffer
	    vRingbufferReturnItem(rcp->ringbuf, item[0]);
	    if (item[1]) vRingbufferReturnItem(rcp->ringbuf, item[1]);
	}
    }
}

static void tcp_reader_task(void *p)
{
    struct netconn *newconn = (struct netconn *)p;
    struct RINGCONN rc;
    TaskHandle_t tcp_writer = NULL;

    rc.conn = newconn;
    rc.ringbuf = xRingbufferCreate(TCP_RINGBUF_SIZE, RINGBUF_TYPE_ALLOWSPLIT);
    if (rc.ringbuf == NULL) {
	ESP_LOGD(TAG, "xRingbufferCreate() fail\n");
    }
    if (rc.ringbuf != NULL) {

	// register ringbuf
	if (uart_add_ringbuf(rc.ringbuf)) {
	    if (xTaskCreate(tcp_writer_task, "tcp_writer", 1024*4, &rc, tskIDLE_PRIORITY, &tcp_writer) == pdPASS) {
		tcp_client(newconn);
		ESP_LOGI(TAG, "tcp_client() returned");
		vTaskDelete(tcp_writer); // stop writer task
	    }
	    // deregister ringbuf
	    uart_delete_ringbuf(rc.ringbuf);
	}
	vRingbufferDelete(rc.ringbuf);
    }

    netconn_close(newconn);
    netconn_delete(newconn);

    vTaskDelete(NULL); // delete this task
}

static void wifi_task(void *p)
{
    err_t xErr;
    struct netconn *conn;
    struct netconn *newconn;
    TaskHandle_t tcp_reader = NULL;

    while (1) {
#ifndef CONFIG_WIFI_SOFTAP
	xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
	ESP_LOGI(TAG, "WiFi connected");
#endif

	conn = netconn_new_with_callback(NETCONN_TCP, callback);
	if (conn == NULL) continue;
	xErr = netconn_bind(conn, IP_ADDR_ANY, TNC_PORT);
	if (xErr != ERR_OK) continue;
	xErr = netconn_listen(conn);
	if (xErr != ERR_OK) continue;

	while (1) {
	    xErr = netconn_accept(conn, &newconn);
	    ESP_LOGD(TAG, "netconn_accept(): %d", xErr);
	    if (xErr != ERR_OK) continue;

	    // create tcp reader task
	    if (xTaskCreate(tcp_reader_task, "tcp_read", 1024*4, newconn, tskIDLE_PRIORITY, &tcp_reader) != pdTRUE) {
		ESP_LOGD(TAG, "xTaskCreate(tcp_reader_task) fail\n");
	    }
	}
    }
}

#if defined(CONFIG_WIFI_ENABLE) && !defnded(CONFIG_WIFI_SOFTAP)
static void wifi_init_sta(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
        },
    };

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    ESP_LOGI(TAG, "connect to ap SSID:%s password:%s",
             CONFIG_WIFI_SSID, CONFIG_WIFI_PASSWORD);

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // for I2S builtin ADC 
    ESP_ERROR_CHECK(esp_wifi_start() );

#ifdef USE_WPS
    ESP_ERROR_CHECK(esp_wifi_start() );
    ESP_LOGI(TAG, "start wps...");

    ESP_ERROR_CHECK(esp_wifi_wps_enable(&config));
    ESP_ERROR_CHECK(esp_wifi_wps_start(0));
#endif
}
#endif

void wifi_start(void)
{
    esp_log_level_set(TAG, ESP_LOG_WARN);

#ifdef CONFIG_WIFI_EABLR
#ifndef CONFIG_WIFI_SOFTAP
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
#else
    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    softap_init();
#endif
#endif

    xTaskCreate(wifi_task, "wifi_task", 4096, NULL, tskIDLE_PRIORITY, NULL);
}
#endif
