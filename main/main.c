/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "driver/adc.h"
#include "esp32/rom/crc.h"

#include "tcb.h"
#include "bell202.h"
#include "i2s_adc_dac.h"
#include "decode.h"
#include "uart.h"
#include "cw.h"
#include "encode.h"
#include "gpio.h"

#ifdef CONFIG_WIFI_ENABLE
#include "wifi.h"
#endif

//#define USE_QUEUE 1

//#define TNC_PORTS 6
#define TNC_PORTS 2
//#define TNC_PORTS 12

static const char TAG[] = "main";
static RingbufHandle_t ringbuf;
tcb_t tcb[TNC_PORTS];

#define AF_IN_ADC_CH ADC1_CHANNEL_0
//#define AF_IN_ADC_CH ADC1_CHANNEL_3
//#define AF_IN_ADC_CH ADC1_CHANNEL_4
//#define AF_IN_ADC_CH ADC1_CHANNEL_5
//#define AF_IN_ADC_CH ADC1_CHANNEL_6
//#define AF_IN_ADC_CH ADC1_CHANNEL_7

#define MAIN_GPIO_PIN GPIO_NUM_27

static void read_ringbuf(void *p)
{
    uint16_t *buf;
    size_t size;
    //size_t len = 0;
    //int errors = 0;
    RingbufHandle_t rb = (RingbufHandle_t)p;

    while (1) {
	//gpio_set_level(MAIN_GPIO_PIN, 0);
	buf = xRingbufferReceive(rb, &size, portMAX_DELAY);
	//gpio_set_level(MAIN_GPIO_PIN, 1);
	if (buf == NULL) {
	    ESP_LOGI(TAG, "xRingbufferReceive() return NULL");
	    continue;
	}
	
#if 0
	if (size != len) {
	    ESP_LOGI(TAG, "xRingbufferReceive: size = %d", size);
	    len = size;
	}
	for (int i = 0; i < 16; i++) {
#if 0
	    if ((buf[i] >> 12) == 0) printf("%04x ", buf[i]);
#else
	    printf("%04x ", buf[i]);
#endif
	}
	printf("\n");
#endif

	for (int i = 0; i < size / sizeof(uint16_t); i++) {
	    uint16_t adc = buf[i];
	    uint8_t port = adc >> 12;

#if TNC_PORTS > 6
	    if ((i & 1) == 0) { // adc2
		if (port < 4) continue;
		port += 6 - 4; // adc2 ch_4 - ch_9, port No. 6 - 11
	    } else {
		if (port != 0) port -= 2; // adc1 ch_0, ch_3 - ch_7, port No. 0, 1 - 5
	    }
#else
	    if (port > 0) port -= 2; // skip ADC1_CH1 and 2
	    //if (port > 1) ESP_LOGI(TAG, "adc channel number: %d", port);
#endif
	    adc &= 0xfff; // clear adc channel number

#ifdef USE_QUEUE 
	    // send adc sample to queue
	    if (port < TNC_PORTS) {
		if (xQueueSend(tcb[port].queue, &adc, 0) != pdTRUE) {
		    ESP_LOGI(TAG, "xQueueSend() fail, %d", port);
		}
	    }
#else
	    demodulator(&tcb[port], adc);
#endif

	}
#define CDT_THRESHOLD 4096
	//led_cdt(tcb[0].cdt > CDT_THRESHOLD);
	vRingbufferReturnItem(rb, buf);
    }
}

#ifdef USE_QUEUE
void decode_task(void *p)
{
    tcb_t *tp = (tcb_t *)p;
    uint16_t adc;

    while (1) {
	if (xQueueReceive(tp->queue, &adc, portMAX_DELAY) != pdTRUE) {
	    ESP_LOGI(TAG, "xQueueReceive() fail, %d", tp->port);
	    continue;
	}

	demodulator(tp, adc);
    }
}
#endif

static void tcb_init(tcb_t *tcb, int ports)
{
    // initialize TNC Control Block
    for (int i = 0; i < ports; i++) {
	tcb_t *tp = &tcb[i];

	tp->port = i;
	tp->avg = 2048;

#ifdef USE_QUEUE
	tp->queue = xQueueCreate(TCB_QUEUE_LENGTH, TCB_QUEUE_ITEM_SIZE);
	if (tp->queue == NULL) {
	    ESP_LOGE(TAG, "xQueueCreate() fail");
	    abort();
	}

	// creating task for decoding
	if (xTaskCreate(decode_task, "decode task", 1024 * 4, tp, tskIDLE_PRIORITY + 1, tp->task) != pdPASS) {
	    ESP_LOGE(TAG, "xTaskCreation() fail");
	    abort();
	}
#endif
    }
}

#define RINGBUF_SIZE (1024 * 2)
#define TX_QUEUE_SIZE (128 * 8)

void app_main(void)
{
#ifdef CONFIG_WIFI_ENABLE
    wifi_start();
#endif

    // initialize Bell202 decoder
    bell202_init();

    // initialize UART
    uart_init();

    // CDT LED init
    gpio_init();

    // initialize TCB
    tcb_init(tcb, TNC_PORTS);

    ringbuf = xRingbufferCreate(RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (ringbuf == NULL) {
	ESP_LOGE(TAG, "xRingbufferCreate() fail");
	abort();
    }

    // create two packet sending tasks
    QueueHandle_t queue0 = xQueueCreate(TX_QUEUE_SIZE, sizeof(uint8_t));
    if (queue0 == NULL) {
	ESP_LOGE(TAG, "xQueueCreate() fail");
	abort();
    }

    QueueHandle_t queue1 = xQueueCreate(TX_QUEUE_SIZE, sizeof(uint8_t));
    if (queue1 == NULL) {
	ESP_LOGE(TAG, "xQueueCreate() fail");
	abort();
    }

    i2s_init(ringbuf, queue0, queue1);
    encode_init(queue0);
    encode_init(queue1);

    // initialize cosine wave generator
//    cw_init(queue);

    if (xTaskCreate(read_ringbuf, "read_ringbuf", 1024 * 4, ringbuf, tskIDLE_PRIORITY+1, NULL) != pdPASS) {
	ESP_LOGE(TAG, "xTaskCreate fail: read_ringbuf");
    }

    ESP_LOGI(TAG, "size of EventBits_t: %d", sizeof(EventBits_t));

    // blink STA LED
    int led = 0;
    while (1) {
	gpio_set_level(GPIO_NUM_27, led & 1);
	gpio_set_level(GPIO_NUM_21, led & 2);
	led++;
	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // end of initialization
    vTaskDelete(NULL);
}
