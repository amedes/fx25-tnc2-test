/*
 * encode.c
 *
 * generate bits train to send modem
 */
#include <stdio.h>
#include <sys/time.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <driver/gpio.h>

#include "packet_table.h"

#define TAG "encode"

#define BUF_SIZE 256
#define AX25_FLAG 0x7e
#define TNC_N 2

//static QueueHandle_t tx_queue;

typedef struct TNC_INFO {
    QueueHandle_t queue;
    uint8_t nrzi_bit;
    int pkt_cnt;
    int byte_cnt;
    int bit_cnt; // counter for sending bits
    uint8_t one_cnt;
    uint8_t port;
} tnc_info_t;

tnc_info_t tnc_info[TNC_N];

#if 0
void send_bit(uint8_t bit)
{
    byte_buf >>=1;
    if (bit) byte_buf |= 0x80; // insert at MSb  
    if (++byte_cnt >= 8) {
	if (xQueueSend(tx_queue, &byte_buf, portMAX_DELAY) != pdTRUE) {
	    ESP_LOGW(TAG, "xQueueSend() fail");
	}
	byte_cnt = 0;
    }
}
#else
void send_bit(uint8_t bit, tnc_info_t *tp)
{
    //static uint8_t nrzi_bit = 0;
    uint32_t data;

    if (!bit) tp->nrzi_bit = !tp->nrzi_bit; // invert bit if 0

    // send bit info. to transmitter task
    data = tp->nrzi_bit | 0x02; // data length is one bit
    if (xQueueSend(tp->queue, &data, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "xQueueSend() fail");
    }
    tp->bit_cnt++;
}

#endif


static void send_byte(uint8_t byte, int stuffing, tnc_info_t *tp)
{
    uint8_t b = byte;

    for (int i = 0; i < 8; i++) {
	uint8_t bit = b & 1;

	b >>= 1;
	send_bit(bit, tp);

	// bit stuffing
	switch (bit) {
	case 1:
	    if (++tp->one_cnt < 5) break;
	    if (stuffing) send_bit(0, tp);
	case 0:
	    tp->one_cnt = 0;
	}
    }
    if (!stuffing) tp->one_cnt = 0;

    tp->byte_cnt++;
}

static void send_packet(uint8_t const *buf, size_t len, tnc_info_t *tp)
{
    if (len <= 0) return;

    //send_byte(AX25_FLAG, false, tp); // start flag and/or end flag
    for (int i = 0; i < 75; i++) send_byte(AX25_FLAG, false, tp); // preamble 500ms

    for (int i = 0; i < len; i++) send_byte(buf[i], true, tp);

    send_byte(AX25_FLAG, false, tp); // end flag

}

extern uint32_t i2s_period_max;
extern uint32_t i2s_period_max_tx;

static EventGroupHandle_t event_group;

static void packet_task(void *q)
{
    int len;
    unsigned const char *p;
    struct timeval tv0, tv1;
    tnc_info_t *tp = (tnc_info_t *)q;
    unsigned int seed = (unsigned int)tp->queue;

    // random delay
    //for (int i = rand_r(&seed) % 1000; i > 0; --i) send_byte(AX25_FLAG, false, tp);

    while (1) {
	p = packet_table;
	tp->pkt_cnt = 0;
	tp->byte_cnt = 0;
	tp->bit_cnt = 0;
	
	gettimeofday(&tv0, NULL); // start time

    	send_bit(1, tp); // start with MARK state
	//for (int i = 0; i < 1; i++) send_byte(AX25_FLAG, false, tp);

#define PTT0_GPIO GPIO_NUM_23
#define PTT1_GPIO GPIO_NUM_22

	while ((len = *p++) > 0) {

extern EventGroupHandle_t i2s_event_ptt;

	    //xEventGroupClearBits(i2s_event_ptt, 1 << tp->port);

#define WAIT_BIT ((1 << 0) | (1 << 1))

	    // synchronize sending packet timing
	    while (xEventGroupSync(event_group, 1 << tp->port, WAIT_BIT, portMAX_DELAY) != WAIT_BIT) {
	        ESP_LOGW(TAG, "xEventGroupSync() fail");
	    }

	    if (tp->port == 0) gpio_set_level(PTT0_GPIO, 1); // PTT on
	    if (tp->port == 1) gpio_set_level(PTT1_GPIO, 1); // PTT on

	    send_packet(p, len, tp);
	    //send_byte(AX25_FLAG, false, tp); // end flag


	    // wait for sending packet
	    xEventGroupWaitBits(i2s_event_ptt, 1 << tp->port, pdTRUE, pdTRUE, portMAX_DELAY);
	    vTaskDelay(5000 / portTICK_PERIOD_MS);

	    p += len;
	    tp->pkt_cnt++;
	}

	send_byte(AX25_FLAG, false, tp); // end flag

	gettimeofday(&tv1, NULL); // end time
	int usec = tv1.tv_usec - tv0.tv_usec;
	int sec = tv1.tv_sec - tv0.tv_sec;
	if (usec < 0) {
	    usec += 1000000;
	    --sec;
	}

	ESP_LOGI(TAG, "sending %d pkts, %d bytes, %d bits, time: %d.%06d sec, period_max: %u, _tx: %u", tp->pkt_cnt, tp->byte_cnt, tp->bit_cnt, sec, usec, i2s_period_max, i2s_period_max_tx);

	extern int q0_empty, q1_empty;
	ESP_LOGI(TAG, "q0_empty: %d, q1_empty: %d", q0_empty, q1_empty);

        //vTaskDelay((rand_r(&seed) % 20000) / portTICK_PERIOD_MS);
	//vTaskDelete(NULL);
    }
}

void encode_init(QueueHandle_t queue)
{
    static uint8_t port_index = 0;
    struct TNC_INFO *tp;
    //tx_queue = queue;
    
    if (port_index >= TNC_N) return;

    if (port_index == 0) {
	event_group = xEventGroupCreate();
	if (event_group == NULL) {
	    ESP_LOGE(TAG, "xEventGroupCreate() fail");
	    abort();
	}
    }

    ESP_LOGI(TAG, "port_index: %d, queue = %p", port_index, queue);

    tp = &tnc_info[port_index];
    tp->port = port_index++;
    tp->queue = queue;

    xEventGroupClearBits(event_group, 1 << tp->port);

    // create packet sender task
    if (xTaskCreate(packet_task, "packet task", 1024 * 2, tp, 5, NULL) != pdPASS) {
	ESP_LOGE(TAG, "xTaskCreate() fail");
	abort();
    };
}
