#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "driver/adc.h"
#include "esp32/rom/crc.h"
#include "driver/uart.h"

#include "tcb.h"
#include "bell202.h"
#include "i2s_adc.h"

#define TAG "uart"

RingbufHandle_t uart_rb;
TaskHandle_t task;

static const uart_port_t uart_num = UART_NUM_0;

#define AX25_ADDR_LEN 7
#define AX25_MIN_PKT_SIZE (AX25_ADDR_LEN * 2 + 1 + 2) // src addr + dst addr + Control + FCS
#define AX25_SSID_MASK 0x0f

#define BUF_SIZE 16

static char buf[BUF_SIZE];

static void uart_putchar(int c)
{
    char ch = c;

    if (ch >= ' ' && ch <= '~') {
	uart_write_bytes(uart_num, &ch, sizeof(ch));
    } else {
	uart_write_bytes(uart_num, buf, snprintf(buf, BUF_SIZE, "<%02x>", ch));
    }
}

static void uart_put_ssid(int ssid)
{
    int n = ssid & AX25_SSID_MASK;

    if (n > 0) uart_write_bytes(uart_num, buf, snprintf(buf, BUF_SIZE, "-%d", n));
}

static void uart_put_number(int num)
{
    uart_write_bytes(uart_num, buf, snprintf(buf, BUF_SIZE, "%d", num));
}

#define KISS_BUF 1

#ifdef KISS_BUF

#define KISS_BUF_SIZE 256
#define FEND 0xc0
#define FESC 0xdb
#define TFEND 0xdc
#define TFESC 0xdd
#define TCP_RB_NUM 4

static RingbufHandle_t tcp_rb[TCP_RB_NUM];

int uart_add_ringbuf(RingbufHandle_t rb)
{
    for (int i = 0; i < TCP_RB_NUM; i++) {
	if (tcp_rb[i] == NULL) {
	    tcp_rb[i] = rb;
	    return true;
	}
    }
    return false;
}

void uart_delete_ringbuf(RingbufHandle_t rb)
{
    for (int i = 0; i < TCP_RB_NUM; i++) {
	if (tcp_rb[i] == rb) {
	    tcp_rb[i] = NULL;
	    break;
	}
    }
}

static void kiss_buf_add(uint8_t buf[], int *index, uint8_t c)
{
    switch (c) {
	case FEND:
	    if (*index < KISS_BUF_SIZE) buf[(*index)++] = FESC;
	    if (*index < KISS_BUF_SIZE) buf[(*index)++] = TFEND;
	    break;
	case FESC:
	    if (*index < KISS_BUF_SIZE) buf[(*index)++] = FESC;
	    if (*index < KISS_BUF_SIZE) buf[(*index)++] = TFESC;
	    break;
	default:
	    if (*index < KISS_BUF_SIZE) buf[(*index)++] = c;
    }
}
#endif // KISS_BUF

static void ax25_dump_packet(uint8_t *item[2], size_t size[2])
{
#ifdef KISS_BUF
    static uint8_t kiss_buf[KISS_BUF_SIZE];
    int kiss_idx;
#endif
    int i;
    int in_addr = 1;
    int len = size[0];
    //uint8_t *data = (uint8_t *)item[0];
    int num;
   
    if (item[1] != NULL) len += size[1];

    if (len < AX25_MIN_PKT_SIZE) return;

    //fcs = data[len - 1] << 8 | data[len - 2];
    //crc = crc16_le(0, data, len - 2);
    //if (fcs != crc) return;

    //printf("%d:%d:", tp->port, ++tp->pkts);

    // extract FCS field
    i = len - 2;
    num = (i < size[0]) ? item[0][i] : item[1][i - size[0]]; // lower 8 bit
    i++;
    num |= ((i < size[0]) ? item[0][i] : item[1][i - size[0]]) << 8; // upper 8bit

    uart_put_number(num >> 12); // port No
    uart_putchar(':');
    uart_put_number(num & 0x0fff); // pkt cnt
    uart_putchar(':');

#ifdef KISS_BUF
    kiss_idx = 0;
    kiss_buf[kiss_idx++] = FEND;
    kiss_buf[kiss_idx++] = (num & 0xf000) >> 8; // port No.
#endif

    for (i = 0; i < len - 2; i++) {
	int c;
	int d;

	c = (i < size[0]) ? item[0][i] : item[1][i - size[0]];
	d = c >> 1; // for addr field shift

	if (in_addr) {
	    if (i % AX25_ADDR_LEN == AX25_ADDR_LEN - 1) { // SSID
		uart_put_ssid(d);
		uart_putchar(',');
	    } else {
		if (d != ' ') uart_putchar(d);
	    }
	    in_addr = (c & 1) == 0;
	} else {
	    uart_putchar(c);
	}
#ifdef KISS_BUF
	kiss_buf_add(kiss_buf, &kiss_idx, c);
#endif
    }
    uart_write_bytes(uart_num, "\r\n", 2);
    //printf("<%02x%02x>\n", data[len-1], data[len-2]);
    //
#ifdef KISS_BUF
    if (kiss_idx < KISS_BUF_SIZE) {
	kiss_buf[kiss_idx++] = FEND;

        for (int i = 0; i < TCP_RB_NUM; i++) {
	    if (tcp_rb[i]) {
		if (xRingbufferSend(tcp_rb[i], kiss_buf, kiss_idx, 0) != pdTRUE)
		    ESP_LOGW(TAG, "xRingbufferSend() fail");
	    }
	}
    }
#endif
}

static void uart_task(void *p)
{
    RingbufHandle_t rb = (RingbufHandle_t)p;
    uint8_t *item[2];
    size_t size[2];

    while (1) {
	if (xRingbufferReceiveSplit(rb, (void **)&item[0], (void **)&item[1], &size[0], &size[1], portMAX_DELAY) != pdTRUE) {
	    ESP_LOGI(TAG, "xRingbufferReceiveSplit() return not pdTRUE");
	    continue;
	}

#if 1
	ax25_dump_packet(item, size);
#endif
	
	vRingbufferReturnItem(rb, item[0]);
	if (item[1] != NULL) vRingbufferReturnItem(rb, item[1]);
    }
}

#define UART_RB_SIZE (1024 * 2)
#define UART_RX_BUF_SIZE 256
#define UART_TX_BUF_SIZE (1024 * 1)

void uart_init(void)
{
    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, 0, NULL, ESP_INTR_FLAG_LEVEL1));

    uart_rb = xRingbufferCreate(UART_RB_SIZE, RINGBUF_TYPE_ALLOWSPLIT);
    if (uart_rb == NULL) {
	ESP_LOGE(TAG, "xRingbufferCreate() fail");
	abort();
    }

    if (xTaskCreate(uart_task, "uart task", 1024 * 4, uart_rb, tskIDLE_PRIORITY, &task) != pdPASS) {
	ESP_LOGE(TAG, "xTaskCreate() fail");
	abort();
    }
}
