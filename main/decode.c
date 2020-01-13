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
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "driver/adc.h"
#include "esp32/rom/crc.h"

#include "tcb.h"
#include "bell202.h"
#include "i2s_adc.h"
#include "uart.h"
#include "gpio.h"

#define USE_DECODE_BITS 1

#define TAG "decode"
//static RingbufHandle_t ringbuf;

#define BAUD_RATE 1200
//#define SAMPLING_RATE 6726
#define SAMPLING_RATE 13200
#define BIT_DURATION (SAMPLING_RATE / BAUD_RATE)

#define AX25_ADDR_LEN 7
#define AX25_MIN_PKT_SIZE (AX25_ADDR_LEN * 2 + 1 + 2) // src addr + dst addr + Control + FCS
#define AX25_SSID_MASK 0x0f

static int ax25_check_fcs(uint8_t data[], int len)
{
    uint16_t fcs, crc;

    if (len < AX25_MIN_PKT_SIZE) return false;

    //gpio_set_level(GPIO_NUM_27, 1);

    fcs = data[len - 1] << 8 | data[len - 2];
    crc = crc16_le(0, data, len - 2);

    //gpio_set_level(GPIO_NUM_27, 0);

    return fcs == crc;
}

static void output_packet(tcb_t *tp)
{
    tp->pkts++;
    //output_packet(tp, tp->data, tp->data_cnt);
    //
    // store port and pkts into fcs field
    tp->data[tp->data_cnt - 2] = tp->pkts & 0xff; // pkts of lower 8 bit
    tp->data[tp->data_cnt - 1] = (tp->port << 4) | ((tp->pkts >> 8) & 0x0f);  // 7-4 bit: port No., 3-0 bit: pkts of upper 4 bit

    if (xRingbufferSend(uart_rb, tp->data, tp->data_cnt, 1) != pdTRUE) {
	ESP_LOGI(TAG, "xRingbufferSend() fail: %d", tp->port);
    }
}

#define AX25_FLAG 0x7e
#define AX25_MASK 0xfc // bit mask of MSb six bits
#define AX25_EOP 0xfc  // end of packet, 7e << 1
#define AX25_STUFF_BIT 0x7c // bit stuffing bit, five of continuous one bits
//#define DATA_LEN (1024+2) // maximum packet size

#ifndef USE_DECODE_BITS

static void decode_bit(tcb_t *tp, uint8_t bit)
{
    //static uint8_t state = FLAG;
    //static uint8_t flag = 0;
    //static uint8_t data[DATA_LEN];
    //static int data_cnt = 0;
    //static uint8_t data_byte = 0;
    //static uint8_t data_bit_cnt = 0;

    tp->flag >>= 1;
    tp->flag |= bit << 7;

    switch (tp->state) {
    case FLAG:
	if (tp->flag == AX25_FLAG) { // found flag
	    tp->state = DATA;
	    tp->data_cnt = 0;
	    tp->data_bit_cnt = 0;
	    //cnt = (edge + SAMPLING_N/2) % SAMPLING_N; // bit sync
	    //ESP_LOGI(TAG, "found AX25_FALG");
	}
	break;

    case DATA:
	if ((tp->flag & AX25_MASK) == AX25_EOP) { // AX.25 flag, end of packet
	    if (ax25_check_fcs(tp->data, tp->data_cnt)) { // FCS ok

		output_packet(tp);
#if 0
		tp->pkts++;
	        //output_packet(tp, tp->data, tp->data_cnt);
		
		// store port and pkts into fcs field
		tp->data[tp->data_cnt - 2] = tp->pkts & 0xff; // pkts of lower 8 bit
		tp->data[tp->data_cnt - 1] = (tp->port << 4) | ((tp->pkts >> 8) & 0x0f);  // 7-4 bit: port No., 3-0 bit: pkts of upper 4 bit
		
		if (xRingbufferSend(uart_rb, tp->data, tp->data_cnt, portMAX_DELAY) != pdTRUE) {
		    ESP_LOGW(TAG, "xRingbufferSend() fail: %d", tp->port);
		}
#endif
	    }
	    tp->state = FLAG;
	    break;
	}

	if ((tp->flag & AX25_MASK) == AX25_STUFF_BIT) break; // delete bit stuffing bit

	tp->data_byte >>= 1;
	tp->data_byte |= bit << 7;
	tp->data_bit_cnt++;
	if (tp->data_bit_cnt >= 8) {
	    if (tp->data_cnt < DATA_LEN) tp->data[tp->data_cnt++] = tp->data_byte;
	    tp->data_bit_cnt = 0;
	}
    }
}

#else

static void decode_bits(tcb_t *tp, uint8_t bits)
{
    if (bits >= 8) { // bit length too long
	tp->state = FLAG;
	return;
    }

    switch (tp->state) {
    case FLAG:
	if (bits == 7) { // AX.25 flag
	    tp->state = DATA;
	    tp->data_cnt = 0;
	    tp->data_bit_cnt = 0; // delete next zero
	}
	break;

    case DATA:
	if (bits == 7) { // AX.25 flag
	    if ((tp->data_bit_cnt == 1) && ax25_check_fcs(tp->data, tp->data_cnt)) { // 1 means next zero, octet boundary
		output_packet(tp);
	    }
	    tp->state = DATA;
	    tp->data_cnt = 0;
	    tp->data_bit_cnt = 0; // delete next zero
	    break;
	}

	if (bits == 6) { // bit stuffing, do not insert trailing zero
	    tp->data_byte >>= 5;
	    tp->data_byte |= 0xf800; // insert '11111' bit pattern
	    tp->data_bit_cnt += 5;
	} else {
	    tp->data_byte >>= bits;
	    tp->data_byte |= 0x8000 - (1 << (16 - bits)); // generate '011...' bit pattern (total "bits" length)
	    tp->data_bit_cnt += bits;
	}

	// move one octet length bits to packet buffer
	if (tp->data_bit_cnt >= 8) {
	    if (tp->data_cnt >= DATA_LEN) { // packet too long, discard
		ESP_LOGW(TAG, "packet too long, %d", tp->port);
		tp->state = FLAG;
		break;
	    }
	    tp->data[tp->data_cnt++] = tp->data_byte >> (16 - tp->data_bit_cnt); // extract top 8 bits
	    tp->data_bit_cnt -= 8;
	}
    }
}
#endif

static void decode(tcb_t *tp, int val)
{
    //static int pval = 0;
    //static int edge = 0;

    tp->edge++;
    if (val != tp->pval) {
	int bits = (tp->edge * BAUD_RATE + SAMPLING_RATE/2) / SAMPLING_RATE;
	//int bit_pattern = ~1; // 0111...

#ifdef USE_DECODE_BITS
	decode_bits(tp, bits);
#else
	// process multiple bits
	decode_bit(tp, 0); // begin with zero
	while (--bits > 0) {
	    decode_bit(tp, 1); // successive bit is all one
	}
#endif
	tp->edge = 0;
	tp->pval = val;
    }
}

void demodulator(tcb_t *tp, uint16_t adc)
{
    // static int avg = 2048;
    int bit;
    int dif;

    //printf("%d,", adc);

	//dac_output_voltage(DAC_CHANNEL_1, adc >> 4);

#define AVERAGE_N 16

#if 0
	static int sum = 0, count = 0;

	sum += adc;
	if (++count >= AVERAGE_N) {
	    tp->cdt = sum / AVERAGE_N;
	    //ESP_LOGI(TAG, "average adc value = %d",  average);
	    sum = 0;
	    count = 0;
	}
#else
#define CDT_THRESHOLD0 (1 << 12)
#define CDT_THRESHOLD1 (1 << 11)
	// carrier detect
	    dif = adc - tp->avg;
	    dif *= dif;
	    tp->cdt = (tp->cdt * (AVERAGE_N - 1) + dif) / AVERAGE_N;
	    //gpio_set_level(GPIO_NUM_14, !(tp->cdt > CDT_THRESHOLD)); // 0: LED ON
	    if (tp->port == 0) {
	    if (tp->cdt > CDT_THRESHOLD0) gpio_set_level(GPIO_NUM_14, 0); // ON
	    else if (tp->cdt <= CDT_THRESHOLD1) gpio_set_level(GPIO_NUM_14, 1); // off
	    }
	// caluculate average value
	tp->avg = (tp->avg * (AVERAGE_N - 1) + adc) / AVERAGE_N;
#if 0
	count++;
	if (count >= 13200) {
		printf("average = %d\n", average);
		count = 0;
	}
#endif
#endif

#define LPF_N SAMPLING_N

	// deocde bell 202
	bit = bell202_decode(tp, (int)adc - tp->avg) < 0;
	if (tp->port == 0) gpio_set_level(GPIO_NUM_12, bit);
	//lpf = (lpf * (LPF_N-1) + val) / LPF_N;
	//bit = (val < 0);

	//dac_output_voltage(DAC_CHANNEL_2, bit * 128);
	// decode AX.25 packet, reverse NRZI, delete bit stuffing bit, etc...
	decode(tp, bit);
}
