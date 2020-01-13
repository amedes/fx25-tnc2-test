#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2s.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"
#include "soc/i2s_periph.h"
#include "soc/sens_periph.h"
#include "soc/syscon_periph.h"

static const char TAG[] = "I2S";

//i2s number
#define I2S_NUM           (0)

#define BAUD_RATE	1200
#define SAMPLES_PER_ONE_BIT	11

#define FACTOR			2
#define I2S_SAMPLE_RATE   (BAUD_RATE * SAMPLES_PER_ONE_BIT * FACTOR)

#define I2S_DMA_BUF_BITS	1 // number of bits stored in a DMA buffer

#define I2S_DMA_BUF_COUNT	2
#define I2S_DMA_BUF_LEN		(SAMPLES_PER_ONE_BIT * FACTOR * I2S_DMA_BUF_BITS)

#define OUTPUT_PIN		GPIO_NUM_27

static QueueHandle_t i2s_queue;

/**
 * @brief I2S ADC/DAC mode init.
 */
static void i2s_config(void)
{
	i2s_port_t i2s_num = I2S_NUM;
	i2s_config_t i2s_config = {
            .mode = I2S_MODE_MASTER
		    //| I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN
		    | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN
		    ,
	    .sample_rate =  I2S_SAMPLE_RATE,
	    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
	    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
	    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
	    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
	    .dma_buf_count = I2S_DMA_BUF_COUNT,
	    .dma_buf_len = I2S_DMA_BUF_LEN,
	    .use_apll = false,
	};

	//install and start i2s driver
	ESP_ERROR_CHECK(i2s_driver_install(i2s_num, &i2s_config, 32, &i2s_queue));
	// stop I2S for setup
	//ESP_ERROR_CHECK(i2s_stop(i2s_num));
	// start I2S
	//ESP_ERROR_CHECK(i2s_start(i2s_num));

	//init DAC pad
	ESP_ERROR_CHECK(i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN));

	// delay for I2S bug workaround?
        vTaskDelay(10 / portTICK_PERIOD_MS);

	// output signal state
	gpio_set_direction(OUTPUT_PIN, GPIO_MODE_OUTPUT);
}

/*
 * I2S read task
 */
static void i2s_task(void*arg)
{
    //size_t bytes_read;
    size_t bytes_written;
    static uint16_t i2s_read_buff[I2S_DMA_BUF_LEN * 2]; // *2 stereo
    int i2s_read_len = sizeof(i2s_read_buff);
    i2s_event_t event;
    QueueHandle_t queue = (QueueHandle_t)arg;

//#define FACTOR 2
#define BAUD_RATE 1200
#define BAUD_DIV 11
#define MARK_DIV 11
#define SPACE_DIV 6
#define MARK_MUL 6
#define SPACE_MUL 11
#define BAUD_SAMPLES (MARK_DIV * FACTOR)
#define MARK_SAMPLES (MARK_DIV * FACTOR)
#define SIN_SAMPLES (BAUD_DIV * FACTOR)
#define SIN_AMPLI_BIT 16
#define I2S_BIT_PER_SAMPLE 16
#define SIN_AMPLI ((1 << SIN_AMPLI_BIT) - 1)
#define SIN_PHASE 6
#define SIN_CHANNEL 2
#define SIN_FREQ 2
#define SIN_PHASE_DIV (MARK_DIV * SPACE_DIV * FACTOR)
#define SIN_CYCLES (MARK_DIV * SPACE_DIV)
    static uint16_t sin_table [SIN_PHASE][SIN_PHASE][SIN_FREQ][SIN_FREQ][SIN_SAMPLES][SIN_CHANNEL];
    const static uint8_t sin_freq[2] = { SPACE_DIV, MARK_DIV }; // 0: mark, 1: space
    //const static uint8_t sin_phase[2] = { MARK_MUL, SPACE_MUL }; // 0: mark, 1: space
#define SPACE_SAMPLES (SPACE_DIV * FACTOR) 
    int i;
    int i0 = 0;
    int i1 = 0;
    int flip = 0;
    uint32_t data = 0;
    uint8_t ptt = 0;
    uint8_t add0 = 0;
    uint8_t p0 = 0, p1 = 0;
    uint8_t f0 = 0, f1 = 0;

    ESP_LOGI(TAG, "IS2_DMA_BUF_LEN: %d", I2S_DMA_BUF_LEN);
    ESP_LOGI(TAG, "i2s_read_len: %d", i2s_read_len);

    for (int p0 = 0; p0 < SIN_PHASE; p0++) {
	for (int p1 = 0; p1 < SIN_PHASE; p1++) {
	    for (int f0 = 0; f0 < SIN_FREQ; f0++) {
		for (int f1 = 0; f1 < SIN_FREQ; f1++) {
    	    	    for (i = 0; i < SIN_SAMPLES; i++) {
		        uint16_t val;

	       	        val = (1.0 + cos((i * SIN_CYCLES / sin_freq[f0] - p0 * BAUD_DIV * FACTOR) * M_PI * 2.0 / SIN_PHASE_DIV)) / 2.0 * SIN_AMPLI + 0.5;
	       	        //val = (1.0 + sin((i * M_PI * 2.0 / SIN_SAMPLES))) / 2.0 * SIN_AMPLI + 0.5;
	        	sin_table[p0][p1][f0][f1][i][0] = val << (I2S_BIT_PER_SAMPLE - SIN_AMPLI_BIT);
	       	        val = (1.0 + cos((i * SIN_CYCLES / sin_freq[f1] - p1 * BAUD_DIV * FACTOR) * M_PI * 2.0 / SIN_PHASE_DIV)) / 2.0 * SIN_AMPLI + 0.5;
	       	        //val = (1.0 + sin((i * M_PI * 2.0 / SIN_SAMPLES * 11/6))) / 2.0 * SIN_AMPLI + 0.5;
	        	sin_table[p0][p1][f0][f1][i][1] = val << (I2S_BIT_PER_SAMPLE - SIN_AMPLI_BIT);
		    }
		}
	    }
	}
    }

    uint16_t *p = &sin_table[0][0][0][0][0][0];
    for (int j = 0; j < 4; j++) {
    for (i = 0; i < SIN_SAMPLES * 2; i++) {
	printf("%d,", *p++ >> 8);
	//printf("%d,", sin_table[0][0][0][0][i][0]);
    }
    printf("\n");
    }
    printf("\n");

    while (1) {
	if (xQueueReceive(i2s_queue, &event, portMAX_DELAY) != pdTRUE) continue;

	switch (event.type) {
	    case I2S_EVENT_DMA_ERROR:
		ESP_LOGW(TAG, "I2S_EVENT_DMA_ERROR");
		continue;
	    case I2S_EVENT_RX_DONE:
		//ESP_LOGI(TAG, "I2S_EVENT_RX_DONE");
	        continue;
	    case I2S_EVENT_TX_DONE:
		//ESP_LOGI(TAG, "I2S_EVENT_TX_DONE");
	        break;
	    default:
	        continue;
	}

#define PTT_GPIO GPIO_NUM_19
	if ((ptt > 0) && (--ptt == 0)) gpio_set_level(PTT_GPIO, 0);

	flip = !flip;
	gpio_set_level(OUTPUT_PIN, flip);
	i = 0;
	//data = rand(); // one DMA buffer holds of 12 bit AFSK signal (duration is 10ms)

#if 0
	while (i < I2S_DMA_BUF_LEN * 2) {
#else
	{
#endif
	    uint8_t byte;

	    if (data <= 1) {
	        if (xQueueReceive(queue, &data, 0) == pdTRUE) {
		    //data = byte | 0x100; // sentinel

		    if (ptt == 0) {
			gpio_set_level(PTT_GPIO, 1);
			ptt = 4;
		    }
		    //ptt = 2; // ptt off after 13.3ms (8/1200 * 2)
		}
	    }

	    add0 = 0;
	    if (data > 1) {
		add0 = (data & 1) ? MARK_MUL : SPACE_MUL;
		f0 = data & 1;
		data >>= 1;
		ptt = 4;
	    } else if (data == 1) {
		f0 = !f0;
		data = 0;
	    }

	    int add1;
	    add1 = (flip) ? MARK_MUL : 0;
	    f1 = flip;
	    f1 = !f0;

#if 0
	    // generate 1 bit os AFSK signal
	    for (int j = 0; j < BAUD_SAMPLES; j++) {
		// port 0
	        i2s_read_buff[i++] = sin_table[i0] << (I2S_BIT_PER_SAMPLE - SIN_AMPLI_BIT);
	        //i2s_read_buff[i++] = (j & 3) ? 65535 : 0;
	        i0 += add0;
		i0 %= SIN_SAMPLES;

		// port 1
	        i2s_read_buff[i++] = sin_table[i1] << (I2S_BIT_PER_SAMPLE - SIN_AMPLI_BIT);
	        //i2s_read_buff[i++] = (j & 3) ? 65535 : 0;
	        i1 += add1;
		i1 %= SIN_SAMPLES;
	    }
	}

	ESP_ERROR_CHECK(i2s_write(I2S_NUM, (void*) i2s_read_buff, i2s_read_len, &bytes_written, 0));
#else
        }
	ESP_ERROR_CHECK(i2s_write(I2S_NUM, (void*) &sin_table[p0][p1][f0][f1][0][0], i2s_read_len, &bytes_written, 0));
	if (!f0) p0 = (p0 + 1) % SIN_PHASE;
	if (!f1) p1 = (p1 + 1) % SIN_PHASE;
#endif
	gpio_set_level(OUTPUT_PIN, 0);
    }
}

/*
 * Initialize ADC read task
 */
void i2s_init(QueueHandle_t queue)
{
    BaseType_t res;

    i2s_config();

    res = xTaskCreate(i2s_task, "i2s_task", 1024 * 2, queue, 5, NULL);
    if (res != pdTRUE) {
	ESP_LOGE(TAG, "xTaskCreate fail");
	vTaskDelete(NULL);
    }
}
