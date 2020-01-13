#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
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

//#define I2S_DAC_OUTPUT 1
#define MULTI_CHANNEL 1
//#define ENABLE_ADC2 1
#define ENABLE_STEREO 1

//i2s number
#define I2S_NUM           (0)

#define BAUD_RATE	1200

#define CH_N 2 // number of adc channels
#define FACTOR CH_N

#define SAMPLES_PER_ONE_BIT	(11 * FACTOR)

//i2s sample rate
#ifdef ENABLE_ADC2
#define I2S_SAMPLE_RATE   (6726 * 6 * 2)
#else
#define I2S_SAMPLE_RATE   (13200 * CH_N * 1)
#endif

#define I2S_BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_16BIT

#define I2S_DMA_BUF_BITS	3
#define I2S_DMA_BUF_COUNT	64
//#define I2S_DMA_BUF_LEN		1024
//#define I2S_DMA_BUF_LEN		(11 * 12 * CH_N) // 1bit / 833us * 8
#define I2S_DMA_BUF_LEN		(SAMPLES_PER_ONE_BIT * I2S_DMA_BUF_BITS)

#define I2S_QUEUE_LEN	4

//I2S built-in ADC unit
#define I2S_ADC_UNIT              ADC_UNIT_1
//I2S built-in ADC channel
#define I2S_ADC_CHANNEL           ADC1_CHANNEL_0 // GPIO39

#define I2S_GPIO_PIN	GPIO_NUM_27
#define I2S_GPIO_PIN_14	GPIO_NUM_14
#define I2S_GPIO_PIN_12	GPIO_NUM_12
#define I2S_GPIO_PIN_13	GPIO_NUM_13

static QueueHandle_t i2s_queue;
static RingbufHandle_t rxd_ringbuf;
static QueueHandle_t txd0_queue;
static QueueHandle_t txd1_queue;

#define BAUD_RATE 1200
#define BAUD_DIV 11
#define PHASE_DIV 11
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

EventGroupHandle_t i2s_event_ptt;

/**
 * @brief I2S ADC/DAC mode init.
 */
static void i2s_config(void)
{
	i2s_port_t i2s_num = I2S_NUM;
	i2s_config_t i2s_config = {
            .mode = I2S_MODE_MASTER
		    | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN
		    | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN
		    ,
	    .sample_rate =  I2S_SAMPLE_RATE,
	    .bits_per_sample = I2S_BITS_PER_SAMPLE,
	    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
	    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
	    //.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
	    .intr_alloc_flags = 0,
	    .dma_buf_count = I2S_DMA_BUF_COUNT,
	    .dma_buf_len = I2S_DMA_BUF_LEN,
	    .use_apll = false,
	    .tx_desc_auto_clear = true,
	};

	//install and start i2s driver
	ESP_ERROR_CHECK(i2s_driver_install(i2s_num, &i2s_config, I2S_QUEUE_LEN, &i2s_queue));
	// stop I2S for setup
	//ESP_ERROR_CHECK(i2s_stop(i2s_num));
	// start I2S
	//ESP_ERROR_CHECK(i2s_start(i2s_num));

	//init DAC pad
	ESP_ERROR_CHECK(i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN)); // DAC1 on GPIO25, DAC2 on GPIO26

	//init ADC pad
	ESP_ERROR_CHECK(i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL));
	// enable adc
	ESP_ERROR_CHECK(i2s_adc_enable(i2s_num));

	// delay for I2S bug workaround?
        vTaskDelay(10 / portTICK_PERIOD_MS);

	//ESP_ERROR_CHECK(adc_set_data_inv(I2S_ADC_UNIT, true));
	
	// ***IMPORTANT*** enable continuous adc sampling
	SYSCON.saradc_ctrl2.meas_num_limit = 0;

	// channel, attenation, bit width
	SYSCON.saradc_sar1_patt_tab[0] = ((ADC1_CHANNEL_0 << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_6) << 24;

#ifdef MULTI_CHANNEL
	// setup multi-channel scanning mode
	int m, n = 0;

	for (int i = 0; i < 2 + CH_N; i++) {
	    if (i == 1) i = 3; // skip ADC1_CHANNEL_1 and 2

	    m = n ^ 1; // invert LSb to flip i2s adc data interleaving

	    SYSCON.saradc_sar1_patt_tab[m / 4] &= ~(0xff << (3 - (m % 4)) * 8);
	    SYSCON.saradc_sar1_patt_tab[m / 4] |= ((i << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_6) << (3 - (m % 4)) * 8;
	    n++;

	    ESP_LOGI(TAG, "SYSCON.saradc_sar1_patt_tab[%d] = %x", m / 4, SYSCON.saradc_sar1_patt_tab[m / 4]);
	}

    	SYSCON.saradc_ctrl.sar1_patt_len = n - 1; // set pattern length
	ESP_LOGI(TAG, "SYSCON.saradc_ctrl.sar1_patt_len = %x", n - 1);

	// adc1 controlled by DIG
        SENS.sar_read_ctrl.sar1_dig_force = true;
        SENS.sar_meas_start1.meas1_start_force = true;
	SENS.sar_meas_start1.sar1_en_pad_force = true;
	SENS.sar_touch_ctrl1.xpd_hall_force = true;
	SENS.sar_touch_ctrl1.hall_phase_force = true;
#endif

#ifdef I2S_BUG_WORKAROUND
    // check i2s adc working properly
    char *audio_buf = malloc(I2S_DMA_BUF_LEN);
    size_t bytes_read;
    if (audio_buf == NULL) {
	ESP_LOGE(TAG, "malloc fail");
	abort();
    }

    ESP_ERROR_CHECK(i2s_read(i2s_num, audio_buf, I2S_DMA_BUF_LEN, &bytes_read, portMAX_DELAY));
    int zero = 0;
    for (int i = 0; i < bytes_read; i++) zero |= audio_buf[i];
    if (zero == 0) {
	ESP_LOGE(TAG, "i2s adc not working, do restart");
	esp_restart();
    }
    free(audio_buf);
#endif

    gpio_set_direction(I2S_GPIO_PIN, GPIO_MODE_OUTPUT);
#if 1
    //gpio_reset_pin(GPIO_NUM_14);
    //gpio_reset_pin(GPIO_NUM_12);
    //gpio_reset_pin(GPIO_NUM_13);
    {
	gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask  = 1ULL << GPIO_NUM_14;
    io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_12;
    io_conf.pin_bit_mask |= 1ULL << GPIO_NUM_13;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

#if 0
    for (int i = 0;; i++) {
	gpio_set_level(GPIO_NUM_14, i & 1);
	gpio_set_level(GPIO_NUM_12, i & 1);
	gpio_set_level(GPIO_NUM_13, i & 1);

	vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
#endif

    }
#if 0
    gpio_reset_pin(GPIO_NUM_14);
    gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);

    gpio_reset_pin(GPIO_NUM_12);
    gpio_set_direction(GPIO_NUM_12, GPIO_MODE_OUTPUT);

    gpio_reset_pin(GPIO_NUM_13);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
#endif
#endif

    /* initialize sin table */
    const uint8_t sin_freq[2] = { SPACE_DIV, MARK_DIV }; // 0: mark, 1: space

#define SPACE_SAMPLES (SPACE_DIV * FACTOR) 

    for (int p0 = 0; p0 < SIN_PHASE; p0++) {
	for (int p1 = 0; p1 < SIN_PHASE; p1++) {
	    for (int f0 = 0; f0 < SIN_FREQ; f0++) {
		for (int f1 = 0; f1 < SIN_FREQ; f1++) {
    	    	    for (int i = 0; i < SIN_SAMPLES; i++) {
		        uint32_t val;

	       	        //val = (1.0 + cos((i * SIN_CYCLES / sin_freq[f0] - p0 * BAUD_DIV * FACTOR) * M_PI * 2.0 / SIN_PHASE_DIV)) / 2.0 * SIN_AMPLI + 0.5;
	       	        val = (1.0 + (1.0 / 2.0 ) * cos((i * SIN_CYCLES / sin_freq[f0] - p0 * PHASE_DIV * FACTOR) * M_PI * 2.0 / SIN_PHASE_DIV)) / 2.0 * SIN_AMPLI + 0.5;
	        	sin_table[p0][p1][f0][f1][i][0] = val << (I2S_BIT_PER_SAMPLE - SIN_AMPLI_BIT);

	       	        //val = (1.0 + cos((i * SIN_CYCLES / sin_freq[f1] - p1 * BAUD_DIV * FACTOR) * M_PI * 2.0 / SIN_PHASE_DIV)) / 2.0 * SIN_AMPLI + 0.5;
	       	        val = (1.0 + (1.0 / 2.0) * cos((i * SIN_CYCLES / sin_freq[f1] - p1 * PHASE_DIV * FACTOR) * M_PI * 2.0 / SIN_PHASE_DIV)) / 2.0 * SIN_AMPLI + 0.5;
	        	sin_table[p0][p1][f0][f1][i][1] = val + (1 << (I2S_BIT_PER_SAMPLE - SIN_AMPLI_BIT));
		    }
		}
	    }
	}
    }
    ESP_LOGI(TAG, "sizeof sin_table[0][0][0][0]: %d", sizeof(sin_table[0][0][0][0]));

#if 0
    for (int p = 0; p < SIN_PHASE; p++) {
	for (int s = 0; s < SIN_SAMPLES; s++) {
	
	    printf("%d, %d, %d, %d\n",
	       	sin_table[p][0][0][0][s][0],
	       	sin_table[0][p][0][0][s][1],
	       	sin_table[p][0][1][0][s][0],
	       	sin_table[0][p][0][1][s][1]
		);
	}
    }
#endif

    i2s_event_ptt = xEventGroupCreate();
    if (i2s_event_ptt == NULL) {
	ESP_LOGE(TAG, "xEventGroupCreate() fail");
	abort();
    }
}

    static void i2s_rx_reader(void);
    static void i2s_tx_writer(void);

static uint16_t i2s_buff[I2S_DMA_BUF_LEN * 2]; // *2 for stereo
static const int i2s_buff_len = sizeof(i2s_buff);

/*
 * I2S read task
 */
uint32_t i2s_period_max = 0;
uint32_t i2s_period_max_tx = 0;
static void i2s_task(void*arg)
{
    //RingbufHandle_t ringbuf = (RingbufHandle_t)arg;
    i2s_event_t event;
    //uint32_t period_max = 0;
    uint32_t prev_time = 0;
    uint32_t t0;
    uint32_t t1;
    uint32_t prev_time_tx = 0;
    uint32_t t0_tx;
    uint32_t t1_tx;
    UBaseType_t ms = 0;

    // cleanup queue
    //ESP_ERROR_CHECK(i2s_stop(I2S_NUM));
    xQueueReset(i2s_queue);
    //ESP_ERROR_CHECK(i2s_start(I2S_NUM));

    while (1) {
	if (xQueueReceive(i2s_queue, &event, portMAX_DELAY) != pdTRUE) {
	    ESP_LOGW(TAG, "xQueueReceive fail");
	    continue;
	}

	ms = uxQueueMessagesWaiting(i2s_queue);
	if (ms > 0) {
	    static uint8_t done = 0;
	    if (++done % 8 == 1) {
		ESP_LOGW(TAG, "uxQueueMessagesWaiting() = %d", ms);

		i2s_event_t ev;
		if (xQueuePeek(i2s_queue, &ev, 0) == pdTRUE) {
		    ESP_LOGW(TAG, "event.type = %d, waiting event.type = %d", event.type, ev.type);
		}
	    }
	}

	switch (event.type) {
	    case I2S_EVENT_DMA_ERROR:
		ESP_LOGW(TAG, "I2S_EVENT_DMA_ERROR");
		break;
	    case I2S_EVENT_RX_DONE:
		//ESP_LOGI(TAG, "I2S_EVENT_RX_DONE");

		t0 = esp_timer_get_time();
		if (prev_time != 0) {
		    t1 = t0 - prev_time;
		    if (t1 > i2s_period_max) {
		        i2s_period_max = t1;
		    }
		}
		prev_time = t0;

		i2s_rx_reader();

	        break;
	    case I2S_EVENT_TX_DONE:
		//ESP_LOGI(TAG, "I2S_EVENT_TX_DONE");

		t0_tx = esp_timer_get_time();
		if (prev_time_tx != 0) {
		    t1_tx = t0_tx - prev_time_tx;
		    if (t1_tx > i2s_period_max_tx) {
		        i2s_period_max_tx = t1_tx;
		    }
		}
		prev_time_tx = t0_tx;

		i2s_tx_writer();

	        break;
	    default: // suppress error for I2S_EVENT_MAX
		ESP_LOGW(TAG, "unknown event: type = %d, size = %d", event.type, event.size);
		break;
	}
    }
}

static void i2s_rx_reader(void)
{
    size_t bytes_read;
    static size_t bytes_read_prev = 0;

    gpio_set_level(I2S_GPIO_PIN, 1);
    //read data from I2S bus, in this case, from ADC.
    ESP_ERROR_CHECK(i2s_read(I2S_NUM, (void*) i2s_buff, i2s_buff_len, &bytes_read, 0));
    gpio_set_level(I2S_GPIO_PIN, 0);

    if (bytes_read != i2s_buff_len) {
	ESP_LOGW(TAG, "i2s_read(): bytes_read = %d", bytes_read);
    }

#if 0
    if (bytes_read != bytes_read_prev) {
	ESP_LOGI(TAG, "bytes_read = %d", bytes_read);
	bytes_read_prev = bytes_read;
    }
#endif

    if (bytes_read > 0) {
	// delete left channel
	int samples = bytes_read / (I2S_BITS_PER_SAMPLE / 8);

	for (int i = 0; i < samples / 2; i++) { // use only right channel
	    i2s_buff[i] = i2s_buff[i * 2];
	}

	if (xRingbufferSend(rxd_ringbuf, i2s_buff, samples , 0) != pdTRUE) {
	    ESP_LOGI(TAG, "xRingbufferSend fail, data size = %d byte", samples);
	}
    }
}


int q0_empty = 0;
int q1_empty = 0;

void i2s_tx_writer(void)
{
    static uint8_t ptt0 = 0, ptt1 = 0;
    static uint32_t data0 = 0, data1 = 0;
    static uint8_t f0 = 0, f1 = 0;
    static uint8_t p0 = 0, p1 = 0;
    size_t bytes_written;

#define PTT_GPIO GPIO_NUM_13

    // PTT off
    if ((ptt0 > 0) && (--ptt0 == 0)) {
	gpio_set_level(PTT_GPIO, 0);
	xEventGroupSetBits(i2s_event_ptt, 1 << 0);
    }
    if ((ptt1 > 0) && (--ptt1 == 0)) {
	//gpio_set_level(PTT_GPIO, 0);
	xEventGroupSetBits(i2s_event_ptt, 1 << 1);
    }

    for (int i = 0; i < I2S_DMA_BUF_BITS; i++) {

    if (data0 <= 1) {
        if (xQueueReceive(txd0_queue, &data0, 0) == pdTRUE) {
		//printf("data0 = %d\n", data0);
	} else q0_empty++;
    }
    if (data1 <= 1) {
        if (xQueueReceive(txd1_queue, &data1, 0) == pdTRUE) {
		//printf("data1 = %d\n", data1);
	} else q1_empty++;
    }

    if (data0 > 1) { // send 1 during no data period
	f0 = data0 & 1;
	data0 >>= 1;
	ptt0 = 4;
    } //else if (i == 0) { f0 = 0; p0 = 0; }
    if (data1 > 1) {
	f1 = data1 & 1;
	data1 >>= 1;
	ptt1 = 4;
    }

#define I2S_BUF_ONE_BIT_LEN (I2S_DMA_BUF_LEN / I2S_DMA_BUF_BITS)

    //ESP_ERROR_CHECK(i2s_write(I2S_NUM, (void*) sin_table[p0][p1][f0][f1], sizeof(sin_table[0][0][0][0]), &bytes_written, 0));
    memcpy(&i2s_buff[i * I2S_BUF_ONE_BIT_LEN * CH_N], sin_table[p0][p1][f0][f1], I2S_BUF_ONE_BIT_LEN * CH_N * sizeof(uint16_t));

    if (!f0) p0 = (p0 + 1) % SIN_PHASE;
    if (!f1) p1 = (p1 + 1) % SIN_PHASE;

    }

    gpio_set_level(GPIO_NUM_14, 1);
    ESP_ERROR_CHECK(i2s_write(I2S_NUM, (void*) i2s_buff, i2s_buff_len, &bytes_written, 0));
    gpio_set_level(GPIO_NUM_14, 0);

    if (bytes_written != i2s_buff_len) {
	ESP_LOGW(TAG, "i2s_write(): bytes_written = %d", bytes_written);
    }
}

/*
 * Initialize ADC read task
 */
void i2s_init(RingbufHandle_t ringbuf, QueueHandle_t queue0, QueueHandle_t queue1)
{
    BaseType_t res;

    rxd_ringbuf = ringbuf;
    txd0_queue = queue0;
    txd1_queue = queue1;

    ESP_LOGI(TAG, "queue0 = %p, queue1 = %p", queue0, queue1);

    i2s_config();

    res = xTaskCreate(i2s_task, "i2s_task", 1024 * 2, ringbuf, configMAX_PRIORITIES-1, NULL);
    if (res != pdTRUE) {
	ESP_LOGE(TAG, "xTaskCreate fail");
	vTaskDelete(NULL);
    }

    ESP_LOGI(TAG, "task priority: %d", configMAX_PRIORITIES - 1);
}
