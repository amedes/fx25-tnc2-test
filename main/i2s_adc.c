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

//#define I2S_DAC_OUTPUT 1
#define MULTI_CHANNEL 1
//#define ENABLE_ADC2 1
#define ENABLE_STEREO 1

//i2s number
#define I2S_NUM           (0)

#define CH_N 2 // number of adc channels

//i2s sample rate
#ifdef ENABLE_ADC2
#define I2S_SAMPLE_RATE   (6726 * 6 * 2)
#else
#define I2S_SAMPLE_RATE   (13200 * CH_N * 1)
#endif

#define I2S_DMA_BUF_COUNT	2
//#define I2S_DMA_BUF_LEN		1024
#define I2S_DMA_BUF_LEN		(11 * 12 * CH_N) // 1bit / 833us * 8

//I2S built-in ADC unit
#define I2S_ADC_UNIT              ADC_UNIT_1
//I2S built-in ADC channel
#define I2S_ADC_CHANNEL           ADC1_CHANNEL_0 // GPIO39

#define I2S_GPIO_PIN	GPIO_NUM_27
static xQueueHandle i2s_queue;

/**
 * @brief I2S ADC/DAC mode init.
 */
static void i2s_config(void)
{
	i2s_port_t i2s_num = I2S_NUM;
	i2s_config_t i2s_config = {
            .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN
#ifdef I2S_DAC_OUTPUT
		    | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN
#endif
		    ,
	    .sample_rate =  I2S_SAMPLE_RATE,
	    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
	    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
#ifdef ENABLE_STEREO
	    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
#else
	    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
#endif
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

#ifdef I2S_DAC_OUTPUT
	//init DAC pad
	ESP_ERROR_CHECK(i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN)); // DAC1 on GPIO25
	//ESP_ERROR_CHECK(i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN)); // DAC1 on GPIO25
#endif

	//init ADC pad
	ESP_ERROR_CHECK(i2s_set_adc_mode(I2S_ADC_UNIT, I2S_ADC_CHANNEL));
	// enable adc2
	//ESP_ERROR_CHECK(i2s_set_adc_mode(ADC_UNIT_2, ADC2_CHANNEL_0));
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
#ifdef ENABLE_ADC2
	int j = 4;
#endif
	for (int i = 0; i < CH_N; i++) {
	    if (i == 1) i = 3; // skip ADC1_CHANNEL_1 and 2
#ifdef ENABLE_ADC2
	    m = n;
#else
	    m = n ^ 1; // invert LSb to flip i2s adc data interleaving
#endif
	    SYSCON.saradc_sar1_patt_tab[m / 4] &= ~(0xff << (3 - (m % 4)) * 8);
	    SYSCON.saradc_sar1_patt_tab[m / 4] |= ((i << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_6) << (3 - (m % 4)) * 8;
#ifdef ENABLE_ADC2
	    SYSCON.saradc_sar2_patt_tab[m / 4] &= ~(0xff << (3 - (m % 4)) * 8);
	    SYSCON.saradc_sar2_patt_tab[m / 4] |= ((j << 4) | (ADC_WIDTH_BIT_12 << 2) | ADC_ATTEN_DB_6) << (3 - (m % 4)) * 8;
	    j++;
#endif
	    n++;
	}
    	SYSCON.saradc_ctrl.sar1_patt_len = n - 1; // set pattern length
#ifdef ENABLE_ADC2
    	SYSCON.saradc_ctrl.sar2_patt_len = n - 1; // set pattern length
    	SYSCON.saradc_ctrl.work_mode = 2; // double mode
    	SYSCON.saradc_ctrl.sar_sel = 1; // select adc2
	SYSCON.saradc_ctrl.data_sar_sel = ADC_ENCODE_12BIT;

	// adc2 controlled by DIG
                SENS.sar_meas_start2.meas2_start_force = true;
                SENS.sar_meas_start2.sar2_en_pad_force = true;
                SENS.sar_read_ctrl2.sar2_dig_force = true;
                SENS.sar_read_ctrl2.sar2_pwdet_force = false;
                SYSCON.saradc_ctrl.sar2_mux = true;

    	SYSCON.saradc_ctrl.sar1_patt_p_clear = 1; // pattern pointer reset
    	SYSCON.saradc_ctrl.sar2_patt_p_clear = 1; // pattern pointer reset
    	SYSCON.saradc_ctrl.sar1_patt_p_clear = 0; 
    	SYSCON.saradc_ctrl.sar2_patt_p_clear = 0;
#endif
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
}

/*
 * I2S read task
 */
static void i2s_task(void*arg)
{
    size_t bytes_read;
#ifdef ENABLE_STEREO
    static uint16_t i2s_read_buff[I2S_DMA_BUF_LEN*2];
#else
    static uint16_t i2s_read_buff[I2S_DMA_BUF_LEN];
#endif
    int i2s_read_len = sizeof(i2s_read_buff);
    RingbufHandle_t ringbuf = (RingbufHandle_t)arg;
    i2s_event_t event;

    while (1) {
	gpio_set_level(I2S_GPIO_PIN, 0);
	if (xQueueReceive(i2s_queue, &event, portMAX_DELAY) != pdTRUE) {
	    ESP_LOGW(TAG, "xQueueReceive fail");
	    continue;
	}
	gpio_set_level(I2S_GPIO_PIN, 1);

	switch (event.type) {
	    case I2S_EVENT_DMA_ERROR:
		ESP_LOGW(TAG, "I2S_EVENT_DMA_ERROR");
		continue;
	    case I2S_EVENT_RX_DONE:
		//ESP_LOGI(TAG, "I2S_EVENT_RX_DONE");
	        break;
	    case I2S_EVENT_TX_DONE:
		//ESP_LOGI(TAG, "I2S_EVENT_TX_DONE");
	        continue;
	    default:
	        continue;
	}

        //read data from I2S bus, in this case, from ADC.
        ESP_ERROR_CHECK(i2s_read(I2S_NUM, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY));

	if (bytes_read > 0) {
#ifdef ENABLE_STEREO
	    // delete left channel
	    bytes_read /= 2;
	    for (int i = 0; i < bytes_read/2; i++) {
		i2s_read_buff[i] = i2s_read_buff[i * 2];
	    }
#endif
	    if (xRingbufferSend(ringbuf, i2s_read_buff, bytes_read , 0) != pdTRUE) {
		ESP_LOGI(TAG, "xRingbufferSend fail");
	    }

#ifdef ENABLE_STEREO
	    bytes_read *= 2;
#endif
#ifdef I2S_DAC_OUTPUT
    	    size_t bytes_written;
	    for (int j = 0; j < bytes_read/2; j++) {
		i2s_read_buff[j] <<= 4;
	    }
            ESP_ERROR_CHECK(i2s_write(I2S_NUM, (void*) i2s_read_buff, bytes_read, &bytes_written, 0));
#endif
	}
    }
}

/*
 * Initialize ADC read task
 */
void i2s_init(RingbufHandle_t ringbuf)
{
    BaseType_t res;

    i2s_config();

    res = xTaskCreate(i2s_task, "i2s_task", 1024 * 2, ringbuf, 15, NULL);
    if (res != pdTRUE) {
	ESP_LOGE(TAG, "xTaskCreate fail");
	vTaskDelete(NULL);
    }
}
