#include <stdlib.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "driver/gpio.h"
#include "soc/sens_periph.h"

#include "driver/dac.h"
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "driver/rmt.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/timer.h"
#include "esp_intr_alloc.h"

#include "cw.h"
//#include "dac.h"

#define TAG "cw"

#define CW_CHANNEL DAC_CHANNEL_2 // GPIO26

#define CW_LEDC_MODE LEDC_HIGH_SPEED_MODE
#define CW_LEDC_TIMER	LEDC_TIMER_1
#define CW_LEDC_CHANNEL LEDC_CHANNEL_1
#define CW_LEDC_GPIO 13

#define BAUD_RATE 1200

#define CW_TIMER_GROUP TIMER_GROUP_0
#define CW_TIMER TIMER_0
#define CW_TIMER_CLK (80 * 1000 * 1000) // 80MHz
#define CW_TIMER_DIV 2 // 80MHz / 2 = 25ns tick

#define CW_TIMER_ALARM (CW_TIMER_CLK / CW_TIMER_DIV / BAUD_RATE)

#define CW_GPIO_ENABLE 1

#ifdef CW_GPIO_ENABLE
#define CW_GPIO_TXD GPIO_NUM_27
#endif

static QueueHandle_t cw_queue;
static TaskHandle_t cw_task;
//static rmt_item16_t *item16_ptr = NULL;

#define ESP_INTR_FLAG_DEFAULT (0)

void IRAM_ATTR cw_timer_isr(void *arg)
{
    QueueHandle_t queue = (QueueHandle_t)arg;
    uint8_t byte;

    if (xQueueReceiveFromISR(queue, &byte, 0) == pdTRUE) {
        cw_bell202(byte);

#if CW_GPIO_ENABLE
	gpio_set_level(CW_GPIO_TXD, byte);
#endif
    }

    // clear timer intr
    TIMERG0.int_clr_timers.t0 = 1;
    // set next alarm time
    //TIMERG0.hw_timer[CW_TIMER].alarm_low = 833; // 1/1200 us
    // enable alarm again, need clear timer intr previously
    TIMERG0.hw_timer[CW_TIMER].config.alarm_en = 1;

    //cw_bell202(0);
    //cw_bell202(rand() % 2);
    //cw_bell202(signal);
    //signal = !signal;

#if 0
    if (item16_ptr != NULL && item16_ptr->duration > 0) {
	cw_bell202(item16_ptr->level);

	// set next alarm time
	TIMERG0.hw_timer[CW_TIMER].alarm_low = item16_ptr->duration;
	item16_ptr++;

	// enable alarm again, need clear timer intr previously
        TIMERG0.hw_timer[CW_TIMER].config.alarm_en = 1;
    } else {
	cw_bell202(1); // set idle state

	vTaskNotifyGiveFromISR(cw_task, NULL);
    }
#endif
}

    static void cw_timer_start(void);

/*
 * Enable cosine waveform generator (CW)
 * on channel 1 / GPIO25 to provide sinusoidal signal
 * It can be used instead of a live signal for testing
 * of speed of logging to the host
 * sequentially with data retrieval from ADC
 */
void cw_init(QueueHandle_t queue)
{
    int fstep = FSTEP_MARK;

    ESP_LOGI(TAG, "Mark, fstep: %d, %d Hz", FSTEP_MARK, (FSTEP_MARK * DIG_CLK) >> 16);
    ESP_LOGI(TAG, "Space, fstep: %d, %d Hz", FSTEP_SPACE, (FSTEP_SPACE * DIG_CLK) >> 16);

    // Enable tone generator common to both DAC channels 1 and 2
    //SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL1_REG, SENS_SW_TONE_EN);
    SENS.sar_dac_ctrl1.sw_tone_en = 1;

    // Enable / connect tone tone generator on / to channel 1 and 2
    //SET_PERI_REG_MASK(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_CW_EN2_M);
    SENS.sar_dac_ctrl2.dac_cw_en1 = 1;
    SENS.sar_dac_ctrl2.dac_cw_en2 = 1;

    // Invert MSB, otherwise part of the waveform will be inverted
    //SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_INV1, 2, SENS_DAC_INV2_S);
    SENS.sar_dac_ctrl2.dac_inv1 = 2; // invert MSB
    SENS.sar_dac_ctrl2.dac_inv2 = 2; // invert MSB
    //SENS.sar_dac_ctrl2.dac_inv2 = 3; // invert all bits except MSB

    // Set the frequency of waveform on CW output
    //SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_FSTEP, fstep, SENS_SW_FSTEP_S);
    SENS.sar_dac_ctrl1.sw_fstep = fstep;

    // Set the scale of waveform to 0:1/1, 1:1/2, 2:1/4, 3:1/8
    //SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_SCALE1, 3, SENS_DAC_SCALE2_S);
    //SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_SCALE1, 0, SENS_DAC_SCALE2_S);
    SENS.sar_dac_ctrl2.dac_scale1 = 0; // 1/1
    SENS.sar_dac_ctrl2.dac_scale2 = 0; // 1/1
    
    // Set the DC offset of waveform to 0
    //SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL2_REG, SENS_DAC_DC1, 0, SENS_DAC_DC2_S);
    SENS.sar_dac_ctrl2.dac_dc1 = 0;
    SENS.sar_dac_ctrl2.dac_dc2 = 0;

#if 1
    // DAC1 & DAC2 do not use DMA (CW enable)
    SENS.sar_dac_ctrl1.dac_dig_force = 0;
#else  
    // DAC1 & DAC2 use DMA (CW disable)
    SENS.sar_dac_ctrl1.dac_dig_force = 1;
#endif

    dac_output_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_2);

    // using timer for 1200buad timing
    timer_config_t timer_conf = {
	.divider = CW_TIMER_DIV,
	.counter_dir = TIMER_COUNT_UP,
	.counter_en = false,
	.alarm_en = true,
	.intr_type = TIMER_INTR_LEVEL,
	.auto_reload = true,
    };
    timer_init(CW_TIMER_GROUP, CW_TIMER, &timer_conf);
    
    // get task
    cw_task = xTaskGetCurrentTaskHandle();

    // set isr
    timer_isr_register(CW_TIMER_GROUP, CW_TIMER, cw_timer_isr, queue, ESP_INTR_FLAG_LEVEL2, NULL);

    cw_timer_start();

#ifdef CW_GPIO_ENABLE
    gpio_config_t io_conf = {
	.intr_type = GPIO_PIN_INTR_DISABLE,
	.mode = GPIO_MODE_OUTPUT,
	.pin_bit_mask = (1ULL << CW_GPIO_TXD),
	.pull_down_en = 0,
	.pull_up_en = 0,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
#endif
}

static void cw_timer_start(void)
{
    // timer stop
    timer_pause(CW_TIMER_GROUP, CW_TIMER);
    timer_disable_intr(CW_TIMER_GROUP, CW_TIMER);

    // set timer value
    timer_set_counter_value(CW_TIMER_GROUP, CW_TIMER, 0ULL);

    // set alarm
    timer_set_alarm_value(CW_TIMER_GROUP, CW_TIMER, CW_TIMER_ALARM); // after 1/1200 s
    timer_enable_intr(CW_TIMER_GROUP, CW_TIMER);
    timer_set_alarm(CW_TIMER_GROUP, CW_TIMER, TIMER_ALARM_EN);

    // timer start
    timer_start(CW_TIMER_GROUP, CW_TIMER);

}

static void cw_timer_stop(void)
{
    // timer stop
    timer_pause(CW_TIMER_GROUP, CW_TIMER);
    timer_disable_intr(CW_TIMER_GROUP, CW_TIMER);
}

void cw_set_freq(int freq)
{
    int fstep = ((freq << CLK_SHIFT) + CLK_MUL/2) / CLK_MUL;

    // Set the frequency of waveform on CW output
    SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_FSTEP, fstep, SENS_SW_FSTEP_S);
}

// move to header file
#if 0
void cw_bell202(int txd)
{
    int fstep = (txd != 0) ? FSTEP_MARK : FSTEP_SPACE;

    // Set the frequency of waveform on CW output
    SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_FSTEP, fstep, SENS_SW_FSTEP_S);
}
#endif

void cw_enable(void)
{
    dac_output_enable(DAC_CHANNEL_1);
    dac_output_enable(DAC_CHANNEL_2);
}

void cw_disable(void)
{
    dac_output_disable(DAC_CHANNEL_1);
    dac_output_disable(DAC_CHANNEL_2);
}

#if 0
void cw_send_data(rmt_item32_t *item32, int item32_size)
{
    item16_ptr = (rmt_item16_t *)item32;
    cw_timer_start();

    // wait for sending packet
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    cw_timer_stop();
}
#endif
