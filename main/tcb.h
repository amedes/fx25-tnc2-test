/*
 * tcb.h
 * 
 * TNC control block
 */
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#if 0
#define DELAYED_N 3
#define FIR_LPF_N 11
#else
#define DELAYED_N 6
#define FIR_LPF_N 15
#endif

enum STATE {
	FLAG = 0,
	DATA
};

#define AX25_FLAG 0x7e
#define DATA_LEN (1024 + 2) // DATA + FCS

#define TCB_QUEUE_LENGTH (1024 * 2)
#define TCB_QUEUE_ITEM_SIZE sizeof(uint16_t)

typedef struct TNC_CNTRL_BLOCK {
    QueueHandle_t queue;
    TaskHandle_t task;
    
    uint8_t port; // port NO. 0 - 5
    uint16_t pkts;

    uint8_t state;
    uint8_t flag;
    uint8_t data[DATA_LEN];
    int data_cnt;
    uint16_t data_byte;
    uint8_t data_bit_cnt;

    int pval;
    int edge;

//void demodulator(uint16_t adc)
    int avg;
    int cdt; // carrier detect

    // bell202()
    int delayed[DELAYED_N];
    int delay_idx;
    int x[FIR_LPF_N];
    int x_idx;
} tcb_t;
