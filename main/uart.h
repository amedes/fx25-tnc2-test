#include <freertos/ringbuf.h>

extern RingbufHandle_t uart_rb;

void uart_init(void);
int uart_add_ringbuf(RingbufHandle_t rb);
void uart_delete_ringbuf(RingbufHandle_t rb);
