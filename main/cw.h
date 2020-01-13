#ifndef __CW_H__
#define __CW_H__

#include "soc/sens_periph.h"
#include "driver/rmt.h"

#define DIG_CLK 8665000 // 8M clk
#define CLK_MUL 15625 // 8000000/65536 = 15625/128
#define CLK_SHIFT 7 // 128 = 2^7
#define CW_FREQ_SHIFT 16 // 65536
//#define FSTEP_VAL(HZ)  (((HZ << CLK_SHIFT) + CLK_MUL/2) / CLK_MUL)
#define FSTEP_VAL(HZ)  (((HZ << CW_FREQ_SHIFT) + DIG_CLK/2) / DIG_CLK)
#if 1
#define FSTEP_MARK  FSTEP_VAL(1200)
#define FSTEP_SPACE FSTEP_VAL(2200)
#else
#define FSTEP_MARK  9
#define FSTEP_SPACE 17
#endif

inline void cw_bell202(int txd)
{
    int fstep = (txd != 0) ? FSTEP_MARK : FSTEP_SPACE;
    //SET_PERI_REG_BITS(SENS_SAR_DAC_CTRL1_REG, SENS_SW_FSTEP, fstep, SENS_SW_FSTEP_S);
    SENS.sar_dac_ctrl1.sw_fstep = fstep;
}

void cw_init(QueueHandle_t queue);
void cw_set_freq(int freq);
void cw_enable(void);
void cw_disable(void);
void cw_send_data(rmt_item32_t *item32, int item32_size);

#endif /* __CW_H__ */
