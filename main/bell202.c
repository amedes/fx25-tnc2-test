/*
 * bell202.c
 *
 * decode Bell 202 FSK 1200 baud signal
 */
#include <stdio.h>
#include <stdint.h>

#include "tcb.h"

// coefficient for low pass filter
static const int an[FIR_LPF_N] = {
#if FIR_LPF_N == 11
	    // LPF
	    // length: 11
	    // window: rectangular
	    // fc: 1200Hz / 6726Hz
	    // multiplier: 2^16
	-2618,-5080,-1527,8168,18786,23385,18786,8168,-1527,-5080,-2618
#elif FIR_LPF_N == 15
	    // LPF
	    // length: 15
	    // window: rectangular
	    // fc: 1200Hz / 13200Hz
	    // multiplier: 2^16
	-2252,-980,1175,3941,6883,9488,11278,11916,11278,9488,6883,3941,1175,-980,-2252
#else
#error
#endif
};

int bell202_decode(tcb_t *tp, int adc)
{
    //static int delayed[DELAYED_N];
    //static int delay_idx = 0;
    //static int x[FIR_LPF_N];
    //static int x_idx = 0;
    int m;
    int sum;
    int i;

    //if (x_idx == 0) printf("%d ", adc);

    m = adc * tp->delayed[tp->delay_idx];
    tp->delayed[tp->delay_idx] = adc;
    tp->delay_idx = (tp->delay_idx + 1) % DELAYED_N;

    tp->x[tp->x_idx] = m >> 12;
    sum = 0;
    for (i = 0; i < FIR_LPF_N; i++) {
	sum += an[i] * tp->x[(tp->x_idx + i) % FIR_LPF_N];
#if 0
	if (sum > (1 << 30)) printf("%d,", sum);
	else
	if (sum < -(1 << 30)) printf("%d,", sum);
#endif
    }
    tp->x_idx += FIR_LPF_N - 1; // idx = idx - 1
    tp->x_idx %= FIR_LPF_N;

    //if (x_idx == 0) printf("%g ", sum);

    return sum;
}

void bell202_init(void)
{
    // do nothing
}
