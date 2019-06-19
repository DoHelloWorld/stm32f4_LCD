#include "Delay.h"

static uint32_t delay_delta(uint32_t t0, uint32_t t1);

static uint32_t delay_delta(uint32_t t0, uint32_t t1)
{
    return (t1 - t0);
}

void delay_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us)
{
    uint32_t t0 = DWT->CYCCNT; 
    uint32_t us_count_tic = us * (SystemCoreClock / 1000000);
    while(delay_delta(t0, DWT->CYCCNT) < us_count_tic);    
}

void delay_ms(uint32_t ms)
{
    volatile uint32_t us_count_tic = ms * (SystemCoreClock / 1000);
    volatile uint32_t t0 = DWT->CYCCNT;
    while(delay_delta(t0, DWT->CYCCNT) < us_count_tic);
}
