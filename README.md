# stm32f4_LCD
Simple LCD library. 

This library works with stm32f4xx devices and others Cortex-M4 ST microcontrollers.
No SPL and HAL makes lib light.

v1.0 supports only 4-bit mode.
You can configurate Pins and Ports in MELT_LCD_Conf.h file.
Also can also confirurate number of ROWS and COLUMNS.
Auto string cutoff. 

This lib also contains delay. Delay is made on DWT timer. You can use your own, just include your lib with function delay_us(uint32_t us).

Hello world:

int main(void)
{
    SystemInit();
    delay_Init();
    LCD_TypeDef LCD;
    LCD.EMSR = (LCD_EMSR_ID);
    LCD.DOCR = (LCD_DOCR_D);
    LCD.CDSR = (LCD_CDSR_RL);
    LCD.FSR  = (LCD_FSR_P);
    meltLcd_Init(&LCD); 
    meltLcd_Puts(0, 0, "Hello world!");
    while(1)
    {
        
    }
    return 0;
}

Creating custom character:

const uint8_t customChar[] = {0x0A, 0x15, 0x0A, 0x04, 0x04, 0x0A, 0x15, 0x0A};
int main(void)
{
    SystemInit();
    delay_Init();
    LCD_TypeDef LCD;
    LCD.EMSR = (LCD_EMSR_ID);
    LCD.DOCR = (LCD_DOCR_D);
    LCD.CDSR = (LCD_CDSR_RL);
    LCD.FSR  = (LCD_FSR_P);
    meltLcd_Init(&LCD); 
    meltLcd_CreateCharacter(1, (uint8_t*)customChar);
    meltLcd_SetCursor(0,0);
    meltLcd_Putchar(1);
    while(1)
    {
        
    }
    return 0;
}

