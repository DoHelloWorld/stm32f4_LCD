#ifndef MELT_LCD_H_
#define MELT_LCD_H_

#include "stm32f4xx.h"
#include "MELT_LCD_Conf.h"

#include "stdio.h"
#include "stdarg.h"
#include "string.h"
#include "Delay.h"

typedef struct
{
   uint8_t EMSR;
   uint8_t DOCR;
   uint8_t CDSR;
   uint8_t FSR;
}LCD_TypeDef;

#define LCD_EMSR_SH            0x01
#define LCD_EMSR_ID            0x02

#define LCD_DOCR_B             0x01
#define LCD_DOCR_C             0x02
#define LCD_DOCR_D             0x04

#define LCD_CDSR_RL            0x04
#define LCD_CDSR_SC            0x08

#define LCD_FSR_P              0x02
#define LCD_FSR_DL             0x10

void meltLcd_Init(LCD_TypeDef * lcd);
void meltLcd_Clear(void);
void meltLcd_ReturnHome(void);
uint8_t meltLcd_SetCursor(uint8_t row, uint8_t column);
void meltLcd_Putchar(const char ch);
uint8_t meltLcd_Puts(uint8_t row, uint8_t column, const char * str);
uint8_t meltLcd_Printf(uint8_t row, uint8_t column, const char * str, ...);
void meltLcd_CreateCharacter(uint8_t address, const uint8_t * buffer);
#endif
