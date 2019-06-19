#include "MELT_LCD.h"


#define MELT_LCD_SEND_CMD   0
#define MELT_LCD_SEND_DATA  1

static void meltLcd_RCC(GPIO_TypeDef * port);
static void meltLcd_Pulse(void);
static void meltLcd_SendHalfByte(uint8_t halfByte);
static void meltLcd_SendByte(uint8_t byte, uint8_t mode);
static void meltLcd_Setup(LCD_TypeDef * lcd);

static void meltLcd_EMS(uint8_t ems);
static void meltLcd_DOC(uint8_t doc);
static void meltLcd_CDS(uint8_t cds);
static void meltLcd_FS(uint8_t fs);


static void meltLcd_RCC(GPIO_TypeDef * port)
{
    if(port == GPIOA)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    else if(port == GPIOB)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    else if(port == GPIOC)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;   
    else if(port == GPIOD)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;   
    else if(port == GPIOE)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;   
    else if(port == GPIOF)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;   
    else if(port == GPIOG)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;   
    else if(port == GPIOH)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
    else if(port == GPIOI)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;       
    else if(port == GPIOJ)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOJEN; 
    else if(port == GPIOK)
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOKEN; 
}

static void meltLcd_Pulse(void)
{
    MELT_LCD_E_PORT->BSRRL = 1 << MELT_LCD_E_PIN;
    delay_us(2);
    MELT_LCD_E_PORT->BSRRH = 1 << MELT_LCD_E_PIN;
}

static void meltLcd_SendHalfByte(uint8_t halfByte)
{
    if((halfByte >> 3) & 0x01)
        MELT_LCD_DB7_PORT->BSRRL = 1 << MELT_LCD_DB7_PIN;
    else
        MELT_LCD_DB7_PORT->BSRRH = 1 << MELT_LCD_DB7_PIN;        
    if((halfByte >> 2) & 0x01)
        MELT_LCD_DB6_PORT->BSRRL = 1 << MELT_LCD_DB6_PIN;
    else
        MELT_LCD_DB6_PORT->BSRRH = 1 << MELT_LCD_DB6_PIN;
    if((halfByte >> 1) & 0x01)
        MELT_LCD_DB5_PORT->BSRRL = 1 << MELT_LCD_DB5_PIN;
    else
        MELT_LCD_DB5_PORT->BSRRH = 1 << MELT_LCD_DB5_PIN;
    if(halfByte & 0x01)
        MELT_LCD_DB4_PORT->BSRRL = 1 << MELT_LCD_DB4_PIN;
    else
        MELT_LCD_DB4_PORT->BSRRH = 1 << MELT_LCD_DB4_PIN;
    meltLcd_Pulse();
}

static void meltLcd_SendByte(uint8_t byte, uint8_t mode)
{
    if(mode == MELT_LCD_SEND_DATA)
        MELT_LCD_A_PORT->BSRRL = 1 << MELT_LCD_A_PIN;
    meltLcd_SendHalfByte(byte >> 4);
    meltLcd_SendHalfByte(byte & 0x0F); 
    delay_us(40);
    MELT_LCD_A_PORT->BSRRH = 1 << MELT_LCD_A_PIN;    
}

static void meltLcd_Setup(LCD_TypeDef * lcd)
{
    meltLcd_SendHalfByte(0x03);
    delay_us(40);
    meltLcd_SendHalfByte(0x03);
    delay_us(40);
    meltLcd_SendHalfByte(0x03);
    delay_us(40);   
    meltLcd_SendHalfByte(0x02);
    delay_us(40);
    meltLcd_FS(lcd->FSR);   
    meltLcd_Clear();    
    meltLcd_EMS(lcd->EMSR);
    meltLcd_CDS(lcd->CDSR);
    meltLcd_DOC(lcd->DOCR);
}

static void meltLcd_EMS(uint8_t ems)
{
    ems &= 0x07;
    ems |= 0x04;
    meltLcd_SendByte(ems, MELT_LCD_SEND_CMD);
    delay_us(40);    
}

static void meltLcd_DOC(uint8_t doc)
{
    doc &= 0x0F;
    doc |= 0x08;
    meltLcd_SendByte(doc, MELT_LCD_SEND_CMD);
    delay_us(40);    
}

static void meltLcd_CDS(uint8_t cds)
{
    cds &= 0x1F;
    cds |= 10;
    meltLcd_SendByte(cds, MELT_LCD_SEND_CMD);
    delay_us(40);    
}

static void meltLcd_FS(uint8_t fs)
{
    fs &= 0x3F;
    fs |= 0x28;
    meltLcd_SendByte(fs, MELT_LCD_SEND_CMD);
    delay_us(40);
}

void meltLcd_Init(LCD_TypeDef * lcd)
{
    meltLcd_RCC(MELT_LCD_A_PORT);
    meltLcd_RCC(MELT_LCD_RW_PORT);
    meltLcd_RCC(MELT_LCD_E_PORT);
    meltLcd_RCC(MELT_LCD_DB4_PORT);
    meltLcd_RCC(MELT_LCD_DB5_PORT);
    meltLcd_RCC(MELT_LCD_DB6_PORT);
    meltLcd_RCC(MELT_LCD_DB7_PORT);
    
    MELT_LCD_A_PORT->MODER &= ~(0x3 << (MELT_LCD_A_PIN << 1));
    MELT_LCD_A_PORT->MODER |= (0x01 << (MELT_LCD_A_PIN << 1));
    MELT_LCD_A_PORT->OTYPER &= ~(0x1 << (MELT_LCD_A_PIN));
    MELT_LCD_A_PORT->OSPEEDR |= (0x3 << (MELT_LCD_A_PIN << 1));
    MELT_LCD_A_PORT->PUPDR &= ~(0x3 << (MELT_LCD_A_PIN << 1));    

    MELT_LCD_RW_PORT->MODER &= ~(0x3 << (MELT_LCD_RW_PIN << 1));
    MELT_LCD_RW_PORT->MODER |= (0x01 << (MELT_LCD_RW_PIN << 1));
    MELT_LCD_RW_PORT->OTYPER &= ~(0x1 << (MELT_LCD_RW_PIN));
    MELT_LCD_RW_PORT->OSPEEDR |= (0x3 << (MELT_LCD_RW_PIN << 1));
    MELT_LCD_RW_PORT->PUPDR &= ~(0x3 << (MELT_LCD_RW_PIN << 1));  
 
    MELT_LCD_E_PORT->MODER &= ~(0x3 << (MELT_LCD_E_PIN << 1));
    MELT_LCD_E_PORT->MODER |= (0x01 << (MELT_LCD_E_PIN << 1));
    MELT_LCD_E_PORT->OTYPER &= ~(0x1 << (MELT_LCD_E_PIN));
    MELT_LCD_E_PORT->OSPEEDR |= (0x3 << (MELT_LCD_E_PIN << 1));
    MELT_LCD_E_PORT->PUPDR &= ~(0x3 << (MELT_LCD_E_PIN << 1));  

    MELT_LCD_DB4_PORT->MODER &= ~(0x3 << (MELT_LCD_DB4_PIN << 1));
    MELT_LCD_DB4_PORT->MODER |= (0x01 << (MELT_LCD_DB4_PIN << 1));
    MELT_LCD_DB4_PORT->OTYPER &= ~(0x1 << (MELT_LCD_DB4_PIN));
    MELT_LCD_DB4_PORT->OSPEEDR |= (0x3 << (MELT_LCD_DB4_PIN << 1));
    MELT_LCD_DB4_PORT->PUPDR &= ~(0x3 << (MELT_LCD_DB4_PIN << 1));  
    
    MELT_LCD_DB5_PORT->MODER &= ~(0x3 << (MELT_LCD_DB5_PIN << 1));
    MELT_LCD_DB5_PORT->MODER |= (0x01 << (MELT_LCD_DB5_PIN << 1));
    MELT_LCD_DB5_PORT->OTYPER &= ~(0x1 << (MELT_LCD_DB5_PIN));
    MELT_LCD_DB5_PORT->OSPEEDR |= (0x3 << (MELT_LCD_DB5_PIN << 1));
    MELT_LCD_DB5_PORT->PUPDR &= ~(0x3 << (MELT_LCD_DB5_PIN << 1));
    
    MELT_LCD_DB6_PORT->MODER &= ~(0x3 << (MELT_LCD_DB6_PIN << 1));
    MELT_LCD_DB6_PORT->MODER |= (0x01 << (MELT_LCD_DB6_PIN << 1));
    MELT_LCD_DB6_PORT->OTYPER &= ~(0x1 << (MELT_LCD_DB6_PIN));
    MELT_LCD_DB6_PORT->OSPEEDR |= (0x3 << (MELT_LCD_DB6_PIN << 1));
    MELT_LCD_DB6_PORT->PUPDR &= ~(0x3 << (MELT_LCD_DB6_PIN << 1)); 
    
    MELT_LCD_DB7_PORT->MODER &= ~(0x3 << (MELT_LCD_DB7_PIN << 1));
    MELT_LCD_DB7_PORT->MODER |= (0x01 << (MELT_LCD_DB7_PIN << 1));
    MELT_LCD_DB7_PORT->OTYPER &= ~(0x1 << (MELT_LCD_DB7_PIN));
    MELT_LCD_DB7_PORT->OSPEEDR |= (0x3 << (MELT_LCD_DB7_PIN << 1));
    MELT_LCD_DB7_PORT->PUPDR &= ~(0x3 << (MELT_LCD_DB7_PIN << 1));    

    MELT_LCD_RW_PORT->BSRRH |= 1 << MELT_LCD_RW_PIN;
    delay_ms(20);
    meltLcd_Setup(lcd);
}

void meltLcd_Clear(void)
{
    meltLcd_SendByte(0x01, MELT_LCD_SEND_CMD);
    delay_us(1500);
}

void meltLcd_ReturnHome(void)
{
    meltLcd_SendByte(0x02, MELT_LCD_SEND_CMD);
    delay_us(40);
}

uint8_t meltLcd_SetCursor(uint8_t row, uint8_t column)
{
    if((row > MELT_LCD_ROW_NUM) || (column > MELT_LCD_COL_NUM))
        return 0;
    
    uint8_t address = 0;
    switch(row)
    {
        case 0:
        {
            address = 0;
            break;
        }
        case 1:
        {
            address = 0x40;
            break;
        }
        case 2:
        {
            address = 0x14;
            break;
        }
        case 3:
        {
            address = 0x54;
            break;
        }
    }
    address += column;
    meltLcd_SendByte(0x80 | address, MELT_LCD_SEND_CMD);
    return 1;
}

void meltLcd_Putchar(const char ch)
{
    meltLcd_SendByte(ch, MELT_LCD_SEND_DATA);
}

uint8_t meltLcd_Puts(uint8_t row, uint8_t column, const char * str)
{
    if(!meltLcd_SetCursor(row, column))
        return 0;
    uint16_t len = strlen(str);
    uint8_t chNum = 0;
    if(len > (MELT_LCD_COL_NUM - column))
        chNum = MELT_LCD_COL_NUM - column;
    else
        chNum = len;
    for(uint8_t i = 0 ; i < chNum; i++)
        meltLcd_Putchar(str[i]);  
    return 1;
}

uint8_t meltLcd_Printf(uint8_t row, uint8_t column, const char * str, ...)
{
    if(!meltLcd_SetCursor(row, column))
        return 0;
    char buffer[MELT_LCD_COL_NUM] = {0};
    va_list args;
    va_start(args, str);
    vsnprintf((char*)buffer, MELT_LCD_COL_NUM, (char*)str, args);
    va_end(args);
    uint8_t chNum = 0;
    uint8_t len = strlen(buffer);
    if(len > (MELT_LCD_COL_NUM - column))
        chNum = MELT_LCD_COL_NUM - column;
    else
        chNum = len;
    for(uint8_t i = 0 ; i < chNum; i++)
        meltLcd_Putchar(buffer[i]);
    return 1;
}

void meltLcd_CreateCharacter(uint8_t address, const uint8_t * buffer)
{
    meltLcd_SendByte(0x40 | (address << 3), MELT_LCD_SEND_CMD);
    delay_us(40);
    for(uint8_t i = 0; i < 8; i++)
    {
        meltLcd_SendByte(buffer[i], MELT_LCD_SEND_DATA);
        delay_us(40);
    }
}
