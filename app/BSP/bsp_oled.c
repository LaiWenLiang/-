#include "bsp_oled.h"
#include "delay.h"

/* ---------------- 软件 I2C（GPIO 位操作，时序不敏感） ---------------- */
#define OLED_SCL(a)  do { if(a) GPIO_SetBits(GPIOE, GPIO_Pin_0); else GPIO_ResetBits(GPIOE, GPIO_Pin_0); } while (0)
#define OLED_SDA(a)  do { if(a) GPIO_SetBits(GPIOE, GPIO_Pin_1); else GPIO_ResetBits(GPIOE, GPIO_Pin_1); } while (0)

static void SoftI2C_Start(void)
{
    OLED_SDA(1); OLED_SCL(1);
    OLED_SDA(0); OLED_SCL(0);
}

static void SoftI2C_Stop(void)
{
    OLED_SDA(0); OLED_SCL(1);
    OLED_SDA(1);
}

static void SoftI2C_WriteByte(uint8_t dat)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        OLED_SDA(dat & 0x80);
        OLED_SCL(1);
        OLED_SCL(0);
        dat <<= 1;
    }
    OLED_SDA(1);   /* 释放 SDA 收 ACK（不校验） */
    OLED_SCL(1);
    OLED_SCL(0);
}

static void OLED_WriteCmd(uint8_t cmd)
{
    SoftI2C_Start();
    SoftI2C_WriteByte(0x78);
    SoftI2C_WriteByte(0x00);
    SoftI2C_WriteByte(cmd);
    SoftI2C_Stop();
}

static void OLED_WriteData(uint8_t dat)
{
    SoftI2C_Start();
    SoftI2C_WriteByte(0x78);
    SoftI2C_WriteByte(0x40);
    SoftI2C_WriteByte(dat);
    SoftI2C_Stop();
}

/* 6x8 ASCII 字库（仅数字、字母大写、少量符号，够用即可） */
static const uint8_t F6x8[][6] =
{
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* space */
    {0x00,0x00,0x00,0x2F,0x00,0x00}, /* ! */
    {0x00,0x00,0x07,0x00,0x07,0x00}, /* " */
    {0x00,0x14,0x7F,0x14,0x7F,0x14}, /* # */
    {0x00,0x24,0x2A,0x7F,0x2A,0x12}, /* $ */
    {0x00,0x62,0x64,0x08,0x13,0x23}, /* % */
    {0x00,0x36,0x49,0x55,0x22,0x50}, /* & */
    {0x00,0x00,0x05,0x03,0x00,0x00}, /* ' */
    {0x00,0x00,0x1C,0x22,0x41,0x00}, /* ( */
    {0x00,0x00,0x41,0x22,0x1C,0x00}, /* ) */
    {0x00,0x14,0x08,0x3E,0x08,0x14}, /* * */
    {0x00,0x08,0x08,0x3E,0x08,0x08}, /* + */
    {0x00,0x00,0x00,0xA0,0x60,0x00}, /* , */
    {0x00,0x08,0x08,0x08,0x08,0x08}, /* - */
    {0x00,0x00,0x60,0x60,0x00,0x00}, /* . */
    {0x00,0x20,0x10,0x08,0x04,0x02}, /* / */
    {0x00,0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x00,0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x00,0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x00,0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x00,0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x00,0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x00,0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x00,0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x00,0x06,0x49,0x49,0x29,0x1E}, /* 9 */
    {0x00,0x00,0x36,0x36,0x00,0x00}, /* : */
};

static void OLED_SetPos(uint8_t x, uint8_t page)
{
    OLED_WriteCmd(0xB0 + page);
    OLED_WriteCmd(((x + 2) & 0xF0) >> 4 | 0x10);
    OLED_WriteCmd((x + 2) & 0x0F);
}

void BSP_OLED_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    delay_ms(100);
    OLED_WriteCmd(0xAE); /* 关显示 */
    OLED_WriteCmd(0xD5); OLED_WriteCmd(0x80);
    OLED_WriteCmd(0xA8); OLED_WriteCmd(0x3F);
    OLED_WriteCmd(0xD3); OLED_WriteCmd(0x00);
    OLED_WriteCmd(0x40);
    OLED_WriteCmd(0x8D); OLED_WriteCmd(0x14);
    OLED_WriteCmd(0x20); OLED_WriteCmd(0x02);
    OLED_WriteCmd(0xA1);
    OLED_WriteCmd(0xC8);
    OLED_WriteCmd(0xDA); OLED_WriteCmd(0x12);
    OLED_WriteCmd(0x81); OLED_WriteCmd(0xEF);
    OLED_WriteCmd(0xD9); OLED_WriteCmd(0xF1);
    OLED_WriteCmd(0xDB); OLED_WriteCmd(0x40);
    OLED_WriteCmd(0xA4);
    OLED_WriteCmd(0xA6);
    BSP_OLED_Clear();
    OLED_WriteCmd(0xAF); /* 开显示 */
}

void BSP_OLED_Clear(void)
{
    uint8_t page, i;
    for (page = 0; page < 8; page++)
    {
        OLED_SetPos(0, page);
        for (i = 0; i < 128; i++)
        {
            OLED_WriteData(0);
        }
    }
}

/* 显示 ASCII 字符串（6x8 字体，支持 space~':'，超出范围显示空格） */
void BSP_OLED_ShowString(uint8_t x, uint8_t page, const char *str)
{
    uint8_t c, i;

    OLED_SetPos(x, page);
    while (*str)
    {
        c = (*str >= ' ' && *str <= ':') ? (*str - ' ') : 0;
        for (i = 0; i < 6; i++)
        {
            OLED_WriteData(F6x8[c][i]);
        }
        str++;
    }
}
