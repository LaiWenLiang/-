#include "bsp_w24c02.h"

/* W24C02 器件地址（A0/A1/A2 接地 -> 0x50，左移一位后写=0xA0 读=0xA1） */
#define W24C02_ADDR_WRITE  0xA0
#define W24C02_ADDR_READ   0xA1

#define SCL_HIGH()  GPIO_SetBits(GPIOF, GPIO_Pin_6)
#define SCL_LOW()   GPIO_ResetBits(GPIOF, GPIO_Pin_6)
#define SDA_HIGH()  GPIO_SetBits(GPIOF, GPIO_Pin_7)
#define SDA_LOW()   GPIO_ResetBits(GPIOF, GPIO_Pin_7)
#define SDA_READ()  GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_7)

/* 粗略微秒级延时（168MHz 下的软件循环，软件 I2C 用，精度要求不高） */
static void I2C_Delay(void)
{
    uint32_t i = 200;
    while (i--)
    {
    }
}

/* SDA 切为输出（开漏模式下直接写高低即可，这里用开漏模式所以不用切方向） */

static void I2C_Start(void)
{
    SDA_HIGH();
    SCL_HIGH();
    I2C_Delay();
    SDA_LOW();
    I2C_Delay();
    SCL_LOW();
}

static void I2C_Stop(void)
{
    SDA_LOW();
    SCL_HIGH();
    I2C_Delay();
    SDA_HIGH();
    I2C_Delay();
}

/* 发送一个字节，返回 0=收到应答ACK，1=无应答 */
static uint8_t I2C_SendByte(uint8_t data)
{
    uint8_t i;
    uint8_t ack;

    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
        {
            SDA_HIGH();
        }
        else
        {
            SDA_LOW();
        }
        data <<= 1;
        SCL_HIGH();
        I2C_Delay();
        SCL_LOW();
    }

    /* 第 9 个时钟：释放 SDA，读应答 */
    SDA_HIGH();
    I2C_Delay();
    SCL_HIGH();
    ack = SDA_READ();
    SCL_LOW();
    return ack;
}

/* 接收一个字节，ack=0 继续读，ack=1 读完了 */
static uint8_t I2C_RecvByte(uint8_t ack)
{
    uint8_t i;
    uint8_t data = 0;

    SDA_HIGH();   /* 释放 SDA */
    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        SCL_HIGH();
        I2C_Delay();
        if (SDA_READ())
        {
            data |= 1;
        }
        SCL_LOW();
    }

    /* 主机给应答 */
    if (ack)
    {
        SDA_HIGH();   /* NACK：不再读了 */
    }
    else
    {
        SDA_LOW();    /* ACK：还要继续读 */
    }
    SCL_HIGH();
    I2C_Delay();
    SCL_LOW();
    SDA_HIGH();
    return data;
}

/* 写周期延时：EEPROM 写入后内部需要约 5ms 才能进行下一次操作 */
static void W24C02_WaitWrite(void)
{
    uint32_t i = 168000;   /* 约 5ms */
    while (i--)
    {
    }
}

void W24C02_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

    /* 开漏输出，空闲时两根线都是高 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOF, &GPIO_InitStructure);

    SCL_HIGH();
    SDA_HIGH();
}

void W24C02_WriteBytes(uint8_t addr, const uint8_t *data, uint8_t len)
{
    while (len--)
    {
        I2C_Start();
        I2C_SendByte(W24C02_ADDR_WRITE);
        I2C_SendByte(addr++);
        I2C_SendByte(*data++);
        I2C_Stop();
        W24C02_WaitWrite();   /* 逐字节写，简单可靠（就 3 字节，用不着页写） */
    }
}

void W24C02_ReadBytes(uint8_t addr, uint8_t *data, uint8_t len)
{
    I2C_Start();
    I2C_SendByte(W24C02_ADDR_WRITE);
    I2C_SendByte(addr);

    I2C_Start();              /* 重复起始，转读 */
    I2C_SendByte(W24C02_ADDR_READ);
    while (len--)
    {
        *data++ = I2C_RecvByte(len == 0 ? 1 : 0);
    }
    I2C_Stop();
}
