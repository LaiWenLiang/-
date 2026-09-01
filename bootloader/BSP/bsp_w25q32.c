#include "bsp_w25q32.h"

/* W25Q32 指令 */
#define CMD_WRITE_ENABLE   0x06
#define CMD_READ_STATUS    0x05
#define CMD_READ_DATA      0x03
#define CMD_PAGE_PROGRAM   0x02
#define CMD_SECTOR_ERASE   0x20
#define CMD_READ_ID        0x9F

#define W25Q32_PAGE_SIZE   256
#define W25Q32_SECTOR_SIZE 4096

/* 片选拉低/拉高 */
#define CS_LOW()   GPIO_ResetBits(GPIOB, GPIO_Pin_12)
#define CS_HIGH()  GPIO_SetBits(GPIOB, GPIO_Pin_12)

/* SPI 收发一个字节 */
static uint8_t W25Q32_SwapByte(uint8_t data)
{
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
    {
    }
    SPI_I2S_SendData(SPI2, data);
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
    {
    }
    return SPI_I2S_ReceiveData(SPI2);
}

void W25Q32_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

    /* PB13(SCK) PB14(MISO) PB15(MOSI) 复用 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);

    /* PB12(CS) 普通输出，平时拉高 */
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    CS_HIGH();

    /* 模式0，主机，8位，软件片选；84MHz/8 = 10.5MHz */
    SPI_InitStructure.SPI_Direction         = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL              = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA              = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial     = 7;
    SPI_Init(SPI2, &SPI_InitStructure);
    SPI_Cmd(SPI2, ENABLE);
}

void W25Q32_ReadID(uint8_t *mf_id, uint16_t *dev_id)
{
    uint8_t id_h, id_l;

    CS_LOW();
    W25Q32_SwapByte(CMD_READ_ID);
    *mf_id = W25Q32_SwapByte(0xFF);
    id_h   = W25Q32_SwapByte(0xFF);
    id_l   = W25Q32_SwapByte(0xFF);
    CS_HIGH();

    *dev_id = ((uint16_t)id_h << 8) | id_l;
}

/* 写使能（每次擦除/写入前都要发一次） */
static void W25Q32_WriteEnable(void)
{
    CS_LOW();
    W25Q32_SwapByte(CMD_WRITE_ENABLE);
    CS_HIGH();
}

/* 等待芯片内部操作完成（状态寄存器 bit0 = 0 表示空闲） */
static void W25Q32_WaitBusy(void)
{
    uint8_t status;

    CS_LOW();
    W25Q32_SwapByte(CMD_READ_STATUS);
    do
    {
        status = W25Q32_SwapByte(0xFF);
    } while (status & 0x01);
    CS_HIGH();
}

void W25Q32_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    CS_LOW();
    W25Q32_SwapByte(CMD_READ_DATA);
    W25Q32_SwapByte((uint8_t)(addr >> 16));
    W25Q32_SwapByte((uint8_t)(addr >> 8));
    W25Q32_SwapByte((uint8_t)(addr));
    while (len--)
    {
        *buf++ = W25Q32_SwapByte(0xFF);
    }
    CS_HIGH();
}

/* 写一页（不超过 256 字节，且不跨页） */
static void W25Q32_WritePage(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    W25Q32_WriteEnable();

    CS_LOW();
    W25Q32_SwapByte(CMD_PAGE_PROGRAM);
    W25Q32_SwapByte((uint8_t)(addr >> 16));
    W25Q32_SwapByte((uint8_t)(addr >> 8));
    W25Q32_SwapByte((uint8_t)(addr));
    while (len--)
    {
        W25Q32_SwapByte(*buf++);
    }
    CS_HIGH();

    W25Q32_WaitBusy();
}

void W25Q32_Write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint16_t once;

    while (len > 0)
    {
        /* 本次最多写到页尾 */
        once = W25Q32_PAGE_SIZE - (addr % W25Q32_PAGE_SIZE);
        if (once > len)
        {
            once = (uint16_t)len;
        }
        W25Q32_WritePage(addr, buf, once);
        addr += once;
        buf  += once;
        len  -= once;
    }
}

void W25Q32_EraseSector(uint32_t addr)
{
    W25Q32_WriteEnable();

    CS_LOW();
    W25Q32_SwapByte(CMD_SECTOR_ERASE);
    W25Q32_SwapByte((uint8_t)(addr >> 16));
    W25Q32_SwapByte((uint8_t)(addr >> 8));
    W25Q32_SwapByte((uint8_t)(addr));
    CS_HIGH();

    W25Q32_WaitBusy();
}

void W25Q32_EraseRange(uint32_t addr, uint32_t len)
{
    uint32_t end = addr + len;

    addr = addr - (addr % W25Q32_SECTOR_SIZE);   /* 对齐到扇区头 */
    while (addr < end)
    {
        W25Q32_EraseSector(addr);
        addr += W25Q32_SECTOR_SIZE;
    }
}
