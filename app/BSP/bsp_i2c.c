#include "bsp_i2c.h"

/* 超时等待宏：防止总线异常时死等卡死（F4 硬件 I2C 经典坑） */
#define I2C_WAIT_FLAG(expr)                                            \
    do {                                                               \
        uint32_t _to = BSP_I2C_TIMEOUT_MS * 16800;                     \
        while (!(expr)) { if (--_to == 0) return -1; }                 \
    } while (0)

void BSP_I2C2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef  I2C_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;   /* I2C 必须开漏 */
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);

    I2C_DeInit(I2C2);
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed          = 100000;
    I2C_Init(I2C2, &I2C_InitStructure);
    I2C_Cmd(I2C2, ENABLE);
}

/* 读：Start -> 地址(写) -> 寄存器 -> 重复Start -> 地址(读) -> 数据 -> Stop
   返回 0 成功，-1 超时/失败 */
int8_t BSP_I2C2_Read(uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    I2C_WAIT_FLAG(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) == RESET);

    I2C_GenerateSTART(I2C2, ENABLE);
    I2C_WAIT_FLAG(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C2, dev_addr, I2C_Direction_Transmitter);
    I2C_WAIT_FLAG(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2C2, reg);
    I2C_WAIT_FLAG(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTART(I2C2, ENABLE);
    I2C_WAIT_FLAG(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C2, dev_addr, I2C_Direction_Receiver);
    I2C_WAIT_FLAG(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    while (len--)
    {
        if (len == 0)
        {
            I2C_AcknowledgeConfig(I2C2, DISABLE);
            I2C_GenerateSTOP(I2C2, ENABLE);
        }
        I2C_WAIT_FLAG(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
        *buf++ = I2C_ReceiveData(I2C2);
    }

    I2C_AcknowledgeConfig(I2C2, ENABLE);
    return 0;
}

int8_t BSP_I2C2_Write(uint8_t dev_addr, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    I2C_WAIT_FLAG(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY) == RESET);

    I2C_GenerateSTART(I2C2, ENABLE);
    I2C_WAIT_FLAG(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(I2C2, dev_addr, I2C_Direction_Transmitter);
    I2C_WAIT_FLAG(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2C2, reg);
    I2C_WAIT_FLAG(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    while (len--)
    {
        I2C_SendData(I2C2, *buf++);
        I2C_WAIT_FLAG(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }

    I2C_GenerateSTOP(I2C2, ENABLE);
    return 0;
}
