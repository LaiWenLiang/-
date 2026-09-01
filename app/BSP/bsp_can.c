#include "bsp_can.h"

static QueueHandle_t s_can_rx_queue = NULL;

void BSP_CAN_Init(QueueHandle_t rx_queue)
{
    GPIO_InitTypeDef       GPIO_InitStructure;
    CAN_InitTypeDef        CAN_InitStructure;
    CAN_FilterInitTypeDef  CAN_FilterInitStructure;
    NVIC_InitTypeDef       NVIC_InitStructure;

    s_can_rx_queue = rx_queue;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    /* PB8 = CAN_RX, PB9 = CAN_TX */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);

    /* 500Kbps: APB1=42MHz, 分频7 -> 6MHz, 12Tq(1+9+2) -> 500KHz */
    CAN_DeInit(CAN1);
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;    /* 总线关闭自动恢复，工业现场必备 */
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW  = CAN_SJW_1tq;
    CAN_InitStructure.CAN_BS1  = CAN_BS1_9tq;
    CAN_InitStructure.CAN_BS2  = CAN_BS2_2tq;
    CAN_InitStructure.CAN_Prescaler = 7;
    CAN_Init(CAN1, &CAN_InitStructure);

    /* 过滤器0：32位掩码模式，全接收（台架调试期；量产可改为只收 ID1/ID2） */
    CAN_FilterInitStructure.CAN_FilterNumber         = 0;
    CAN_FilterInitStructure.CAN_FilterMode           = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale          = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh         = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow          = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh     = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow      = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation     = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* 返回 0 成功，1 无空邮箱 */
uint8_t BSP_CAN_Send(uint32_t id, const uint8_t *data, uint8_t len)
{
    CanTxMsg tx;
    uint8_t  mailbox;

    tx.StdId = id;
    tx.ExtId = 0;
    tx.IDE   = CAN_Id_Standard;
    tx.RTR   = CAN_RTR_Data;
    tx.DLC   = (len > 8) ? 8 : len;
    memcpy(tx.Data, data, tx.DLC);

    mailbox = CAN_Transmit(CAN1, &tx);
    if (mailbox == CAN_TxStatus_NoMailBox)
    {
        return 1;
    }

    /* 等待发送完成（带超时） */
    uint32_t timeout = 16800;   /* ~1ms */
    while (CAN_TransmitStatus(CAN1, mailbox) != CAN_TxStatus_Ok)
    {
        if (--timeout == 0)
        {
            return 1;
        }
    }
    return 0;
}

void CAN1_RX0_IRQHandler(void)
{
    CanRxMsg   rx;
    CanFrame_t frame;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_Receive(CAN1, CAN_FIFO0, &rx);
        if (rx.IDE == CAN_Id_Standard && s_can_rx_queue != NULL)
        {
            frame.id  = rx.StdId;
            frame.len = rx.DLC;
            memcpy(frame.data, rx.Data, rx.DLC > 8 ? 8 : rx.DLC);
            xQueueSendFromISR(s_can_rx_queue, &frame, &xHigherPriorityTaskWoken);
        }
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
