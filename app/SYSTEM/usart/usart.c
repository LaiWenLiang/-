#include "usart.h"
#include "bsp_crc32.h"
#include "task.h"
#include <string.h>
#include <stdarg.h>

/***************************************************************************************************
 * USART1 调试口（PA9 TX / PA10 RX）：中断接收 + TX DMA2 Stream7 Channel4
 * 两种模式（g_uart1_mode，KEY3 切换）：
 *   蓝牙  : 文本行缓冲，供 VOFA 调参
 *   RS485 : 0x55 0xAA 帧嗅探，完整帧 CRC16 校验后投递 OTA 队列；DE 脚 PD3 控制收发方向
 ***************************************************************************************************/
#define DBG_RX_BUF   64
#define DBG_TX_BUF   256

#define RS485_DE_PIN   GPIO_Pin_3
#define RS485_DE_GPIO  GPIOD

uint8_t g_uart1_mode = UART1_MODE_BT;   /* 默认蓝牙模式 */

static char     s_dbg_buf[DBG_RX_BUF];
static char     s_dbg_line[DBG_RX_BUF];
static volatile uint8_t  s_dbg_len  = 0;
static volatile uint8_t  s_dbg_flag = 0;
static uint8_t  s_dbg_tx_buf[DBG_TX_BUF];   /* DMA 发送缓冲（发起前必须等上一帧发完） */
static uint8_t  s_dbg_tx_fmt[DBG_TX_BUF];   /* Debug_Printf 格式化暂存 */

static QueueHandle_t s_ota_rx_queue = NULL;             /* RS485 帧投递队列 */
static uint8_t  s_ota_frame[OTA_FRAME_MAX_SIZE];        /* 拼帧缓冲 */
static uint16_t s_ota_pos  = 0;                         /* 已收到的字节数 */
static uint16_t s_ota_want = 0;                         /* 本帧总长度（含帧头CRC），0=还在找帧头 */

void Debug_UART_SetOtaQueue(QueueHandle_t rx_queue)
{
    s_ota_rx_queue = rx_queue;
}

void Debug_UART_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOD | RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    /* RS485 方向脚：PD3，低电平=接收（默认） */
    GPIO_InitStructure.GPIO_Pin   = RS485_DE_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_ResetBits(RS485_DE_GPIO, RS485_DE_PIN);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    /* TX -> DMA2 Stream7 Channel4，单次模式（每帧手动重启） */
    DMA_DeInit(DMA2_Stream7);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)s_dbg_tx_buf;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize         = DBG_TX_BUF;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_Low;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);

    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_Cmd(USART1, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 8;   /* 低优先级调试口 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

uint16_t Debug_UART_ReadLine(char *buf, uint16_t maxlen)
{
    uint16_t len;

    if (!s_dbg_flag)
    {
        return 0;
    }

    USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
    len = strlen(s_dbg_line);
    if (len >= maxlen)
    {
        len = maxlen - 1;
    }
    memcpy(buf, s_dbg_line, len);
    buf[len] = '\0';
    s_dbg_flag = 0;
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    return len;
}

/* RS485 模式：逐字节拼 0x55 0xAA 帧，CRC16 通过后投递 OTA 队列 */
static void Ota_RxByte(uint8_t ch)
{
    if (s_ota_want == 0)
    {
        /* 还在找帧头 */
        if (s_ota_pos == 0 && ch == OTA_FRAME_HEAD_0)
        {
            s_ota_frame[s_ota_pos++] = ch;
        }
        else if (s_ota_pos == 1 && ch == OTA_FRAME_HEAD_1)
        {
            s_ota_frame[s_ota_pos++] = ch;
            s_ota_want = 5;   /* 至少先收齐 帧头2+命令1+长度2 */
        }
        else
        {
            s_ota_pos = 0;
        }
        return;
    }

    s_ota_frame[s_ota_pos++] = ch;

    if (s_ota_pos == 5)
    {
        /* 长度字段收齐，算本帧总长度 */
        s_ota_want = (uint16_t)(2 + 1 + 2 + s_ota_frame[3] + (s_ota_frame[4] << 8) + 2);
        if (s_ota_frame[3] > OTA_FRAME_MAX_DATA)   /* 长度超上限，废帧重来 */
        {
            s_ota_pos  = 0;
            s_ota_want = 0;
        }
    }
    else if (s_ota_pos == s_ota_want)
    {
        /* 整帧收完，校验 CRC16（对 命令字+长度+数据 计算） */
        uint16_t data_total = (uint16_t)(1 + 2 + s_ota_frame[3] + (s_ota_frame[4] << 8));
        uint16_t crc_recv   = (uint16_t)(s_ota_frame[s_ota_pos - 2] | (s_ota_frame[s_ota_pos - 1] << 8));
        uint16_t crc_calc   = CRC16_Modbus(&s_ota_frame[2], data_total);

        if (crc_recv == crc_calc && s_ota_rx_queue != NULL)
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            OtaFrame_t frame;

            frame.len = s_ota_pos;
            memcpy(frame.buf, s_ota_frame, s_ota_pos);
            xQueueSendFromISR(s_ota_rx_queue, &frame, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
        s_ota_pos  = 0;
        s_ota_want = 0;
    }
}

void USART1_IRQHandler(void)
{
    /* RS485 模式发送完成：拉低 DE 回到接收态 */
    if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
        USART_ClearITPendingBit(USART1, USART_IT_TC);
        USART_ITConfig(USART1, USART_IT_TC, DISABLE);
        GPIO_ResetBits(RS485_DE_GPIO, RS485_DE_PIN);
    }

    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        uint8_t ch = (uint8_t)USART_ReceiveData(USART1);

        if (g_uart1_mode == UART1_MODE_485)
        {
            Ota_RxByte(ch);
            return;
        }

        /* 蓝牙模式：文本行缓冲 */
        if (ch == '\n' || ch == '\r')
        {
            if (s_dbg_len > 0)
            {
                s_dbg_buf[s_dbg_len] = '\0';
                strcpy(s_dbg_line, s_dbg_buf);
                s_dbg_flag = 1;
                s_dbg_len  = 0;
            }
        }
        else if (s_dbg_len < DBG_RX_BUF - 1)
        {
            s_dbg_buf[s_dbg_len++] = ch;
        }
        else
        {
            s_dbg_len = 0;   /* 溢出丢弃 */
        }
    }
}

/* DMA 发送一帧：等上一帧发完（保证 s_dbg_tx_buf 可覆写），然后启动 DMA */
uint16_t Debug_UART_Write(const uint8_t *data, uint16_t len)
{
    if (len == 0)
    {
        return 0;
    }
    if (len > DBG_TX_BUF)
    {
        len = DBG_TX_BUF;
    }

    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE)          /* 等 DMA 传完 */
    {
    }
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)/* 等最后一个字节移出 */
    {
    }

    memcpy(s_dbg_tx_buf, data, len);

    if (g_uart1_mode == UART1_MODE_485)
    {
        GPIO_SetBits(RS485_DE_GPIO, RS485_DE_PIN);       /* 切到发送方向 */
        USART_ITConfig(USART1, USART_IT_TC, ENABLE);     /* 发完由 TC 中断拉回接收 */
    }

    DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
    DMA_SetCurrDataCounter(DMA2_Stream7, len);
    DMA_Cmd(DMA2_Stream7, ENABLE);
    return len;
}

/* 格式化打印（DMA 发送），用法同 printf */
uint16_t Debug_Printf(const char *fmt, ...)
{
    va_list ap;
    int     len;

    va_start(ap, fmt);
    len = vsnprintf((char *)s_dbg_tx_fmt, DBG_TX_BUF, fmt, ap);
    va_end(ap);
    if (len <= 0)
    {
        return 0;
    }
    return Debug_UART_Write(s_dbg_tx_fmt, (uint16_t)((len > DBG_TX_BUF) ? DBG_TX_BUF : len));
}

/* printf 重定向到 USART1（轮询兜底；高频打印请改用 Debug_Printf 走 DMA） */
int fputc(int ch, FILE *f)
{
    (void)f;
    while ((USART1->SR & USART_FLAG_TC) == RESET)
    {
    }
    USART_SendData(USART1, (uint8_t)ch);
    return ch;
}

/***************************************************************************************************
 * USART2 UWB 基站（PA2 TX / PA3 RX）：DMA 循环接收 + 空闲中断整帧入队
 ***************************************************************************************************/
static uint8_t           s_uwb_dma_buf[UWB_RX_BUF_SIZE];
static QueueHandle_t     s_uwb_rx_queue = NULL;
static volatile uint32_t s_uwb_last_rx  = 0;

uint32_t UWB_UART_LastRxTick(void)
{
    return s_uwb_last_rx;
}

void UWB_UART_Init(uint32_t baudrate, QueueHandle_t rx_queue)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    s_uwb_rx_queue = rx_queue;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    /* RX -> DMA1 Stream5 Channel4，循环模式 */
    DMA_DeInit(DMA1_Stream5);
    DMA_InitStructure.DMA_Channel            = DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr    = (uint32_t)s_uwb_dma_buf;
    DMA_InitStructure.DMA_DIR                = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize         = UWB_RX_BUF_SIZE;
    DMA_InitStructure.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc          = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode               = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority           = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode           = DMA_FIFOMode_Disable;
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream5, ENABLE);

    USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    USART_Cmd(USART2, ENABLE);

    /* 中断优先级必须 >= configMAX_SYSCALL_INTERRUPT_PRIORITY(5) 才能调 FromISR API */
    NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/* 空闲中断：一帧结束，拷贝出有效数据投递队列，复位 DMA 指针 */
void USART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        BaseType_t  xHigherPriorityTaskWoken = pdFALSE;
        UWB_Frame_t frame;
        uint16_t    len;

        (void)USART2->SR;   /* 清 IDLE：先读 SR 再读 DR */
        (void)USART2->DR;

        DMA_Cmd(DMA1_Stream5, DISABLE);
        len = UWB_RX_BUF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream5);
        if (len > 0 && len <= UWB_RX_BUF_SIZE && s_uwb_rx_queue != NULL)
        {
            frame.len = len;
            memcpy(frame.buf, s_uwb_dma_buf, len);
            xQueueSendFromISR(s_uwb_rx_queue, &frame, &xHigherPriorityTaskWoken);
            s_uwb_last_rx = xTaskGetTickCountFromISR();
        }
        DMA_SetCurrDataCounter(DMA1_Stream5, UWB_RX_BUF_SIZE);
        DMA_Cmd(DMA1_Stream5, ENABLE);

        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/***************************************************************************************************
 * USART3 语音模块（PC10 TX / PC11 RX）：轮询发送
 ***************************************************************************************************/
void Speaker_UART_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);

    USART_InitStructure.USART_BaudRate            = baudrate;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);
    USART_Cmd(USART3, ENABLE);
}

void Speaker_UART_Send(const uint8_t *data, uint16_t len)
{
    while (len--)
    {
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
        {
        }
        USART_SendData(USART3, *data++);
    }
}
