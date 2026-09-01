#include "bsp_esp8266.h"

/***************************************************************************************************
 * 接收用环形缓冲：串口中断里只管往里塞，任务里慢慢取
 * ESP8266 默认 115200，字节中断完全来得及，代码比 DMA 简单
 ***************************************************************************************************/
#define ESP_RX_BUF_SIZE   2048

static uint8_t           s_rx_buf[ESP_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0;   /* 中断写入位置 */
static volatile uint16_t s_rx_tail = 0;   /* 任务读取位置 */

#define ESP_RST_HIGH()  GPIO_SetBits(GPIOD, GPIO_Pin_0)
#define ESP_RST_LOW()   GPIO_ResetBits(GPIOD, GPIO_Pin_0)

/* 中断里收到一个字节 */
static void ESP8266_PushByte(uint8_t ch)
{
    uint16_t next = (s_rx_head + 1) % ESP_RX_BUF_SIZE;

    if (next != s_rx_tail)   /* 满了就丢最新的，防止覆盖没读的数据 */
    {
        s_rx_buf[s_rx_head] = ch;
        s_rx_head = next;
    }
}

void UART5_IRQHandler(void)
{
    if (USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
    {
        ESP8266_PushByte((uint8_t)USART_ReceiveData(UART5));
    }
}

void ESP8266_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    /* PC12 = UART5_TX，PD2 = UART5_RX */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2,  GPIO_AF_UART5);

    /* PD0 = RST 输出，平时拉高 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    ESP_RST_HIGH();

    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStructure);

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    USART_Cmd(UART5, ENABLE);

    /* 网络数据不急，中断优先级放低 */
    NVIC_InitStructure.NVIC_IRQChannel                   = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 9;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void ESP8266_SendRaw(const uint8_t *data, uint16_t len)
{
    while (len--)
    {
        while (USART_GetFlagStatus(UART5, USART_FLAG_TXE) == RESET)
        {
        }
        USART_SendData(UART5, *data++);
    }
}

void ESP8266_SendStr(const char *str)
{
    ESP8266_SendRaw((const uint8_t *)str, strlen(str));
}

void ESP8266_FlushRx(void)
{
    s_rx_tail = s_rx_head;
}

uint8_t ESP8266_ReadByte(uint8_t *ch, uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();

    while (s_rx_tail == s_rx_head)
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms))
        {
            return 0;
        }
        vTaskDelay(1);
    }
    *ch = s_rx_buf[s_rx_tail];
    s_rx_tail = (s_rx_tail + 1) % ESP_RX_BUF_SIZE;
    return 1;
}

uint8_t ESP8266_WaitStr(const char *str, uint32_t timeout_ms)
{
    uint8_t   ch;
    uint16_t  matched = 0;
    uint16_t  target_len = strlen(str);
    TickType_t start = xTaskGetTickCount();

    while ((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeout_ms))
    {
        uint32_t left = timeout_ms - (xTaskGetTickCount() - start);
        if (ESP8266_ReadByte(&ch, left) == 0)
        {
            break;
        }
        if (ch == (uint8_t)str[matched])
        {
            matched++;
            if (matched == target_len)
            {
                return 1;
            }
        }
        else
        {
            /* 不匹配就重新对齐（处理 "abab" 类部分重叠） */
            matched = (ch == (uint8_t)str[0]) ? 1 : 0;
        }
    }
    return 0;
}

uint8_t ESP8266_SendCmd(const char *cmd, const char *ack, uint32_t timeout_ms)
{
    ESP8266_FlushRx();
    ESP8266_SendStr(cmd);
    ESP8266_SendStr("\r\n");
    return ESP8266_WaitStr(ack, timeout_ms);
}
