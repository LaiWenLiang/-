#include "app_onenet_ota.h"
#include "app.h"
#include "ota_config.h"
#include "bsp_esp8266.h"
#include "bsp_w25q32.h"
#include "bsp_w24c02.h"
#include "bsp_crc32.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* OneNET 鉴权 token 生成（SHA1/HMAC/base64 实现在本文件末尾） */
static uint16_t OneNET_Token(char *out, const char *product_id, const char *dev_key_base64, uint32_t expire_time);

/***************************************************************************************************
 * OneNET OTA 升级流程（HTTP 透传方式，走 ESP8266）：
 *
 *   连接 WiFi
 *     -> POST /ota/device/version       上报当前版本
 *     -> GET  /ota/south/check          查询有没有升级任务（有则返回 token/固件大小/版本号）
 *     -> GET  /ota/south/download/token 分片下载固件，边收边写 W25Q32 暂存区
 *     -> CRC32 校验 -> 写元数据 -> 写 EEPROM 标志 UPDATE -> 复位
 *     -> Bootloader 接管（和 RS485 升级共用同一套底层）
 *
 * ESP8266 工作在"透传模式"（AT+CIPMODE=1）：HTTP 响应是纯净的字节流，没有 +IPD 前缀，
 * 方便按 Content-Length 精确接收固件。
 ***************************************************************************************************/

#define OTA_HOST            "ota.heclouds.com"
#define TOKEN_EXPIRE        4102444800UL   /* token 过期时间：2100-01-01，一次签名长期有效 */
#define HTTP_HEAD_BUF       768            /* HTTP 响应头缓冲 */
#define JSON_BUF            1200           /* 查询任务返回的 JSON 缓冲 */
#define DL_TIMEOUT_MS       15000          /* 下载数据流停顿超时 */
#define WIFI_RETRY_MS       10000          /* WiFi 断了重连间隔 */
#define CHECK_PERIOD_MS     30000          /* 正常时每隔多久查一次升级任务 */

static char     s_token[192];     /* OneNET 鉴权 token */
static char     s_http_head[HTTP_HEAD_BUF];
static uint8_t  s_json[JSON_BUF];
static uint8_t  s_page[256];      /* 写 W25Q32 用的页缓冲 */

/* ---------------- 小工具：从 JSON 里取字符串/整数 ---------------- */

/* 找 "key":"value" 里的 value，返回 1=找到 */
static uint8_t Json_GetStr(uint8_t *json, const char *key, char *out, uint16_t out_max)
{
    char   pattern[32];
    char  *p, *start;
    uint16_t len;

    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    p = strstr((char *)json, pattern);
    if (p == NULL) return 0;

    start = strchr(p + strlen(pattern), ':');
    if (start == NULL) return 0;
    start = strchr(start, '"');
    if (start == NULL) return 0;
    start++;                            /* 跳过开头的引号 */

    p = strchr(start, '"');
    if (p == NULL) return 0;

    len = p - start;
    if (len >= out_max) len = out_max - 1;
    memcpy(out, start, len);
    out[len] = '\0';
    return 1;
}

/* 找 "key":123 里的数字，返回 1=找到 */
static uint8_t Json_GetInt(uint8_t *json, const char *key, uint32_t *value)
{
    char  pattern[32];
    char *p;

    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    p = strstr((char *)json, pattern);
    if (p == NULL) return 0;
    p = strchr(p + strlen(pattern), ':');
    if (p == NULL) return 0;

    *value = (uint32_t)atoi(p + 1);
    return 1;
}

/* ---------------- ESP8266 基础动作 ---------------- */

/* 连 WiFi，成功返回 1 */
static uint8_t Wifi_Connect(void)
{
    char cmd[80];

    if (ESP8266_SendCmd("AT", "OK", 1000) == 0)
    {
        return 0;
    }
    ESP8266_SendCmd("ATE0", "OK", 1000);                 /* 关回显，解析干净 */
    ESP8266_SendCmd("AT+CWMODE_CUR=1", "OK", 1000);      /*  Station 模式，不写 Flash */
    ESP8266_SendCmd("AT+CIPMODE=1", "OK", 1000);         /* 透传模式 */

    snprintf(cmd, sizeof(cmd), "AT+CWJAP_CUR=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASSWORD);
    if (ESP8266_SendCmd(cmd, "WIFI GOT IP", 15000) == 0) /* 连 WiFi 比较慢 */
    {
        return 0;
    }
    Debug_Printf("onenet: wifi connected\n");
    return 1;
}

/* 建立到 OTA 服务器的 TCP 连接并进入透传发送态，成功返回 1 */
static uint8_t Http_Open(void)
{
    char cmd[80];

    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",80", OTA_HOST);
    if (ESP8266_SendCmd(cmd, "CONNECT", 8000) == 0)
    {
        return 0;
    }
    if (ESP8266_SendCmd("AT+CIPSEND", ">", 3000) == 0)
    {
        ESP8266_SendCmd("AT+CIPCLOSE", "CLOSED", 2000);
        return 0;
    }
    return 1;
}

/* 退出透传并关闭连接 */
static void Http_Close(void)
{
    vTaskDelay(pdMS_TO_TICKS(500));   /* 发 +++ 前要静默一会，AT 固件要求 */
    ESP8266_SendStr("+++");
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP8266_SendCmd("AT+CIPCLOSE", "OK", 2000);
}

/* 发送 HTTP 请求头（透传态直接发原文） */
static void Http_SendHead(const char *method, const char *path, uint16_t body_len)
{
    char head[640];
    uint16_t len;

    if (body_len > 0)
    {
        len = snprintf(head, sizeof(head),
                       "%s %s HTTP/1.1\r\n"
                       "Host: %s\r\n"
                       "Authorization: %s\r\n"
                       "Content-Type: application/json\r\n"
                       "Content-Length: %u\r\n"
                       "Connection: close\r\n\r\n",
                       method, path, OTA_HOST, s_token, body_len);
    }
    else
    {
        len = snprintf(head, sizeof(head),
                       "%s %s HTTP/1.1\r\n"
                       "Host: %s\r\n"
                       "Authorization: %s\r\n"
                       "Connection: close\r\n\r\n",
                       method, path, OTA_HOST, s_token);
    }
    ESP8266_SendRaw((uint8_t *)head, len);
}

/* 读 HTTP 响应头（读到 \r\n\r\n 为止），返回 1=成功；s_http_head 里是响应头原文 */
static uint8_t Http_ReadHead(uint32_t timeout_ms)
{
    uint8_t  ch;
    uint16_t len = 0;

    while (len < HTTP_HEAD_BUF - 1)
    {
        if (ESP8266_ReadByte(&ch, timeout_ms) == 0)
        {
            return 0;
        }
        s_http_head[len++] = ch;
        if (len >= 4 && memcmp(&s_http_head[len - 4], "\r\n\r\n", 4) == 0)
        {
            s_http_head[len] = '\0';
            return 1;
        }
    }
    s_http_head[len] = '\0';
    return 0;
}

/* 从响应头里取 Content-Length，没找到返回 -1（说明是 chunked 或其它，放弃） */
static int32_t Http_BodyLen(void)
{
    char *p = strstr(s_http_head, "Content-Length:");

    if (p == NULL)
    {
        p = strstr(s_http_head, "content-length:");   /* 有的服务器小写 */
    }
    if (p == NULL) return -1;
    return atoi(p + 15);
}

/* ---------------- OneNET 接口 ---------------- */

/* 上报当前版本号 */
static void Onenet_ReportVersion(void)
{
    char path[96];
    char body[48];
    uint16_t body_len;

    if (Http_Open() == 0) return;

    snprintf(path, sizeof(path), "/ota/device/version?dev_id=%s", ONENET_DEVICE_ID);
    body_len = snprintf(body, sizeof(body), "{\"f_version\":\"%s\"}", APP_VERSION);

    Http_SendHead("POST", path, body_len);
    ESP8266_SendRaw((uint8_t *)body, body_len);

    if (Http_ReadHead(8000))
    {
        Debug_Printf("onenet: report version %s -> %s\n", APP_VERSION,
                     strstr(s_http_head, "200") ? "ok" : "fail");
    }
    Http_Close();
}

/* 查询升级任务。有任务返回 1，并把固件大小/任务 token/版本号存到出参 */
static uint8_t Onenet_CheckTask(uint32_t *fw_size, char *dl_token, uint16_t dl_token_max,
                                char *ver, uint16_t ver_max)
{
    char     path[160];
    int32_t  body_len;
    uint16_t got = 0;
    uint8_t  ch;

    if (Http_Open() == 0) return 0;

    snprintf(path, sizeof(path),
             "/ota/south/check?dev_id=%s&manuf=%s&model=%s&type=1&version=%s&cdn=false",
             ONENET_DEVICE_ID, ONENET_MANUF, ONENET_MODEL, APP_VERSION);
    Http_SendHead("GET", path, 0);

    if (Http_ReadHead(8000) == 0)
    {
        Http_Close();
        return 0;
    }

    body_len = Http_BodyLen();
    if (body_len <= 0 || body_len >= JSON_BUF)
    {
        Http_Close();
        return 0;
    }

    while (got < body_len)
    {
        if (ESP8266_ReadByte(&ch, 8000) == 0) break;
        s_json[got++] = ch;
    }
    s_json[got] = '\0';
    Http_Close();

    Debug_Printf("onenet: check resp %s\n", s_json);

    /* 没有任务时返回里没有 token 字段 */
    if (Json_GetStr(s_json, "token", dl_token, dl_token_max) == 0)
    {
        return 0;
    }
    if (Json_GetInt(s_json, "size", fw_size) == 0)
    {
        return 0;
    }
    if (Json_GetStr(s_json, "version", ver, ver_max) == 0)
    {
        ver[0] = '?';   /* 没解析到版本号也不至于显示空白 */
        ver[1] = '\0';
    }
    return 1;
}

/* 下载固件到 W25Q32 暂存区，返回 1=完整收完 */
static uint8_t Onenet_Download(const char *dl_token, uint32_t fw_size)
{
    char     path[160];
    int32_t  body_len;
    uint32_t received = 0;
    uint16_t page_used = 0;
    uint8_t  ch;

    if (Http_Open() == 0) return 0;

    snprintf(path, sizeof(path), "/ota/south/download/%s?dev_id=%s", dl_token, ONENET_DEVICE_ID);
    Http_SendHead("GET", path, 0);

    if (Http_ReadHead(DL_TIMEOUT_MS) == 0)
    {
        Http_Close();
        return 0;
    }
    if (strstr(s_http_head, "200") == NULL)
    {
        Debug_Printf("onenet: download rejected: %s\n", s_http_head);
        Http_Close();
        return 0;
    }

    body_len = Http_BodyLen();
    if (body_len > 0 && (uint32_t)body_len != fw_size)
    {
        Debug_Printf("onenet: size mismatch %ld/%lu\n", (long)body_len, fw_size);
    }

    /* 升级期间停车 */
    xEventGroupSetBits(g_event_system, EVT_OTA_MODE);

    /* 边收边写：攒满 256 字节写一页 */
    W25Q32_EraseRange(W25Q32_STAGE_ADDR, fw_size);
    while (received < fw_size)
    {
        if (ESP8266_ReadByte(&ch, DL_TIMEOUT_MS) == 0)
        {
            Debug_Printf("onenet: download stall at %lu/%lu\n", received, fw_size);
            goto fail;
        }
        s_page[page_used++] = ch;
        if (page_used == sizeof(s_page))
        {
            W25Q32_Write(W25Q32_STAGE_ADDR + received, s_page, sizeof(s_page));
            received  += sizeof(s_page);
            page_used  = 0;
        }
    }
    if (page_used > 0)   /* 尾部不满一页的零头 */
    {
        W25Q32_Write(W25Q32_STAGE_ADDR + received, s_page, page_used);
        received += page_used;
    }

    Http_Close();
    Debug_Printf("onenet: download done %lu bytes\n", received);
    return 1;

fail:
    Http_Close();
    xEventGroupClearBits(g_event_system, EVT_OTA_MODE);
    return 0;
}

/* 校验 + 写元数据 + 写标志 + 复位（与 RS485 通道同一套收尾） */
static void Onenet_FinishAndReset(uint32_t fw_size)
{
    uint32_t crc;
    uint32_t i;
    uint8_t  meta[8];

    /* 分段回读暂存区算 CRC32 */
    crc = 0xFFFFFFFF;
    for (i = 0; i < fw_size; i += sizeof(s_page))
    {
        uint32_t once = fw_size - i;
        if (once > sizeof(s_page)) once = sizeof(s_page);
        W25Q32_Read(W25Q32_STAGE_ADDR + i, s_page, once);
        crc = CRC32_Append(crc, s_page, once);
    }
    crc = ~crc;

    meta[0] = (uint8_t)(fw_size);
    meta[1] = (uint8_t)(fw_size >> 8);
    meta[2] = (uint8_t)(fw_size >> 16);
    meta[3] = (uint8_t)(fw_size >> 24);
    meta[4] = (uint8_t)(crc);
    meta[5] = (uint8_t)(crc >> 8);
    meta[6] = (uint8_t)(crc >> 16);
    meta[7] = (uint8_t)(crc >> 24);
    W25Q32_EraseSector(W25Q32_META_ADDR);
    W25Q32_Write(W25Q32_META_ADDR, meta, 8);

    /* 写 EEPROM 标志 = UPDATE */
    {
        uint8_t flag[3];
        flag[0] = OTA_FLAG_UPDATE;
        flag[1] = EEPROM_KEY_H;
        flag[2] = EEPROM_KEY_L;
        W24C02_WriteBytes(EEPROM_FLAG_ADDR, flag, 3);
    }

    Debug_Printf("onenet: ota ready, reset to update\n");
    vTaskDelay(pdMS_TO_TICKS(500));
    NVIC_SystemReset();
}

/* ---------------- 主任务 ---------------- */

#define ASK_TIMEOUT_TICKS   pdMS_TO_TICKS(60000)   /* 升级提示 60 秒没人按 = 默认拒绝 */

void App_OnenetOta_Task(void *param)
{
    uint32_t fw_size;
    char     dl_token[64];
    char     new_ver[16];
    char     rejected_token[64] = {0};   /* 用户拒绝过的任务，不再重复提示 */
    uint8_t  wifi_ok = 0;
    TickType_t ask_start;

    (void)param;

    ESP8266_Init();

    /* 生成鉴权 token（一次即可长期用） */
    OneNET_Token(s_token, ONENET_PRODUCT_ID, ONENET_DEVICE_KEY, TOKEN_EXPIRE);

    for (;;)
    {
        if (!wifi_ok)
        {
            wifi_ok = Wifi_Connect();
            if (!wifi_ok)
            {
                vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_MS));
                continue;
            }
        }

        Onenet_ReportVersion();

        if (Onenet_CheckTask(&fw_size, dl_token, sizeof(dl_token), new_ver, sizeof(new_ver)))
        {
            Debug_Printf("onenet: new firmware %s, size=%lu\n", new_ver, fw_size);

            if (fw_size >= 512 && fw_size <= OTA_APP_MAX_SIZE &&
                strcmp(dl_token, rejected_token) != 0)          /* 拒绝过的任务不再提示 */
            {
                /* 通知 OLED 显示提示，等用户按键（KEY1=接受 / KEY2=拒绝，60 秒默认拒绝） */
                strncpy(g_ota_new_ver, new_ver, sizeof(g_ota_new_ver) - 1);
                g_ota_new_ver[sizeof(g_ota_new_ver) - 1] = '\0';
                g_ota_state = OTA_ST_ASK;
                ask_start   = xTaskGetTickCount();

                while (g_ota_state == OTA_ST_ASK)
                {
                    if ((xTaskGetTickCount() - ask_start) > ASK_TIMEOUT_TICKS)
                    {
                        g_ota_state = OTA_ST_REJECT;
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(200));
                }

                if (g_ota_state == OTA_ST_ACCEPT)
                {
                    g_ota_state = OTA_ST_UPGRADING;             /* OLED 显示"正在升级中" */
                    if (Onenet_Download(dl_token, fw_size))
                    {
                        Onenet_FinishAndReset(fw_size);         /* 走到这里就复位了 */
                    }
                    g_ota_state = OTA_ST_IDLE;                  /* 下载失败：回到正常显示 */
                }
                else
                {
                    /* 拒绝：记住这个任务 token，不再重复弹提示 */
                    strncpy(rejected_token, dl_token, sizeof(rejected_token) - 1);
                    rejected_token[sizeof(rejected_token) - 1] = '\0';
                    Debug_Printf("onenet: user rejected %s\n", new_ver);
                    g_ota_state = OTA_ST_IDLE;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CHECK_PERIOD_MS));

        /* 心跳：顺便确认 WiFi 还连着 */
        if (ESP8266_SendCmd("AT", "OK", 1000) == 0)
        {
            wifi_ok = 0;
        }
    }
}

/* ==================== OneNET 鉴权（原 bsp_token.c 并入） ==================== */
/***************************************************************************************************
 * 本文件包含三个小工具：SHA1、HMAC-SHA1、base64 编解码
 * 都是标准算法的直白实现，没有依赖任何库
 ***************************************************************************************************/

/* ==================== SHA1 ==================== */
typedef struct
{
    uint32_t h[5];        /* 结果寄存器 */
    uint8_t  buf[64];     /* 当前块缓冲 */
    uint32_t buf_len;     /* 缓冲里已有字节数 */
    uint32_t total_len;   /* 总字节数 */
} SHA1_Ctx;

static uint32_t sha1_rol(uint32_t v, uint8_t n)
{
    return (v << n) | (v >> (32 - n));
}

/* 处理一个 64 字节块 */
static void sha1_block(SHA1_Ctx *ctx, const uint8_t *block)
{
    uint32_t w[80];
    uint32_t a, b, c, d, e, f, k, tmp;
    uint8_t  i;

    for (i = 0; i < 16; i++)
    {
        w[i] = ((uint32_t)block[i * 4] << 24) | ((uint32_t)block[i * 4 + 1] << 16) |
               ((uint32_t)block[i * 4 + 2] << 8) | block[i * 4 + 3];
    }
    for (i = 16; i < 80; i++)
    {
        w[i] = sha1_rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }

    a = ctx->h[0]; b = ctx->h[1]; c = ctx->h[2]; d = ctx->h[3]; e = ctx->h[4];

    for (i = 0; i < 80; i++)
    {
        if (i < 20)      { f = (b & c) | ((~b) & d);       k = 0x5A827999; }
        else if (i < 40) { f = b ^ c ^ d;                  k = 0x6ED9EBA1; }
        else if (i < 60) { f = (b & c) | (b & d) | (c & d); k = 0x8F1BBCDC; }
        else             { f = b ^ c ^ d;                  k = 0xCA62C1D6; }
        tmp = sha1_rol(a, 5) + f + e + k + w[i];
        e = d; d = c; c = sha1_rol(b, 30); b = a; a = tmp;
    }

    ctx->h[0] += a; ctx->h[1] += b; ctx->h[2] += c; ctx->h[3] += d; ctx->h[4] += e;
}

static void sha1_init(SHA1_Ctx *ctx)
{
    ctx->h[0] = 0x67452301; ctx->h[1] = 0xEFCDAB89;
    ctx->h[2] = 0x98BADCFE; ctx->h[3] = 0x10325476;
    ctx->h[4] = 0xC3D2E1F0;
    ctx->buf_len = 0;
    ctx->total_len = 0;
}

static void sha1_update(SHA1_Ctx *ctx, const uint8_t *data, uint32_t len)
{
    while (len--)
    {
        ctx->buf[ctx->buf_len++] = *data++;
        ctx->total_len++;
        if (ctx->buf_len == 64)
        {
            sha1_block(ctx, ctx->buf);
            ctx->buf_len = 0;
        }
    }
}

static void sha1_final(SHA1_Ctx *ctx, uint8_t out[20])
{
    uint64_t bit_len = (uint64_t)ctx->total_len * 8;
    uint8_t  i;

    /* 补 0x80，再补 0，直到剩 8 字节放长度 */
    i = 0x80;
    sha1_update(ctx, &i, 1);
    i = 0;
    while (ctx->buf_len != 56)
    {
        sha1_update(ctx, &i, 1);
    }
    for (i = 0; i < 8; i++)
    {
        uint8_t b = (uint8_t)(bit_len >> (56 - i * 8));
        sha1_update(ctx, &b, 1);
    }
    for (i = 0; i < 5; i++)
    {
        out[i * 4]     = (uint8_t)(ctx->h[i] >> 24);
        out[i * 4 + 1] = (uint8_t)(ctx->h[i] >> 16);
        out[i * 4 + 2] = (uint8_t)(ctx->h[i] >> 8);
        out[i * 4 + 3] = (uint8_t)(ctx->h[i]);
    }
}

/* ==================== HMAC-SHA1 ==================== */
static void hmac_sha1(const uint8_t *key, uint16_t key_len,
                      const uint8_t *msg, uint16_t msg_len,
                      uint8_t out[20])
{
    uint8_t  k_pad[64];
    uint8_t  tmp[20];
    uint16_t i;
    SHA1_Ctx ctx;

    /* 密钥超过 64 字节先算一次 SHA1；密钥不足补 0 */
    memset(k_pad, 0, 64);
    if (key_len > 64)
    {
        sha1_init(&ctx);
        sha1_update(&ctx, key, key_len);
        sha1_final(&ctx, k_pad);
    }
    else
    {
        memcpy(k_pad, key, key_len);
    }

    /* 内层：SHA1((key ^ ipad) || msg) */
    sha1_init(&ctx);
    for (i = 0; i < 64; i++) { k_pad[i] ^= 0x36; }
    sha1_update(&ctx, k_pad, 64);
    for (i = 0; i < 64; i++) { k_pad[i] ^= 0x36; }   /* 还原 */
    sha1_update(&ctx, msg, msg_len);
    sha1_final(&ctx, tmp);

    /* 外层：SHA1((key ^ opad) || 内层结果) */
    sha1_init(&ctx);
    for (i = 0; i < 64; i++) { k_pad[i] ^= 0x5C; }
    sha1_update(&ctx, k_pad, 64);
    sha1_update(&ctx, tmp, 20);
    sha1_final(&ctx, out);
}

/* ==================== base64 ==================== */
static const char b64_table[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static uint16_t base64_encode(const uint8_t *in, uint16_t len, char *out)
{
    uint16_t i = 0, o = 0;

    while (i < len)
    {
        uint32_t v = 0;
        uint8_t  n = 0;
        uint8_t  j;

        for (j = 0; j < 3; j++)
        {
            v <<= 8;
            if (i < len) { v |= in[i++]; n++; }
        }
        out[o++] = b64_table[(v >> 18) & 0x3F];
        out[o++] = b64_table[(v >> 12) & 0x3F];
        out[o++] = (n > 1) ? b64_table[(v >> 6) & 0x3F] : '=';
        out[o++] = (n > 2) ? b64_table[v & 0x3F] : '=';
    }
    out[o] = '\0';
    return o;
}

static int8_t b64_val(char c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}

static uint16_t base64_decode(const char *in, uint8_t *out)
{
    uint16_t o = 0;
    uint32_t v = 0;
    uint8_t  n = 0;
    int8_t   d;

    while (*in && *in != '=')
    {
        d = b64_val(*in++);
        if (d < 0) continue;
        v = (v << 6) | (uint8_t)d;
        n++;
        if (n == 4)
        {
            out[o++] = (uint8_t)(v >> 16);
            out[o++] = (uint8_t)(v >> 8);
            out[o++] = (uint8_t)v;
            v = 0; n = 0;
        }
    }
    if (n == 3)      { v <<= 6;  out[o++] = (uint8_t)(v >> 16); out[o++] = (uint8_t)(v >> 8); }
    else if (n == 2) { v <<= 12; out[o++] = (uint8_t)(v >> 16); }
    return o;
}

/* ==================== OneNET token ==================== */
static uint16_t OneNET_Token(char *out, const char *product_id, const char *dev_key_base64, uint32_t expire_time)
{
    char     content[160];
    char     sign_b64[40];
    uint8_t  key[64];
    uint8_t  sign[20];
    uint16_t key_len;

    /* 1. 设备密钥 base64 解码后才是 HMAC 的 key */
    key_len = base64_decode(dev_key_base64, key);

    /* 2. 拼接签名内容：et\nmethod\nres\nversion */
    snprintf(content, sizeof(content), "%lu\nsha1\nproducts/%s\n2018-10-31",
             (unsigned long)expire_time, product_id);

    /* 3. HMAC-SHA1 签名后 base64 */
    hmac_sha1(key, key_len, (const uint8_t *)content, strlen(content), sign);
    base64_encode(sign, 20, sign_b64);

    /* 4. 拼出完整 token */
    return (uint16_t)snprintf(out, 160,
                              "version=2018-10-31&res=products/%s&et=%lu&method=sha1&sign=%s",
                              product_id, (unsigned long)expire_time, sign_b64);
}
