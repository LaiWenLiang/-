#ifndef __BSP_TOKEN_H
#define __BSP_TOKEN_H

#include <stdint.h>

/***************************************************************************************************
 * OneNET 鉴权 token 生成
 * token 格式：version=2018-10-31&res=products/产品ID&et=过期时间戳&method=sha1&sign=签名
 * 签名 = base64( HMAC-SHA1( key=base64解码(设备密钥), 内容=et\nmethod\nres\nversion ) )
 ***************************************************************************************************/

/* 生成 OneNET token，写入 out（至少 160 字节），返回 token 长度 */
uint16_t OneNET_Token(char *out, const char *product_id, const char *dev_key_base64, uint32_t expire_time);

#endif /* __BSP_TOKEN_H */
