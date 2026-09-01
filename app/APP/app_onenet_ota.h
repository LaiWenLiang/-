#ifndef __APP_ONENET_OTA_H
#define __APP_ONENET_OTA_H

/***************************************************************************************************
 * OneNET 平台信息（在 OneNET 后台"设备详情"页能看到）
 ***************************************************************************************************/
#define ONENET_PRODUCT_ID   "JH0Fn7bV02"                                       /* 产品ID */
#define ONENET_DEVICE_ID    "001"                                              /* 设备名称 */
#define ONENET_DEVICE_KEY   "......"    /* 设备密钥（上传GitHub前已抹除，自己编译时填回真实密钥） */
#define ONENET_MANUF        "other"      /* 建升级包时选的厂商名称，保持后台一致 */
#define ONENET_MODEL        "other"      /* 建升级包时选的模组型号，保持后台一致 */

/* WiFi 配置（改成你实际环境的 2.4G WiFi） */
#define WIFI_SSID           "MyHomeWiFi"
#define WIFI_PASSWORD       "12345678"

/* 当前 APP 固件版本号（平台上创建升级任务时按它区分新旧） */
#define APP_VERSION         "1.0.0"

void App_OnenetOta_Task(void *param);

#endif /* __APP_ONENET_OTA_H */
