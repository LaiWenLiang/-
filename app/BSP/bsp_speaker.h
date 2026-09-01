#ifndef __BSP_SPEAKER_H
#define __BSP_SPEAKER_H

#include "bsp.h"

/* 语音播报编号（任务通知值直接使用） */
typedef enum
{
    VOICE_NONE = 0,
    VOICE_WELCOME,
    VOICE_REMOTE_MODE,
    VOICE_FOLLOW_MODE,
    VOICE_SPEED_LOW,
    VOICE_SPEED_HIGH,
    VOICE_REMOTE_LOW_BATTERY,
    VOICE_CAR_LOW_BATTERY,
} VoiceId_t;

void BSP_Speaker_Init(void);
void BSP_Speaker_Play(VoiceId_t id);

#endif /* __BSP_SPEAKER_H */
