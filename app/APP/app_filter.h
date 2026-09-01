#ifndef __APP_FILTER_H
#define __APP_FILTER_H

#include "app.h"

void  Kalman_Init(KalmanState_t *k, float q, float r);
float Kalman_Update(KalmanState_t *k, float measurement);

#endif /* __APP_FILTER_H */
