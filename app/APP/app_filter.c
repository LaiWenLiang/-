#include "app_filter.h"

void Kalman_Init(KalmanState_t *k, float q, float r)
{
    k->x = 0;
    k->p = 1.0f;
    k->q = q;
    k->r = r;
}

float Kalman_Update(KalmanState_t *k, float measurement)
{
    float kg;

    k->p = k->p + k->q;
    kg   = k->p / (k->p + k->r);
    k->x = k->x + kg * (measurement - k->x);
    k->p = (1.0f - kg) * k->p;
    return k->x;
}
