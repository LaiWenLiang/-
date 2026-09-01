#include "app_avoid.h"

/**************************************************************************
 * 多级避障状态机：
 *   任意一路 < 20cm            -> 急停 STOP_MOTOR
 *   连续3个周期 < 50cm          -> 进入警戒模式：
 *       四路全堵(25/30cm)      -> 原地掉头（朝空旷侧，方向锁死）
 *       < 22~25cm              -> 左后/右后倒车 10 周期
 *       < 70~80cm              -> 原地转向避让 10 周期
 *   连续3个周期 >= 50cm         -> 退出警戒，恢复跟随
 **************************************************************************/
TurnDirection_t Avoid_Judge(const TOF_Distance_t *tof)
{
    static int car_warn_mode = 0, car_number = 0;
    static int turn_number = 0, turn_lock = 0, move_time = 0;
    static TurnDirection_t car_move = NO_ACTION;

    int t_l  = tof->left_mm;
    int t_ml = tof->middle_l_mm;
    int t_mr = tof->middle_r_mm;
    int t_r  = tof->right_mm;
    int barrier  = 0;
    int car_status = (t_l + t_ml) - (t_mr + t_r);   /* 左右两侧空旷度差 */

    /* 最高优先级：急停 */
    if (t_l < AVOID_ESTOP_CM || t_ml < AVOID_ESTOP_CM ||
        t_mr < AVOID_ESTOP_CM || t_r < AVOID_ESTOP_CM)
    {
        return STOP_MOTOR;
    }

    /* 警戒区计数：连续确认防抖动 */
    if (t_l < AVOID_WARN_CM || t_ml < AVOID_WARN_CM ||
        t_mr < AVOID_WARN_CM || t_r < AVOID_WARN_CM)
    {
        if (car_number >= 0) { car_number = 0; }
        car_number--;
    }
    else
    {
        if (car_number <= 0) { car_number = 0; }
        car_number++;
    }

    if (car_number <= -3)      { car_number = 0; car_warn_mode = 1; }   /* 进入警戒 */
    else if (car_number >= 3 && car_move == NO_ACTION)
    {
        car_number = 0; car_warn_mode = 0;                              /* 退出警戒 */
    }

    /* 是否需要掉头：四路全部被近距离堵死 */
    if (t_l < 25)  barrier++;
    if (t_ml < 30) barrier++;
    if (t_mr < 30) barrier++;
    if (t_r < 25)  barrier++;
    if (barrier == 4)     { turn_number = 1; }
    else if (barrier <= 0) { turn_number = 0; turn_lock = 0; }

    if (car_warn_mode == 1 && turn_lock == 0)
    {
        if (turn_number == 1)   /* 掉头 */
        {
            car_move = (car_status < 0) ? SPOT_LEFT_TURN : SPOT_RIGHT_TURN;
        }
        else if (t_l < AVOID_BACK_OUT_CM || t_ml < AVOID_BACK_MID_CM ||
                 t_mr < AVOID_BACK_MID_CM || t_r < AVOID_BACK_OUT_CM)
        {
            car_move = (car_status < 0) ? LEFT_BACKWARD : RIGHT_BACKWARD;
            move_time = 10;
        }
        else if (t_l < AVOID_TURN_OUT_CM || t_ml < AVOID_TURN_MID_CM ||
                 t_mr < AVOID_TURN_MID_CM || t_r < AVOID_TURN_OUT_CM)
        {
            car_move = (car_status < 0) ? SPOT_LEFT_TURN : SPOT_RIGHT_TURN;
            move_time = 10;
        }
        else if (move_time <= 0)
        {
            car_move = NO_ACTION;   /* 障碍解除，恢复跟随 */
        }
    }

    if (move_time >= 0) { move_time--; }

    if (car_move == SPOT_LEFT_TURN || car_move == SPOT_RIGHT_TURN)
    {
        turn_lock = 1;   /* 掉头期间锁死方向 */
    }

    return car_move;
}
