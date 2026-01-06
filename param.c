#include "param.h"
#include "math.h"

Motor_Param_t motor_list[] = {
    {2.2f, 960.0f, "Y112M-6"},
    {3.0f, 960.0f, "Y132S-6"},
    {4.0f, 960.0f, "Y132M1-6"},
    {0, 0, "ERR"},
};

/// @brief 获取电机参数
/// @param P_w 卷筒所需功率
/// @return Motor_Param_t电机参数结构体
Motor_Param_t get_nw_param(float P_w)
{
    Motor_Param_t param = {0};
    if (P_w > 1.5f && P_w < 2.2f)
    {
        param = motor_list[0];
    }
    else if (P_w > 2.2f && P_w < 3.0f)
    {
        param = motor_list[1];
    }
    else if (P_w > 3.0f && P_w < 4.0f)
    {
        param = motor_list[2];
    }
    else
    {
        param = motor_list[3];
    }
    return param;
}

Bearing_Param_t bearings_list[] = {
    {20, 42, 12, "6004 or 7004AC"},
    {25, 47, 12, "6005 or 7005AC"},
    {30, 55, 13, "6006 or 7006AC"},
    {35, 62, 14, "6007 or 7007AC"},
    {40, 68, 15, "6008 or 7008AC"},
    {45, 75, 16, "6009 or 7009AC"},
    {50, 80, 16, "6010 or 7010AC"},
};

Bearing_Param_t get_bearing_param(int d)
{
    Bearing_Param_t bearing = {0};
    switch (d)
    {
    case 20:
        bearing = bearings_list[0];
        break;
    case 25:
        bearing = bearings_list[1];
        break;
    case 30:
        bearing = bearings_list[2];
        break;
    case 35:
        bearing = bearings_list[3];
        break;
    case 40:
        bearing = bearings_list[4];
        break;
    case 45:
        bearing = bearings_list[5];
        break;
    case 50:
        bearing = bearings_list[6];
        break;
    }

    return bearing;
}

// 将value四舍五入到nearest的倍数
int round_to_nearest(int value, int nearest)
{
    return (int)(round((double)value / nearest) * nearest);
}
