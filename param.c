#include "param.h"
#include "math.h"
#include "stdio.h"

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
    {20, 47, 14, "6204"},
    {25, 52, 15, "6205"},
    {30, 62, 16, "6206"},
    {35, 72, 17, "6207"},
    {40, 80, 18, "6208"},
    {45, 85, 19, "6209"},
    {50, 90, 20, "6210"},
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

/// @brief 齿根弯曲疲劳强度校核
/// @param T_i
/// @param T_ii
/// @param K_Ht 载荷系数
/// @param B2 大齿轮齿宽
/// @param m 模数
/// @param n1 高速轴的转速
void bending_fatigue_strength_check(float T_i, float K_Ht, float B2, float m, float n1, float Z1)
{
    const int xigema_F1_Allowable = 476; // 单位MPa
    const int xigema_f2_Allowable = 408; // 单位MPa

    float Y_Fa1 = 2.94f; // 小齿轮齿形系数，小齿轮20齿，和课本取值是不一样的
    float Y_Sa1 = 1.56f; // 小齿轮应力修正系数

    float Y_Fa2 = 2.25f; // 大齿轮齿形系数, 大齿轮80齿，
    float Y_Sa2 = 1.76f; // 大齿轮应力修正系数

    float xigema_F1 = (2 * K_Ht * T_i * Y_Fa1 * Y_Sa1) / (B2 * m * m * Z1);
    float xigema_F2 = xigema_F1 * (Y_Fa2 * Y_Sa2) / (Y_Fa1 * Y_Sa1);

    // 齿轮的圆周速度
    float v1 = (PI * m * Z1 * n1) / (60 * 1000.0f);

    printf("----按齿根弯曲疲劳强度校核----\n");
    printf("σ_F1 = %.3f MPa %c [σ_F1] = %d\n", xigema_F1, xigema_F1 < xigema_F1_Allowable ? '<' : '>', xigema_F1_Allowable);
    printf("σ_F2 = %.3f MPa %c [σ_F2] = %d\n", xigema_F2, xigema_F2 < xigema_f2_Allowable ? '<' : '>', xigema_f2_Allowable);
    printf("齿轮圆周速度v1 = %.3f m/s %c 八级直齿圆柱轮的6m/s\n", v1, v1 < 6.0f ? '<' : '>');
    printf("\n");
}

/// @brief 轴的