#ifndef PARAM_H
#define PARAM_H

#define DEBUG_FLG 1
#define PI 3.1415926535897932384626433832795f
#define INNER_I1 4.0f // 默认齿轮传动比为4，参数不合理改

typedef struct
{
    float P_en; // 电机的额定功率
    float n_m;  // 电机的额定转速
    char *name; // 电机的名称
} Motor_Param_t;


typedef struct
{
    int d;      // 轴承的内径
    int D;      // 轴承外径
    int B;      // 轴承宽度
    char *name; // 轴承名称
} Bearing_Param_t;

Motor_Param_t get_nw_param(float P_w);

Bearing_Param_t get_bearing_param(int d);

int round_to_nearest(int value, int nearest);
void bending_fatigue_strength_check(float T_i, float K_Ht, float B2, float m, float n1);

#endif
