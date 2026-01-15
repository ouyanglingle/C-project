#ifndef PARAM_H
#define PARAM_H

#define DEBUG_FLG 1
#define PI 3.1415926535897932384626433832795f
#define INNER_I1 4.0f // 默认齿轮传动比为4，参数不合理改
#define MOTOR_MODE 0  // 0表示1000电机，1表示1500电机

typedef struct
{
    float P_en; // 电机的额定功率
    float n_m;  // 电机的满载转速
    char *name; // 电机的名称
} Motor_Param_t;

typedef struct
{
    int d;      // 轴承的内径
    int D;      // 轴承外径
    int B;      // 轴承宽度
    char *name; // 轴承名称
    float Cr; // 基本额定动载荷
    float C0r; // 额定静载荷
} Bearing_Param_t;

typedef struct
{
    int b;
    int h;
    int L;
} pingjian_t;

pingjian_t pingjian_param(float d12);
float find_closest_endpoint(float a, float b, float x);
float find_closest_in_array_range(float arr[], int size, float x, int *index);
int round_upper(float value_abs);
Motor_Param_t get_nw_param(float P_w, int id);

Bearing_Param_t get_bearing_param(int d);

int round_to_nearest(int value, int nearest);
void bending_fatigue_strength_check(float T_i, float K_Ht, float B2, float m, float n1, float Z1);

#endif
