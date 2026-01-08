#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "locale.h"
#include "param.h"
#include "stdint.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    // 默认值
    float F = 1500.0f; // 拉力
    float V = 1.2f;    // 速度
    float D = 220.0f;  // 直径
    if (argc == 4)
    {
        F = atof(argv[1]); // 拉力
        V = atof(argv[2]); // 速度
        D = atof(argv[3]); // 直径
        printf("参数传入成功\n");
    }
    else
        printf("未传入参数，将使用默认参数\n");
    // ----电动机的选择----

    printf("拉力F: %.2f N, 速度V: %.2f m/s, 直径D: %.2f mm\n", F, V, D);
    const float inta_1 = 0.99f; // 联轴器效率
    const float inta_2 = 0.99f; // 滚动轴承效率
    const float inta_3 = 0.97f; // 闭式齿轮效率s
    const float inta_v = 0.96f; // V带效率
    const float inta_w = 0.96f; // 工作机效率
    //
    float inta_a = inta_1 * inta_2 * inta_2 * inta_3 * inta_v * inta_w;
    // 卷筒所需功率, 单位kW
    float P_w = (F * V) / 1000.0f;
    // 电动机所需额定功率, 单位kW
    float P_d = round((P_w / inta_a) * 100.0) / 100.0; // 四舍五入到小数点后两位
    // 卷筒轴转速, 单位r/min
    float n_w = 60.0f * 1000.0f * V / (PI * D);
    // 电机选型
    Motor_Param_t motor = get_nw_param(P_d, MOTOR_MODE);
#if DEBUG_FLG
    printf("一、电动机的选择\n");
    printf("卷筒所需功率: %.3f kW\n", P_w);
    printf("电动机所需额定功率: %f kW\n", P_d);
    printf("inta_a: %.3f\n", inta_a);
    printf("卷筒轴转速;: %.3f r/min\n", n_w);
    printf("电机型号: %s, 额定功率P_en: %.3f kW, 满载转速: %.3f r/min\n", motor.name, motor.P_en, motor.n_m);
    printf("\n");
#endif
    // ----分配传动比----
    // 总传动比i_a
    float i_a = 1.0f * motor.n_m / n_w;
    // 齿轮传动比i_1
    float i_1 = INNER_I1;
    // V带传动比i_v
    float i_v = i_a / i_1;
    // ----动力学参数计算----
    // 电机转矩T_d 单位N·mm
    float T_d = 9550.0f * 1000 * (P_d / motor.n_m);
    // 高速轴参数
    float P_i = 1.0f * P_d * inta_v;
    float n_i = 1.0f * motor.n_m / i_v;
    float T_i = 9550.0f * 1000 * (P_i / n_i);
    // 低速轴参数
    float P_ii = 1.0f * P_i * inta_2 * inta_3;
    float n_ii = 1.0f * n_i / i_1;
    float T_ii = 9550.0f * 1000 * (P_ii / n_ii);
    // 卷筒轴参数
    float P_iii = P_ii * inta_2 * inta_1;
    float n_iii = n_ii;
    float T_iii = 9550.0f * 1000 * (P_iii / n_iii);
#if DEBUG_FLG
    printf("二、动力学参数计算\n");
    printf("装置总传动比ia: %.3f\n", i_a);
    printf("齿轮传动比i1: %.3f , V带传动比: %.3f\n", i_1, i_v);
    printf("电机转矩T_d: %.3f N·m\n", T_d);
    printf("高速轴参数 P_i: %.3f kW, n_i: %.3f r/min, T_i: %.3f N·mm\n", P_i, n_i, T_i);
    printf("低速轴参数 P_ii: %.3f kW, n_ii: %.3f r/min, T_ii: %.3f N·mm\n", P_ii, n_ii, T_ii);
    printf("卷筒轴参数 P_iii: %.3f kW, n_iii: %.3f r/min, T_iii: %.3f N·mm\n", P_iii, n_iii, T_iii);
    printf("\n");
#endif
    // ---- V带设计 ----
    float K_A = 1.1f;                                                                                 // 查表得工作情况系数Page219
    float P_ca = K_A * P_d;                                                                           // 查p220发现v在1.0~1.4和D在200~240都应该选用A型V带
    const int d_d1 = 75;                                                                              // 那么小带轮直径默认75mm
    float n_v1 = motor.n_m;                                                                           // 小带轮转速等于电机转速
    float n_v2 = n_i;                                                                                 // 大带轮转速等于高速轴转速
    float d_d2_0 = (n_v1 / n_v2) * d_d1 * (1 - 0.02);                                                 // 计算大带轮的基准直径
    int d_d2 = round_to_nearest(d_d2_0, 5);                                                           // 直径取整
    float V_v = (PI * d_d1 * motor.n_m) / (60 * 1000);                                                // 带速单位m/s,在 5~30 算合适
    float a_v00 = 1.5 * (d_d1 + d_d2);                                                                // 初步计算中心距
    float a_v0 = round_to_nearest(a_v00, 100);                                                        // 最终取整
    float L_v0 = 2 * a_v0 + (PI / 2) * (d_d1 + d_d2) + (d_d2 - d_d1) * (d_d2 - d_d1) / (4 * a_v0);    // 计算V带长度
    float L_d_actual = L_v0;                                                                          // 最终的V带长度
    float Ld_A[12] = {110, 1250, 1430, 1550, 1640, 1750, 1940, 2050, 2200, 2300, 2480, 2700};         // 带长度表
    float Kl_A[12] = {0.91, 0.93, 0.96, 0.98, 0.99, 1.00, 1.02, 1.04, 1.06, 1.07, 1.09, 1.10};        // 带长度系数表
    int index_A = 0;                                                                                  // 带长度索引
    L_d_actual = find_closest_in_array_range(Ld_A, sizeof(Ld_A), L_v0, &index_A);                     // 查表选取V带长度
    float a_v_actual = round(a_v0 + (L_d_actual - L_v0) / 2);                                         // 计算最终V带中心距
    float alpha_v = 180 - (d_d2 - d_d1) / a_v_actual * 57.3;                                          // 验算小带轮包角
    float P0_A_Select[4] = {950, 1200, 1450, 1600};                                                   // A型V带的基本额定功率表的选择
    float P0_A[4] = {0.51, 0.60, 0.68, 0.73};                                                         // A型V带的基本额定功率表的结果
    int index_P0 = 0;                                                                                 // 基本额定功率索引
    find_closest_in_array_range(P0_A_Select, sizeof(P0_A_Select), n_v1, &index_P0);                   // 查表选取基本额定功率的索引
    float P0_actual = P0_A[index_P0];                                                                 // 通过索引基本额定功率
    float i_v_actual = d_d2 / (d_d1 * (1 - 0.02));                                                    // 实际V带传动比
    float delta_P0 = -1;                                                                              // 额定功率增量
    int delta_P0_index = -1;                                                                          // 额定功率增量索引
    if (i_v_actual > 1.51 && i_v_actual < 1.99)                                                       //
    {                                                                                                 //
        float delta_P0_Select[5] = {800, 980, 1200, 1460, 1600};                                      // 基本额定功率增量表头
        float delta_P0_Values[5] = {0.09, 0.10, 0.13, 0.15, 0.17};                                    // 基本额定功率增量表值
        find_closest_in_array_range(delta_P0_Select, sizeof(delta_P0_Select), n_v1, &delta_P0_index); // 查表选取V带基本额定功率增量的索引
        delta_P0 = delta_P0_Values[delta_P0_index];                                                   // 通过索引确定基本额定功率增量
    }
    else if (i_v_actual > 1.9)                                                                        // 带速大于等于2
    {                                                                                                 //
        float delta_P0_Select[5] = {800, 980, 1200, 1460, 1600};                                      // 基本额定功率增量表头
        float delta_P0_Values[5] = {0.10, 0.11, 0.15, 0.17, 0.19};                                    // 基本额定功率增量表值
        find_closest_in_array_range(delta_P0_Select, sizeof(delta_P0_Select), n_v1, &delta_P0_index); // 查表选取V带基本额定功率增量的索引
        delta_P0 = delta_P0_Values[delta_P0_index];                                                   // 通过索引确定基本额定功率增量
    }
    float K_a_A[5] = {0.89, 0.92, 0.95, 0.98, 1.00};                                              // 包角修正系数取值表
    float alpha_v_Select[5] = {140, 150, 160, 170, 180};                                          // 包角修正系数取值表头
    int alpha_v_index = -1;                                                                       // 包角修正系数索引
    find_closest_in_array_range(alpha_v_Select, sizeof(alpha_v_Select), alpha_v, &alpha_v_index); // 查表选取包角修正系数的索引
    float K_a_actual = K_a_A[alpha_v_index];                                                      // 通过索引包角修正系数
    float z_v0 = P_ca / ((P0_actual + delta_P0) * K_a_actual * Kl_A[index_A]);                    // 初步计算V带根数Z
    int z_v_actual = round_upper(z_v0);                                                           // 最终upper取整为V带根数Z
    const float q_v = 0.105f;                                                                     // A型V带的单位长度质量kg/m
    float F_v0 = (500 * P_ca / (z_v_actual * V_v)) * ((2.5 / K_a_actual) - 1) + q_v * V_v * V_v;  // 单根V带的初拉力计算
    float F_Q = 2 * z_v_actual * F_v0 * sin(alpha_v / 2 * PI / 180);                              // 惊天压轴力

#if DEBUG_FLG
    printf("----V带设计----\n");
    printf("已知大带轮宽度: 50 mm\n");
    printf("求出计算功率Pc = %.3f kW\n", P_ca);
    printf("查p220发现v在1.0~1.4和D在200~240都应该选用A型V带\n");
    printf("小带轮直径默认d_d1: %d mm\n", d_d1);
    printf("p222初步计算大带轮直径d_d2: %.3f mm\n", d_d2_0);
    printf("最终选取大带轮直径d_d2: %d mm, 误差在5%% 之内，故允许\n", d_d2);
    printf("带速V_v: %.3f m/s, %s 5~30m/s范围内\n", V_v, V_v >= 5 && V_v <= 30 ? "在" : "不在");
    printf("初步计算中心距a_v: %.3f mm, 初步取整 %.2f mm, %s 0.7(d1+d2)<a0<2(d1+d2)\n", a_v00, a_v0, (a_v0 > 0.7 * (d_d1 + d_d2) && a_v0 < 2 * (d_d1 + d_d2)) ? "符合" : "不符合");
    printf("初步计算V带长度L_v0: %.3f mm\n", L_v0);
    printf("查表选取V带长度Ld: %.3f mm, 索引为%d\n", L_d_actual, index_A + 1);
    printf("最终计算中心距a_v: %.3f mm (已经四舍五入去除小数部分)\n", a_v_actual);
    printf("验算小带轮包角alpha_v: %.3f °, %s 大于120°\n", alpha_v, alpha_v > 120 ? "符合" : "不符合");
    printf("查表选取基本额定功率P0_A: %.3f kW, 索引为%d\n", P0_actual, index_P0 + 1);
    printf("(本条记录可忽略)计算得V带传动比i_v: %.3f, 误差在5%% 之内，故允许\n", i_v_actual);
    printf("查表选取V带基本额定功率增量delta_P0: %.3f kW, 索引为%d\n", delta_P0, delta_P0_index + 1);
    printf("查表选取当包角为 %.2f 时的包角修正系数K_a: %.3f, 索引为%d\n", alpha_v, K_a_actual, alpha_v_index + 1);
    printf("初步计算得V带根数z: %.3f, 最终upper取整为 %d \n", z_v0, z_v_actual);
    printf("A型V带的单位长度质量 %.2f kg/m", q_v);
    printf("单根V带的初拉力F_v: %.3f N\n", F_v0);
    printf("压轴力F_Q: %.3f N\n", F_Q);

    printf("\n");
#endif

        // ---- 齿轮设计 ----
        const int16_t xigema_H1 = 655;
        const int16_t xigema_H2 = 559;
        const int16_t xigema_F1 = 476;
        const int16_t xigema_F2 = 408;
        const float K_Ht = 1.50f;             // 载荷系数
        const float φd = 0.8;                 // 齿宽系数
        const float Z_H = 2.49;               //
        const float Z_E = 188.9;              // 区域系数
        const float cos20 = 0.9396926207859f; // Cos(20°)结果
        const float tan20 = 0.3639702342662f; // tan(20°)结果
        const uint16_t Z1 = 20;               // 小齿轮齿数默认20
        float Z2 = Z1 * i_1;                  // 大齿轮齿数
        //
        float d1 = 2.32f * pow((K_Ht * T_i / φd) * ((i_1 + 1) / i_1) * (Z_E / xigema_H2) * (Z_E / xigema_H2), 1.0f / 3.0f);
        // 计算模数
        float m_1 = d1 / Z1;
        float m = 0.0f;
        if (m_1 > 2 && m_1 < 2.5)
            m = 2.5f;
        else if (m_1 > 1.5 && m_1 < 2)
            m = 2.0f;
        else if (m_1 > 2.5 && m_1 < 3)
            m = 3.0f;
        else if (round(m_1) < m_1 && m_1 > 3 && round(m_1) < 3.5)
            m = 3.0f;
        // 计算中心距
        float a = (Z1 + Z2) * m / 2.0f;
        float d_1 = m * Z1; // 小齿轮分度圆
        float d_2 = m * Z2; // 大齿轮分度圆
        // 计算齿宽
        float b = φd * d1;
        int16_t B2 = round_to_nearest((int16_t)b, 5); // 大齿轮宽度
        int16_t B1 = B2 + 5;                          // 小齿轮宽度
        // 计算齿轮其它几何参数
        float h_a = m * 1; // 齿顶高，标准齿的ha*为1
        // // 计算接触疲劳强度用重合度系数Zε
        // float alpha_a1 = acosf((Z1 * cos20) / (Z1 + 2)); // * 180.0f / PI;
        // float alpha_a2 = acosf((Z2 * cos20) / (Z2 + 2)); // * 180.0f / PI;
        // 计算
    #if DEBUG_FLG
        printf("四、齿轮设计\n");
        printf("小齿轮40MnB调质, 大齿轮ZG35SiMn调质\n");
        printf("小齿轮齿数默认 %d,\n", Z1);
        printf("初次计算小齿轮直径D1: %.3f mm, 计算得模数为: %.2f, 最终模数为: %.2fmm\n", d1, m_1, m);
        printf("最终小齿轮分度圆：%.2f, 大齿轮分度圆: %.2f\n", d_1, d_2);
        printf("中心距a为: %.2f mm\n", a);
        printf("通过公式b = φd * d1计算的齿宽为b: %.2f mm\n", b);
        printf("小齿轮齿宽B1: %d mm, 大齿轮齿宽B2: %d mm\n", B1, B2);
        printf("剩下的自己算吧\n");
        printf("\n");
    #endif
        bending_fatigue_strength_check(T_i, K_Ht, B2, m, n_i, Z1); // 齿根弯曲疲劳强度校核

        // ---- 轴的设计与校核 ----
        // 选用45调质材料，假装按照课本算到危险截面的当量弯矩Me
        const uint8_t A0 = 110;
        float d0 = A0 * pow(P_i / n_i, 1.0f / 3.0f);
        // 安装平键，所以加大5%
        float d = d0 * 1.05f;
        // 四舍五入最终选取
        d = round_to_nearest(d, 2);
        // 选用课本方案二, 取定位轴肩h=2mm
        const uint8_t h = 2;
        float d12 = d;
        float d23 = d12 + 2 * h;
        if (d23 > d12 && d23 < 25) // 教材表5.2
            d23 = 25;
        else if (d23 > 25 && d23 < 28)
            d23 = 28;
        else if (d23 > 28 && d23 < 30)
            d23 = 30;
        d23 = round(d23);

        float $d34 = d23 + 2 * h;
        float d34 = $d34;
        if ($d34 < 30)
            d34 = 30;
        else if ($d34 > 30 && $d34 < 35)
            d34 = 35;
        d34 = round(d34);
        float d45 = round(d34 + 2 * h);
        float d4 = d45;
        if (d4 > 31.5 && d4 < 33.5)
            d4 = 33.5;
        else if (d4 > 33.5 && d4 < 35.5)
            d4 = 35.5;
        else if (d4 > 35.5 && d4 < 37.5)
            d4 = 37.5;
        d4 = round(d4);

        float d56 = d4 + 2 * h;
        float d5 = round(d56);
        if (d5 > 35.5 && d5 < 37.5)
            d5 = 37.5;
        else if (d5 > 37.5 && d5 < 40)
            d5 = 40;
        else if (d5 > 40 && d5 < 42.5)
            d5 = 42.5;
        float d6 = round(d34);

    #if DEBUG_FLG
        printf("五、轴的初步设计与校核\n");
        printf("轴的材质为45调质\n");
        printf("----高速轴直径设计----\n");
        printf("轴的设计直径d: %.2f mm\n", d);
        printf("轴的直径d1: %.2f mm\n", d12);
        printf("轴的直径d2: %.2f mm\n", d23);
        printf("轴的直径d3理论为: %.2f mm, 结合轴承选型应为%.2fmm, 实际上用表5.2就有国标轴承能用\n", $d34, d34); // 实际上用表5.2就有国标轴承能用
        printf("轴的直径d4: %.2f mm\n", d4);
        printf("轴的直径d5: %.2f mm\n", d5);
        printf("轴的直径d6: %.2f mm\n", d6);
        printf("\n");
    #endif
        // ----箱体设计----
        // 机座壁厚
        float delta = 0.025 * a + 1;
        delta = delta < 8 ? 8 : delta;
        // 机盖壁厚
        float delta_1 = 0.02 * a + 1;
        delta_1 = delta_1 < 8 ? 8 : delta_1;
        float b0 = 1.5 * delta;            // 机座凸缘厚度
        float b1 = 1.5 * delta_1;          // 机盖凸缘厚度
        float b2 = 2.5 * delta;            // 机座底凸缘厚度
        uint8_t d_f = 0.036 * a + 12;      // 地脚螺栓直径
        uint8_t n = a <= 250 ? 4 : 6;      // 地脚螺栓数目
        uint8_t d1_1 = 0.75 * d_f;         // 轴承旁连接螺栓直径
        uint8_t d3_3 = 0.45 * d_f;         // 轴承端盖螺钉直径
        float m0_0 = 0.85 * delta;         // 机座肋厚
        uint8_t e = 1.2 * d3_3;            // 轴承端盖凸缘厚度
        float Delta1 = round(1.2 * delta); // 大齿轮顶圆与内机壁距离，教材说大于1.2倍delta即可,我取10
        float Delta2 = 10;                 // 齿轮端面与内机壁距离,教材说大于delta同时一般取>=10
        uint8_t Delta3 = 4;                // 教材使用了脂润滑，△3在3~5只之间即可
        uint8_t c1 = 0, c2 = 0, D0 = 0;    // df,d1,d2到外机壁距离 ; df,d2到凸缘边缘距离 ; 未知
        switch (d1_1)
        {
        case 8:
            c1 = 13, c2 = 11, D0 = 18;
            break;
        case 10:
            c1 = 16, c2 = 14, D0 = 22;
            break;
        case 12:
            c1 = 18, c2 = 16, D0 = 26;
            break;
        case 14:
            c1 = 20, c2 = 18, D0 = 30;
            break;
        default:
            break;
        }
        uint16_t B = delta + c1 + c2 + 5; // 轴承座宽度尺寸

    #if DEBUG_FLG
        printf("六、箱体设计\n");
        printf("机座壁厚: %.3f mm, 机盖壁厚: %.3f mm\n", delta, delta_1);
        printf("机座凸缘厚度b: %.3f mm, 机盖凸缘厚度b1: %.3f mm\n", b0, b1);
        printf("机座底凸缘厚度b2: %.3f mm\n", b2);
        printf("地脚螺栓直径df: %d mm, 地脚螺栓数目n: %d, 轴承旁连接螺栓直径d1:%d\n", d_f, n, d1_1);
        printf("机座肋厚m0: %.3f mm\n", m0_0);
        printf("轴承端盖螺钉直径d3: %d mm, 轴承端盖凸缘厚度:e %d mm\n", d3_3, e);
        printf("大齿轮顶圆与内机壁距离△1: %.2f mm, 齿轮端面与内机壁距离△2: %.2f mm, 脂润滑△3为%d\n", Delta1, Delta2, Delta3);
        printf("df,d1,d2到外机壁距离c1: %dmm, df,d2到凸缘边缘距离c2: %dmm, D0: %dmm\n", c1, c2, D0);
        printf("轴承座宽度尺寸B: %d mm\n", B);
        printf("\n");
    #endif
        // ----高速轴长度设计----
        const uint8_t L_B = 25; // 讲义的图上有25mm
        const uint8_t W_V = 50; // 讲义的带轮宽50mm
        // 小齿轮所受的圆周力
        float F_t1 = 2 * T_i / d_1;
        // 小齿轮所受径向力
        float F_r1 = F_t1 * tan20;
        // 轴承选型
        Bearing_Param_t bearing_High = get_bearing_param((int)d34);

        float L1 = W_V - 2;
        float L2_0 = L_B + e + m0_0;
        float L2 = L2_0;
        if (round(L2_0) < L2_0)
            L2 = round(L2_0) + 1;
        else
            L2 = round(L2_0);
        float L3 = bearing_High.B + Delta3 + Delta2 + 3;
        float L4 = B1 - 3; // 用齿轮宽度减去2~3
        float L5 = 6;
        float L6 = bearing_High.B + Delta3 + 3;

    #if DEBUG_FLG
        printf("----高速轴长度设计----\n");
        printf("小齿轮所受的圆周力为: %.2f\n", F_t1);
        printf("小齿轮所受的径向力为: %.2f\n", F_r1);
        printf("轴承选型: %s, 内径:%d, 外径%d, 宽度:%d\n", bearing_High.name, bearing_High.d, bearing_High.D, bearing_High.B);
        printf("第一段长L1: %.2f\n", L1);
        printf("第二段长L2: %.2f\n", L2);
        printf("第三段长L3: %.2f\n", L3);
        printf("第四段长L4: %.2f\n", L4);
        printf("第五段长L5和轴环宽度相等: %.2f\n", L5);
        printf("第六段长L6: %.2f\n", L6);
        printf("\n");
    #endif
        // ----低速轴直径设计----
        const float h_L = 1.5;
        float d0_L0 = A0 * pow(P_ii / n_ii, 1.0f / 3.0f);
        uint8_t d0_L = round_to_nearest(d0_L0, 2);
        uint8_t d12_L = d0_L;
        uint8_t d23_L = 0;
        float d23_L0 = d12_L + 2 * h_L;
        if (d23_L0 > d12_L && d23_L0 < 37.5) // 教材表5.2
            d23_L = round_to_nearest(d23_L0, 2);

        uint8_t d34_L, d34_L0;
        d34_L0 = d23_L + 2 * h_L;
        if (d34_L0 > 30 && d34_L0 <= 35)
            d34_L = 35;
        else if (d34_L0 > 35 && d34_L0 <= 40)
            d34_L = 40;

        Bearing_Param_t bearing_Low = get_bearing_param(d34_L);

        float d45_L0 = d34_L + 2 * h_L;
        float d45_L = round_to_nearest(d45_L0, 2);
        /*
        if (d45_L0 > 37 && d45_L0 < 40)
            d45_L = 40;
        else if (d45_L0 > 40 && d45_L0 < 42)
            d45_L = 42;
        else if (d45_L0 > 42 && d45_L0 < 45)
            d45_L = 45;
           */
        float d56_L0 = d45_L + 2 * h_L;
        float d56_L = round_to_nearest(d56_L0, 2);
        float d67_L = d34_L;
        // ---- 低速轴长度设计----

    #if DEBUG_FLG
        printf("----低速轴直径设计----\n");
        printf("取轴肩h=1.2\n");
        printf("按扭矩估算最小直径为dmin：%.2fmm, 取整为d1: %d mm\n", d0_L0, d0_L);
        printf("计算得d2：%.2f mm, 取整为d2: %d mm\n", d23_L0, d23_L);
        printf("计算得d3：%d mm, 结合轴承选型应取d3: %d mm\n", d34_L0, d34_L);
        printf("计算得d4：%.2f mm, 取整为d4: %.2f mm\n", d45_L0, d45_L);
        printf("计算得d5：%.2f mm, 取整为d5: %.2f mm\n", d56_L0, d56_L);
        printf("计算得d6和d3相同，为%.2f mm\n", d67_L);
        printf("轴承选型: %s, 内径:%d, 外径%d, 宽度:%d\n", bearing_Low.name, bearing_Low.d, bearing_Low.D, bearing_Low.B);
        printf("\n");
    #endif

    // char input;
    // printf("按Q键退出程序...\n");
    // while (1)
    // {
    //     printf("\r等待输入... (按Q退出) ");
    //     fflush(stdout);

    //     if (scanf("%c", &input))
    //         if (input == 'q' || input == 'Q')
    //             break;
    // }

    return 0;
}
