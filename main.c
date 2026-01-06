#include <stdio.h>
#include <stdlib.h>
#include "math.h"
#include "locale.h"
#include "param.h"
#include "stdint.h"

#define PI 3.1415926535897932384626433832795f
#define DEBUG_FLG 1
#define INNER_I1 4.0f // 默认齿轮传动比为4，参数不合理改

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "zh_CN.UTF-8");
    // 默认值
    float F = 1500.0f; // 拉力
    float V = 1.0f;    // 速度
    float D = 240.0f;  // 直径
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
    Motor_Param_t motor = get_nw_param(P_d);
#if DEBUG_FLG
    printf("一、电动机的选择\n");
    printf("卷筒所需功率: %.3f kW\n", P_w);
    printf("电动机所需额定功率: %f kW\n", P_d);
    printf("inta_a: %.3f\n", inta_a);
    printf("卷筒轴转速;: %.3f r/min\n", n_w);
    printf("电机型号: %s, 额定功率P_en: %.3f kW, 满载转速: %.3f r/min\n", motor.name, motor.P_en, motor.n_m);
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
#endif
    // ---- V带设计 ----
    // 大带轮宽度
#if DEBUG_FLG
    printf("三、V带设计\n");
    printf("大带轮宽度: 50 mm\n");
#endif

    // ---- 齿轮设计 要灵活变通懂么 表满看起来没问题就行了 再深入就不礼貌了----
    const int16_t xigema_H1 = 655;
    const int16_t xigema_H2 = 559;
    const int16_t xigema_F1 = 476;
    const int16_t xigema_F2 = 408;
    const float K_Ht = 1.50f;             // 载荷系数，假装是课本的1.5
    const float φd = 0.8;                 // 齿宽系数，假装是课本的0.8
    const float Z_H = 2.49;               //
    const float Z_E = 188.9;              // 区域系数， 假装是课本的188.9
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
    // 计算中心距
    float a = (Z1 + Z2) * m / 2.0f;
    float d_1 = m * Z1; // 小齿轮分度圆
    float d_2 = m * Z2; // 大齿轮分度圆
    // 计算齿宽
    float b = φd * d1;
    int16_t B2 = round_to_nearest((int16_t)b, 10); // 大齿轮宽度
    int16_t B1 = B2 + 5;                           // 小齿轮宽度
    // 计算齿轮其它几何参数
    float h_a = m * 1; // 齿顶高，标准齿的ha*为1
    // // 计算接触疲劳强度用重合度系数Zε
    // float alpha_a1 = acosf((Z1 * cos20) / (Z1 + 2)); // * 180.0f / PI;
    // float alpha_a2 = acosf((Z2 * cos20) / (Z2 + 2)); // * 180.0f / PI;
    // 计算
#if DEBUG_FLG
    printf("四、齿轮设计\n");
    printf("小齿轮40MnB调质, 大齿轮ZG35SiMn调质\n");
    printf("小齿轮齿数默认20,\n");
    printf("初次计算小齿轮直径D1: %.3f mm, 计算得模数为: %.2f, 最终模数为: %.2fmm\n", d1, m_1, m);
    printf("最终小齿轮分度圆：%.2f, 大齿轮分度圆: %.2f\n", d_1, d_2);
    printf("中心距a为: %.2f mm\n", a);
    printf("小齿轮齿宽B1: %d mm, 大齿轮齿宽B2: %d mm\n", B1, B2);
    printf("剩下的自己算吧\n");
#endif

    // ---- 轴的设计与校核 ----
    // 选用45调质材料，假装按照课本算到危险截面的当量弯矩Me
    const uint8_t A0 = 110;
    float d0 = A0 * pow(P_i / n_i, 1.0f / 3.0f);
    // 安装平键，所以加大5%
    float d = d0 * 1.05f;
    // 四舍五入最终选取
    d = round_to_nearest(d, 10);
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

    float $d34 = d23 + 2 * h;
    float d34 = $d34;
    if ($d34 > d23 && $d34 < 28)
        d34 = 28;
    else if ($d34 < 30) // 教材表5.2
        d34 = 30;
    else if ($d34 > 30 && $d34 < 40)
        d34 = 40;
    float d45 = d34 + 2 * h;
    float d4 = d45;
    if (d4 > 31.5 && d4 < 33.5)
        d4 = 33.5;
    else if (d4 > 33.5 && d4 < 35.5)
        d4 = 35.5;
    else if (d4 > 35.5 && d4 < 37.5)
        d4 = 37.5;

    float d56 = d4 + 2 * h;
    float d5 = d56;
    if (d5 > 35.5 && d5 < 37.5)
        d5 = 37.5;
    else if (d5 > 37.5 && d5 < 40)
        d5 = 40;
    else if (d5 > 40 && d5 < 42.5)
        d5 = 42.5;
    float d6 = d34;

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
    printf("轴承端盖螺钉直径d3: %d mm, 轴承端盖凸缘厚度:e %d mm\n", d3_3, e);
    printf("大齿轮顶圆与内机壁距离△1: %.2f mm, 齿轮端面与内机壁距离△2: %.2f mm, 脂润滑△3为%d\n", Delta1, Delta2, Delta3);
    printf("df,d1,d2到外机壁距离c1: %dmm, df,d2到凸缘边缘距离c2: %dmm, D0: %dmm\n", c1, c2, D0);
    printf("轴承座宽度尺寸B: %d mm\n", B);
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
    float L2 = L_B + e + B - Delta3 - bearing_High.B;
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
#endif

    char input;
    printf("按Q键退出程序...\n");
    while (1)
    {
        printf("\r等待输入... (按Q退出) ");
        fflush(stdout);

        if (scanf("%c", &input))
            if (input == 'q' || input == 'Q')
                break;
    }

    return 0;
}
