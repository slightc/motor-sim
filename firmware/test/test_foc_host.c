/* -*- coding: utf-8 -*-
 * test_foc_host.c —— foc.c 的 PC 原生回归测试
 *
 * 把固件算法 foc.c 在主机上编译，逐点比对仿真 core 生成的黄金值（golden.h）。
 * 验证"固件 FOC == 仿真 FOC（同输入同输出）"，即 docs/05 的算法移植闭环。
 *
 * 构建运行：见 firmware/test/run_host_test.sh
 */
#include <stdio.h>
#include <math.h>
#include "foc.h"
#include "golden.h"

static int fails = 0;
static void chk(const char *what, float got, float exp, float tol) {
    float e = fabsf(got - exp);
    if (e > tol) {
        printf("  [FAIL] %-22s got=%.6f exp=%.6f |err|=%.2e (tol=%.0e)\n",
               what, got, exp, e, tol);
        fails++;
    }
}

int main(void) {
    const float TOL = 2e-4f;   /* float vs double 累积误差容差 */

    printf("== Clarke/Park 变换 ==\n");
    for (int i = 0; i < GOLDEN_XFORM_N; ++i) {
        xform_t g = GOLDEN_XFORM[i];
        float al, be, d, q;
        foc_clarke(g.a, g.b, g.c, &al, &be);
        foc_park(al, be, g.th, &d, &q);
        chk("clarke.alpha", al, g.alpha, TOL);
        chk("clarke.beta",  be, g.beta,  TOL);
        chk("park.d", d, g.d, TOL);
        chk("park.q", q, g.q, TOL);
        /* 逆变换往返一致性 */
        float a2, b2, c2, al2, be2;
        foc_inv_park(d, q, g.th, &al2, &be2);
        foc_inv_clarke(al2, be2, &a2, &b2, &c2);
        chk("roundtrip.alpha", al2, g.alpha, TOL);
        chk("roundtrip.beta",  be2, g.beta,  TOL);
    }

    printf("== SVPWM 占空比 ==\n");
    for (int i = 0; i < GOLDEN_SVPWM_N; ++i) {
        svpwm_t g = GOLDEN_SVPWM[i];
        float duty[3];
        foc_svpwm(g.valpha, g.vbeta, g.vdc, duty);
        chk("svpwm.d0", duty[0], g.d0, TOL);
        chk("svpwm.d1", duty[1], g.d1, TOL);
        chk("svpwm.d2", duty[2], g.d2, TOL);
    }

    printf("== 电流环（vs FieldWeakeningFOC）==\n");
    {
        motor_params_t m = {0.5f, 4e-3f, 6e-3f, 0.03f, 4};
        foc_t f;
        foc_init(&f, &m, 24.0f, 2.5f);   /* v_dc=24, i_max=2.5 对齐 InverterLimits */
        for (int i = 0; i < GOLDEN_ILOOP_N; ++i) {
            iloop_t g = GOLDEN_ILOOP[i];
            foc_current_step(&f, g.id_ref, g.iq_ref,
                             g.ia, g.ib, g.ic, g.th, GOLDEN_DT);
            chk("iloop.vd", f.v_d, g.vd, 1e-3f);
            chk("iloop.vq", f.v_q, g.vq, 1e-3f);
            chk("iloop.d0", f.duty[0], g.d0, TOL);
            chk("iloop.d1", f.duty[1], g.d1, TOL);
            chk("iloop.d2", f.duty[2], g.d2, TOL);
        }
    }

    if (fails == 0) {
        printf("\nALL PASS —— 固件 FOC 与仿真逐点一致。\n");
        return 0;
    }
    printf("\n%d 项不一致。\n", fails);
    return 1;
}
