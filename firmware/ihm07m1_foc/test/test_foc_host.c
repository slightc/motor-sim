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
#include "param_id.h"
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

    printf("== 反电动势观测器（vs BackEMFObserver）==\n");
    {
        bemf_obs_t o;
        bemf_obs_init(&o, OBS_R, OBS_L, OBS_P, OBS_FLP);
        bemf_obs_preset(&o, 0.0f, OBS_WE);
        /* 重建与 Python 端完全相同的 400 拍驱动序列 */
        float th_true = 0.0f, th_est = 0.0f, we_est = 0.0f;
        for (int k = 0; k < OBS_N; ++k) {
            th_true = fmodf(th_true + OBS_WE * GOLDEN_DT, 6.283185307179586f);
            float va = -OBS_WE * OBS_PSI * sinf(th_true);
            float vb =  OBS_WE * OBS_PSI * cosf(th_true);
            float ia = 0.3f * cosf(th_true);
            float ib = 0.3f * sinf(th_true);
            bemf_obs_update(&o, ia, ib, va, vb, GOLDEN_DT, &th_est, &we_est);
        }
        chk("obs.theta_final", th_est, OBS_THETA_FINAL, 1e-3f);
        chk("obs.omega_final", we_est, OBS_OMEGA_FINAL, 1e-2f);
        /* 收敛性：锁到 WE 附近 */
        chk("obs.omega~WE", we_est, OBS_WE, 5.0f);
    }

    printf("== 参数自整定公式 ==\n");
    {
        chk("calc_rs", pid_calc_rs(2.4f - 1.2f, 1.0f - 0.5f), 2.4f, 1e-5f); /* ΔV/ΔI */
        /* L = V·dt/Δi：给 L=4mH, V=3, dt=50us → Δi=0.0375 */
        chk("calc_L", pid_calc_L(3.0f, 50e-6f, 3.0f * 50e-6f / 4e-3f), 4e-3f, 1e-6f);
        chk("calc_psi", pid_calc_psi(2.0f, 0.5f, 1.0f, 50.0f), (2.0f - 0.5f) / 50.0f, 1e-6f);
    }

    printf("== 自整定闭环（合成 R-L 电机，回收已知 Rs/Ld/Lq）==\n");
    {
        const float DT = 50e-6f, VDC = 24.0f;
        const float R_true = 0.5f, LD_true = 4e-3f, LQ_true = 6e-3f;
        param_id_t p;
        param_id_init(&p, VDC, 2.5f, DT);
        /* 合成电机：θ=0 下 α 受 Ld 支配、β 受 Lq 支配，前向欧拉积分 */
        float ialpha = 0.0f, ibeta = 0.0f;
        pid_phase_t ph = PID_ALIGN;
        for (int k = 0; k < 60000 && ph != PID_DONE && ph != PID_FAIL; ++k) {
            float ia, ib, ic;
            foc_inv_clarke(ialpha, ibeta, &ia, &ib, &ic);
            float duty[3];
            ph = param_id_step(&p, ia, ib, ic, duty);
            /* 从占空比还原 αβ 电压（共模零序在 clarke 里抵消）*/
            float va = (duty[0] - 0.5f) * VDC;
            float vb = (duty[1] - 0.5f) * VDC;
            float vc = (duty[2] - 0.5f) * VDC;
            float valpha, vbeta;
            foc_clarke(va, vb, vc, &valpha, &vbeta);
            ialpha += (valpha - R_true * ialpha) / LD_true * DT;
            ibeta  += (vbeta  - R_true * ibeta)  / LQ_true * DT;
        }
        if (ph != PID_DONE) { printf("  [FAIL] 自整定未完成 (phase=%d)\n", ph); fails++; }
        chk("id.Rs", p.Rs, R_true,  0.05f);   /* 5% 容差（含 dt 离散/纹波）*/
        chk("id.Ld", p.Ld, LD_true, 0.4e-3f);
        chk("id.Lq", p.Lq, LQ_true, 0.6e-3f);
        printf("  回收: Rs=%.3fΩ Ld=%.2fmH Lq=%.2fmH (真值 0.500/4.00/6.00)\n",
               p.Rs, p.Ld * 1e3f, p.Lq * 1e3f);
    }

    if (fails == 0) {
        printf("\nALL PASS —— 固件 FOC 与仿真逐点一致。\n");
        return 0;
    }
    printf("\n%d 项不一致。\n", fails);
    return 1;
}
