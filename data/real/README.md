# data/real — 真机录波（sim↔real 对齐的真值基准）

存放从真实台架采集的电机录波，是 `tools/regress.py` 做「仿真↔实测」防退化回归的真值基准
（docs/05 §3-4）。**版本化、只增不改**（改动需新版本号）。

## 布局

```
data/real/
  golden/                     回归用黄金集（覆盖工况：低/高速、轻/重载、阶跃、变温）
    <工况>_<日期>_<固件>.csv   单条录波，字段见下（docs/05 §3.1）
    manifest.json             工况/输入序列/设备·固件版本/额定值
  validation/                 单次上板的人读验证报告（模板见 firmware/*/docs）
    <日期>_<电机>_<固件>.md
```

## CSV 字段（与仿真 `Observation`/`Measurements` 对齐，docs/05 §3.1）

```
t, i_a,i_b,i_c, i_d,i_q, v_a,v_b,v_c, theta_e, omega_m,
T_winding, v_dc, setpoint, t_load, label
```

- 回归重放用其中的**输入序列** `v_a,v_b,v_c, t_load`（+ 首行状态作初值）驱动仿真，
  逐点比对 `i_d,i_q,theta_e,omega_m`（及 `T_winding`，若开热模型）。
- `dt` 由相邻 `t` 推得；逐拍记录最稳（重放误差最小）。

## 跑回归

```bash
# 真机数据就位后：
python3 tools/regress.py --golden data/real/golden --config presets/<motor>.py \
                         --baseline report.json --out report.json
# 无硬件先自检整条链路（合成黄金，临时目录）：
python3 tools/regress.py --selftest
```

> 注：本目录现为空（仅此 README）。真机数据由实际采集填入；`--selftest` 用的合成录波
> 写到系统临时目录，不入库（它不是真机数据，别与 golden 混淆）。
