# 硬件：X-NUCLEO-IHM07M1

基于 L6230 的三相 BLDC 驱动板（已联网核对规格）。

| 项 | 值 |
|----|----|
| 驱动 | L6230, 8–48 VDC |
| 电流能力 | 2.8A 峰值 / 1.4 Arms（小电机） |
| PWM | 最高 100 kHz |
| 分流电阻 | R43/R44/R45 = 0.33Ω 1W |
| 运放 | TSV994, 增益 1.53 |
| 电流→电压 | 0.33×1.53 = 0.505 V/A |
| ADC | STM32 内部 12-bit, ±3.27A 量程 → **LSB ≈ 1.6mA** |
| 采样 | 三分流/单分流可选(跳线 J5/J6) |
| 其它 | 板载 BEMF(六步)电路、霍尔/编码器接口 |

## 低速高负载无感的硬件要点

1. **必须三分流**：单分流在低速/低调制下相电流重构不可靠。
2. **电流分辨率够**(1.6mA LSB)，瓶颈在**模拟噪声**——同步 PWM 中点采样、布线滤波。
3. **HFI 信号** ~17mA vs 1.6mA LSB，约 10 LSB 余量，可用。
4. **"高负载"被 2.8A 峰值卡死**——小电机；要更大电流换 100mΩ 分流(分辨率降)。
5. 板载 BEMF 电路仅对六步有效，低速 FOC 无感不用它。

仿真对齐：`CurrentSensor(adc_bits=12, i_range=3.27, noise_std=0.003)` + `InverterLimits(24, 2.5)`。

## 代码入口

无需手工拼装上述参数，直接用硬件抽象层：

```python
from ihm07m1 import X_NUCLEO_IHM07M1     # hardware/ 目录
hw  = X_NUCLEO_IHM07M1(position="encoder")
sim = hw.build_simulator(controller)     # 自动装好 plant+inverter+sensors+limits
```

`hardware/` 把整块板子抽象成 `HardwareProfile`（电机/功率级/电流·位置传感器），
电机电磁参数为占位默认值，应由真机标定回填（见 `05_hardware_deployment.md`）。详见 `hardware/README.md`。
