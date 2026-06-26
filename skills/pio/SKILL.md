---
name: pio
description: >
  用 PlatformIO 构建/烧录/调试/测试嵌入式固件（MCU 工程）。当用户需要：编译固件
  (pio run)、烧录到开发板 (upload)、串口监视 (monitor)、片上调试 (debug)、跑单元测试
  (pio test)、管理平台/框架/库依赖、配置 platformio.ini（board/framework/build_flags）、
  组织固件工程结构、或把算法做成可在 PC 上回归的硬件无关模块时，使用本 skill。
  涉及 platformio/pio/固件/firmware/烧录/upload/flash/MCU/单片机/嵌入式/STM32/ESP32/
  Arduino/HAL 等关键词即触发。具体板子/引脚/构建环境等**项目特定信息见该固件目录下的 README**。
---

# pio — 用 PlatformIO 构建嵌入式固件

通用 PlatformIO 使用指南。**项目特定的事实**（目标板、引脚映射、接线、构建环境、
运行模式、安全须知）**不在本 skill**，而在对应固件目录的 `README`——动手前先读它。

## 何时用

- 编译 / 烧录 / 监视 / 调试 / 测试一个 PlatformIO 固件工程
- 新增或修改外设驱动、板级支持(BSP)、中断、控制/采集逻辑
- 配置 `platformio.ini`（环境、平台、框架、编译选项、上传/调试工具）
- 把算法拆成硬件无关模块，在 PC 上做回归

## 常用命令

```bash
cd <固件目录>                 # 含 platformio.ini 的目录
pio run                       # 编译（默认环境；首次自动拉平台/工具链/框架）
pio run -e <env>              # 指定环境编译
pio run -t upload             # 烧录到板子
pio run -t clean              # 清理构建产物
pio run -v                    # 详细输出（排错看完整编译/链接命令）
pio device monitor            # 串口监视（波特率见 monitor_speed）
pio device list               # 列出已连接串口/设备
pio debug                     # 启动片上调试（需 debug_tool）
pio test                      # 跑单元测试（test/ 目录）
pio test -e native            # 在 PC 本机跑测试（需 native 环境）
pio pkg install               # 安装/同步依赖
pio check                     # 静态分析
```

排错优先级：`pio run -v` 看真实编译命令 → 确认 `platformio.ini` 的 board/framework →
确认 `pio device list` 能看到板子（烧录失败多是接口/权限/驱动）。

## platformio.ini 结构

每个 `[env:...]` 是一个构建目标。最小骨架：

```ini
[env:<env_name>]
platform  = <platform>       ; 芯片平台，如 ststm32 / espressif32 / atmelavr
board     = <board_id>       ; 具体板子 ID，用 `pio boards <关键词>` 查
framework = <framework>      ; 框架，如 stm32cube / arduino / zephyr / cmsis

upload_protocol = <tool>     ; 烧录接口，如 stlink / esptool / jlink
debug_tool      = <tool>
monitor_speed   = 115200

build_flags =                ; 预处理宏 / 头文件路径 / 优化
  -Iinclude
  -DSOME_DEFINE=1
  -Wall -Wextra

build_src_filter = +<*> -<../test/>   ; 控制哪些源文件进固件
```

- `pio boards <keyword>` 查板子 ID；`pio platform show <platform>` 看可用框架。
- 公共配置可放 `[env]` 或自定义段 + `extends` 复用。
- 板子专属常量（时钟、引脚、外设地址）**不要散在各处**——集中到一个 BSP 头文件，
  换板子只改一处（具体映射写在固件 README，不写进本 skill）。

## 工程结构约定（通用）

```
<固件目录>/
  platformio.ini
  README              ← 项目特定: 板子/引脚/接线/构建环境/运行说明（动手前先读）
  include/            公共头文件（接口、BSP 引脚/常量、框架配置头）
  src/                固件源码（main + 驱动 + 业务逻辑）
  lib/                私有库（每个子目录一个库，LDF 自动识别）
  test/              单元/集成测试（pio test；可含 native 主机测试）
```

划分建议：**硬件相关**（寄存器/HAL/引脚/中断）与**纯算法/业务逻辑**分文件。
后者不依赖任何硬件头，便于复用与测试（见下）。

## 关键实践：算法硬件无关 + PC 端回归

把核心算法写成**纯计算模块**（只依赖 `<math.h>`/标准库，不碰寄存器/HAL），
就能在 PC 上原生编译、用已知输入/期望输出做回归，无需上板：

```bash
# 直接 gcc 编译纯算法模块 + 测试 harness（不链接任何 HAL）
cc -std=c11 -O2 -Wall -Wextra -Iinclude test/test_xxx.c src/xxx.c -lm -o /tmp/t && /tmp/t
# 或用 PlatformIO 的 native 测试环境： pio test -e native
```

好处：算法逻辑改动**先在 PC 上验证**（秒级、可断言、可对拍参考实现），
再上板验外设时序。改算法 → 跑回归 → 再编译固件，是稳的迭代节奏。

## 本环境注意

- 工具链/平台/框架由 PlatformIO 按 `platformio.ini` 自动下载并缓存（首次较慢）。
- 烧录/监视需物理连板；纯编译与 PC 端回归无需硬件。
- 上电驱动真实硬件前，务必读该固件 README 的接线与安全说明。
