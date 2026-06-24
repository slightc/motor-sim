# -*- coding: utf-8 -*-
"""
motorsim.hardware —— 硬件抽象层

把真实硬件抽象成参数配置（电机/逆变器/电流·位置传感器/电压），并提供工厂方法装配
成 core 对象。对外主入口：

    from hardware_profile import HardwareProfile
    from ihm07m1 import X_NUCLEO_IHM07M1

新增一块硬件 = 新写一个返回 HardwareProfile 的模块（仿 ihm07m1.py），core 与 controller 不动。
"""
