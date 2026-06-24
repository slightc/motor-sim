# 自定义扩展

实现三个 Protocol(`Controller` / `Inverter` / `SensorSuite`)之一即可插入仿真，**无需改动 core**。

- `custom_controller.py` — 加一个新控制器(P 型位置保持)
- `custom_sensor.py` — 加一个新传感器(带温漂的电流传感器)

何时需要动 core？只有当你要引入一个**新的物理现象**（例如本项目为磁极辨识加入的 d 轴极性饱和）。新的*算法*永远只是新 Controller。
