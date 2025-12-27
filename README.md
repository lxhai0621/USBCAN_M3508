# M3508电机的Python驱动库

通过达妙科技USB转CAN模块控制大疆M3508电机的Python驱动库，支持电流控制和PID速度闭环控制。

## 功能特性

- 支持大疆M3508电机 (C620电调)
- 电流控制模式 (-16384 ~ 16384)
- PID速度闭环控制
- **持续控制模式**: 后台线程自动进行PID控制
- 实时获取电机状态（转速、角度、电流、温度）

## 硬件要求

- 大疆M3508电机 + C620电调
- 达妙科技USB转CAN模块
- USB数据线

## 软件要求

- Python 3.7+
- pyserial
- simple_pid

## 安装

```bash
# 安装依赖
pip install pyserial simple_pid
```

## API 文档

### USBCANDriver 类

USB驱动类，管理USB转CAN模块和电机控制。

#### 初始化

```python
from m3508_driver import create_driver

# 自动连接
driver = create_driver(port="COM3", baudrate=921600)

# 或手动连接
driver = USBCANDriver(port="COM3", baudrate=921600)
driver.connect()
```

#### 电机管理

| 方法 | 说明 |
|------|------|
| `add_motor(motor_id, kp, ki, kd)` | 添加单个电机 |
| `add_motors([id1, id2, ...], kp, ki, kd)` | 批量添加电机 |

#### 电流控制

| 方法 | 说明 |
|------|------|
| `set_motor_current(motor_id, current)` | 设置电机电流 (-16384~16384) |

#### 速度控制 (PID)

| 方法 | 说明 |
|------|------|
| `set_motor_speed(motor_id, speed, continuous=False)` | 设置电机目标速度 |
| `set_motors_speed({id: speed, ...})` | 批量设置电机速度 |

**continuous 参数说明:**
- `False` (默认): 单次控制，发送一次控制命令
- `True`: 持续控制，后台线程每10ms自动发送PID控制命令

#### PID参数设置

| 方法 | 说明 |
|------|------|
| `set_motor_pid(motor_id, kp, ki, kd)` | 设置单个电机的PID参数 |
| `set_motors_pid({id: (kp, ki, kd), ...})` | 批量设置PID参数 |
| `set_motor_pid_enabled(motor_id, enabled)` | 启用/禁用PID控制 |
| `set_motors_pid_enabled({id: enabled, ...})` | 批量启用/禁用PID |

#### 电机停止

| 方法 | 说明 |
|------|------|
| `stop_motor(motor_id)` | 停止单个电机的持续控制 |
| `stop_all_continuous_control()` | 停止所有电机的持续控制 |
| `stop_all_motors()` | 停止所有电机并清零电流 |

#### 反馈数据

| 方法 | 说明 |
|------|------|
| `get_motor_feedback(motor_id)` | 获取电机反馈数据 |

#### 连接管理

| 方法 | 说明 |
|------|------|
| `connect()` | 连接到USB转CAN模块 |
| `disconnect()` | 断开连接 |

### M3508Motor 类

电机类，包含PID控制器和反馈数据。

#### 方法

| 方法 | 说明 |
|------|------|
| `set_motor_speed(speed)` | 设置目标速度 (RPM) |
| `set_pid_tunings(kp, ki, kd)` | 设置PID参数 |
| `compute_pid()` | 计算PID输出 |
| `get_output_speed()` | 获取减速箱输出速度 |
| `enable_pid()` / `disable_pid()` | 启用/禁用PID |
| `set_current(current)` | 设置并限制电流 |

#### 属性

| 属性 | 说明 |
|------|------|
| `motor_id` | 电机ID (1-8) |
| `can_id` | CAN ID (0x201-0x208) |
| `feedback` | 反馈数据对象 |
| `target_speed` | 目标速度 |

### MotorFeedback 数据类

电机反馈数据。

```python
@dataclass
class MotorFeedback:
    angle: float          # 角度 0-360度
    speed: int            # 转速 RPM (电机转速)
    current: int          # 电流 mA
    torque_current: int   # 扭矩电流原始值
    temperature: int      # 温度 摄氏度
    location: int         # 多圈位置
```

## 运行示例

运行示例程序：

```bash
python m3508_example.py
```

**可用示例：**

| 选项 | 示例名称 | 说明 |
|------|----------|------|
| 1 | 基本电机控制 | 演示电流控制模式，设置电机正反转 |
| 2 | 持续读取电机反馈 | 持续读取并显示电机状态（角度、速度、电流、温度） |
| 3 | PID速度闭环控制 (单次控制) | 手动循环调用PID控制，需要用户代码循环 |
| 4 | 持续速度控制 (continuous=True) | 后台线程自动控制，无需用户循环 |
| 5 | 批量设置电机速度 | 演示批量控制多个电机 |


## 项目文件

```
USBCAN_M3508/
├── m3508_driver.py                    # 主驱动库
├── m3508_example.py                   # 使用示例
├── README.md                          # README文件
├── RoboMaster C620无刷电机调速器使用说明（中英日）V1.01.pdf  # C620电调手册
├── USB 转CAN 帧格式.xlsx              # USB-CAN协议文档
├── USB2CAN_2.0.0.3.exe                # USB转CAN上位机
└── USB转CAN软件使用教程.pdf            # 软件使用教程
```

## 注意事项

1. **串口占用**: 确保COM口未被其他程序占用
2. **波特率**: 默认921600，与USB转CAN模块匹配
3. **电机ID**: 电机ID必须为1-8
4. **PID调优**: 默认PID参数为保守值，精准控制需要根据实际负载调优


## 参考文档

- [RoboMaster C620电调手册](RoboMaster C620无刷电机调速器使用说明（中英日）V1.01.pdf)
- [USB转CAN协议](USB 转CAN 帧格式.xlsx)
- [USB转CAN软件教程](USB转CAN软件使用教程.pdf)
