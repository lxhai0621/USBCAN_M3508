"""
M3508电机驱动使用示例
演示如何使用m3508_driver.py控制M3508电机
"""

import time
from m3508_driver import create_driver

def example_basic_control():
    """示例1: 基本电机控制"""
    print("=== 示例1: 基本电机控制 ===")
    # 创建驱动并连接
    driver = create_driver(port="COM3", baudrate=921600)

    # 添加电机 (参数: motor_id)
    driver.add_motor(1)
    driver.add_motor(2)

    try:
        # 设置电机电流 (电流值范围: -16384 ~ 16384)
        # 正值正转，负值反转
        driver.set_motor_current(1, 2000)   # 电机1正转
        driver.set_motor_current(2, -2000)  # 电机2反转

        print("电机1和2以电流2000启动...")
        time.sleep(2)

        # 停止电机
        driver.stop_all_motors()
        print("电机已停止")

    finally:
        driver.disconnect()


def example_feedback_read():
    """示例2: 持续读取电机反馈"""
    print("\n=== 示例2: 持续读取电机反馈 ===")

    driver = create_driver(port="COM3", baudrate=921600)
    driver.add_motor(1)
    driver.add_motor(2)

    try:
        # 启动电机
        driver.set_motor_current(1, 2500)
        driver.set_motor_current(2, -2000)
        print("电机1和2启动，开始读取反馈...")

        # 持续读取10秒
        start_time = time.time()
        while time.time() - start_time < 10:
            fb1 = driver.get_motor_feedback(1)
            fb2 = driver.get_motor_feedback(2)
            print(f"电机1 - 角度: {fb1.angle:.1f}° | 速度: {fb1.speed:4d} RPM | "
                  f"电流: {fb1.current:4d} mA | 温度: {fb1.temperature}°C | "
                  f"电机2 - 角度: {fb2.angle:.1f}° | 速度: {fb2.speed:4d} RPM | "
                  f"电流: {fb2.current:4d} mA | 温度: {fb2.temperature}°C")
            time.sleep(0.1)

        # 停止电机
        driver.stop_all_motors()
        print("电机已停止")

    finally:
        driver.disconnect()


def example_pid_speed_control():
    """示例3: PID速度闭环控制 (单次控制模式)"""
    print("\n=== 示例3: PID速度闭环控制 ===")

    # 创建驱动并连接
    driver = create_driver(port="COM3", baudrate=921600)

    # 添加电机，指定PID参数 (kp, ki, kd)
    driver.add_motor(1, kp=2.0, ki=0.5, kd=0.1)
    driver.add_motor(2, kp=2.0, ki=0.5, kd=0.1)

    try:
        target_speed = 3000
        # 运行PID控制10秒
        print("启动PID控制...")
        start_time = time.time()

        while time.time() - start_time < 10:
            # 设置目标速度 (单次控制模式，需要循环调用)
            driver.set_motor_speed(1, target_speed)
            driver.set_motor_speed(2, target_speed)

            # 显示状态
            fb1 = driver.get_motor_feedback(1)
            fb2 = driver.get_motor_feedback(2)
            print(f"电机1 - 目标: {target_speed:4d} RPM | 实际: {fb1.speed:4d} RPM | "
                  f"误差: {target_speed - fb1.speed:4d}")
            print(f"电机2 - 目标: {target_speed:4d} RPM | 实际: {fb2.speed:4d} RPM | "
                  f"误差: {target_speed - fb2.speed:4d}")

            time.sleep(0.01)  # 10ms控制周期

        # 停止
        driver.stop_all_motors()
        print("PID控制完成")

    finally:
        driver.disconnect()


def example_continuous_speed_control():
    """示例4: 持续速度控制 (continuous=True)"""
    print("\n=== 示例4: 持续速度控制 ===")

    # 创建驱动并连接
    driver = create_driver(port="COM3", baudrate=921600)

    # 添加电机，指定PID参数
    driver.add_motor(1, kp=2.0, ki=0.5, kd=0.1)
    driver.add_motor(2, kp=2.0, ki=0.5, kd=0.1)

    try:
        # 启动持续速度控制
        print("启动持续速度控制: 1000 RPM")
        driver.set_motor_speed(1, 1000, continuous=True)
        driver.set_motor_speed(2, 1000, continuous=True)

        time.sleep(3)

        # 改变速度 (无需重启线程)
        print("改变速度: 2000 RPM")
        driver.set_motor_speed(1, 2000, continuous=True)
        driver.set_motor_speed(2, 2000, continuous=True)

        time.sleep(3)

        # 反向转动
        print("反向转动: -1000 RPM")
        driver.set_motor_speed(1, -1000, continuous=True)
        driver.set_motor_speed(2, -1000, continuous=True)

        time.sleep(3)

        # 反向转动
        print("反向转动: -2000 RPM")
        driver.set_motor_speed(1, -2000, continuous=True)
        driver.set_motor_speed(2, -2000, continuous=True)

        time.sleep(3)

        # 停止电机
        print("停止电机")
        driver.stop_motor(1)
        driver.stop_motor(2)
        print("持续控制完成")
        time.sleep(5)
        print("程序结束")

    finally:
        driver.disconnect()


def example_batch_speed_control():
    """示例5: 批量设置电机速度"""
    print("\n=== 示例5: 批量设置电机速度 ===")

    driver = create_driver(port="COM3", baudrate=921600)

    # 批量添加电机
    driver.add_motors([1, 2, 3, 4], kp=2.0, ki=0.5, kd=0.1)

    try:
        # 启动持续控制
        print("启动电机1和2的持续控制")
        driver.set_motor_speed(1, 1500, continuous=True)
        driver.set_motor_speed(2, 1500, continuous=True)

        time.sleep(2)

        # 批量设置速度 (电机1,2在持续控制模式会自动更新；电机3,4单次控制)
        print("批量设置速度: {1: 2000, 2: -2000, 3: 1000, 4: -1000}")
        driver.set_motors_speed({1: 2000, 2: -2000, 3: 1000, 4: -1000})

        time.sleep(2)

        # 显示状态
        for motor_id in [1, 2, 3, 4]:
            fb = driver.get_motor_feedback(motor_id)
            print(f"电机{motor_id} - 速度: {fb.speed:4d} RPM")

        # 停止所有电机
        driver.stop_all_motors()
        print("所有电机已停止")

    finally:
        driver.disconnect()


if __name__ == "__main__":
    # 运行示例
    import sys

    examples = {
        '1': example_basic_control,
        '2': example_feedback_read,
        '3': example_pid_speed_control,
        '4': example_continuous_speed_control,
        '5': example_batch_speed_control,
    }

    print("M3508电机驱动示例")
    print("请选择要运行的示例:")
    for key, name in [
        ('1', '基本电机控制'),
        ('2', '持续读取电机反馈'),
        ('3', 'PID速度闭环控制 (单次控制模式)'),
        ('4', '持续速度控制 (continuous=True)'),
        ('5', '批量设置电机速度')
    ]:
        print(f"  {key}. {name}")

    choice = input("\n请输入选项 (1-5, 默认1): ").strip() or '1'

    if choice in examples:
        try:
            examples[choice]()
        except KeyboardInterrupt:
            print("\n用户中断")
        except Exception as e:
            print(f"错误: {e}")
    else:
        print("无效选项")
