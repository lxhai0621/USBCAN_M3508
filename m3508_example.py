"""
M3508电机驱动使用示例
演示如何使用m3508_driver.py控制M3508电机
"""

import time
from m3508_driver import USBCANDriver, M3508Motor, create_driver

def example_basic_control():
    """示例1: 基本电机控制"""
    print("=== 示例1: 基本电机控制 ===")
    # 创建驱动并连接
    driver = create_driver(port="COM3", baudrate=921600)

    # 创建电机
    motor1 = driver.create_motor(1)  # CAN ID: 0x201
    motor2 = driver.create_motor(2)  # CAN ID: 0x202

    try:
        # 设置电机电流 (电流值范围: -16384 ~ 16384)
        # 正值正转，负值反转
        driver.set_motor_current(1, 2000)   # 电机1正转
        driver.set_motor_current(2, -2000)   # 电机2反转
        driver.send_control_command() #发送电流值

        print("电机1和2以电流2000启动...")
        time.sleep(2)
        # 停止电机
        driver.stop_all_motors()
        print("电机已停止")

    finally:
        driver.disconnect()


def example_feedback_read():
    """示例2: 持续读取电机反馈"""
    print("\n============= 示例2: 持续读取电机反馈 ==========")

    driver = create_driver(port="COM3", baudrate=921600)
    motor1 = driver.create_motor(1)
    motor2 = driver.create_motor(2)

    try:
        #启动电机
        driver.set_motor_current(1, 2500)
        driver.set_motor_current(2, -2000)  # 反转
        print("电机1和2启动，开始读取反馈...")

        # 持续读取10秒
        start_time = time.time()
        while time.time() - start_time < 10:
            fb1 = motor1.feedback
            fb2 = motor2.feedback
            print(f"电机1 - 角度: {fb1.angle:.1f}° | 速度: {fb1.speed:4d} RPM | "
                  f"输出速度: {motor1.get_output_speed():5.1f} RPM | "
                  f"电流: {fb1.current:4d} mA | 温度: {fb1.temperature}°C | "
                  f"电机2 - 角度: {fb2.angle:.1f}° | 速度: {fb2.speed:4d} RPM | "
                  f"输出速度: {motor2.get_output_speed():5.1f} RPM | "
                  f"电流: {fb2.current:4d} mA | 温度: {fb2.temperature}°C | ")
            time.sleep(0.1)

        # 停止电机
        driver.stop_all_motors()
        print("电机已停止")

    finally:
        driver.stop_reading()
        driver.disconnect()

def example_pid_speed_control():
    """示例3: PID速度闭环控制"""
    print("\n=== 示例3: PID速度闭环控制 ===")

    # 创建驱动并连接
    driver = create_driver(port="COM3", baudrate=921600)

    # 创建电机，指定PID参数 (kp, ki, kd)
    # 参数需要根据实际情况调整
    motor1 = driver.create_motor(1)
    motor2 = driver.create_motor(2)
    motor1.set_pid_tunings(kp=2.0, ki=0.5, kd=0.1)  # 调整PID参数
    motor2.set_pid_tunings(kp=2.0, ki=0.5, kd=0.1)

    try:
        target_speed = 3000
        # 运行PID控制10秒
        print("启动PID控制...")
        start_time = time.time()

        while time.time() - start_time < 10:
            driver.set_motor_speed(1, 3000)
            driver.set_motor_speed(2, 3000)
            # 显示状态
            fb1 = motor1.feedback
            fb2 = motor2.feedback
            print(f"motor1:目标: {target_speed:4d} RPM | 实际: {fb1.speed:4d} RPM | "
                  f"误差: {target_speed - fb1.speed:4d} ")
            print(f"motor2:目标: {target_speed:4d} RPM | 实际: {fb2.speed:4d} RPM | "
                  f"误差: {target_speed - fb2.speed:4d} ")

            time.sleep(0.01)  # 10ms控制周期

        # 停止
        driver.stop_all_motors()
        print("PID控制完成")

    finally:
        driver.stop_reading()
        driver.disconnect()



if __name__ == "__main__":
    # 运行示例
    import sys

    examples = {
        '1': example_basic_control,
        '2': example_feedback_read,
        '3': example_pid_speed_control,
    }

    print("M3508电机驱动示例")
    print("请选择要运行的示例:")
    for key, name in [
        ('1', '基本电机控制'),
        ('2', '持续读取电机反馈'),
        ('3', 'PID速度闭环控制')
    ]:
        print(f"  {key}. {name}")

    choice = input("\n请输入选项 (1-3, 默认1): ").strip() or '1'

    if choice in examples:
        try:
            examples[choice]()
        except KeyboardInterrupt:
            print("\n用户中断")
        except Exception as e:
            print(f"错误: {e}")
    else:
        print("无效选项")
