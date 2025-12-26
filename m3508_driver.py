"""
M3508电机驱动库
通过达妙科技USB转CAN模块控制大疆M3508电机
支持PID速度闭环控制
"""
import serial
import struct
import time
import threading
from typing import Optional, Tuple
from dataclasses import dataclass
from enum import IntEnum
from simple_pid import PID

class CANCMD(IntEnum):
    """CAN命令类型"""
    SEND_FRAME = 0x01       # 转CAN发送帧
    PC_HANDSHAKE = 0x02     # PC与设备握手
    QUERY_STATUS = 0x03     # 非反馈CAN转发 (不反馈发送状态)

@dataclass
class MotorFeedback:
    """电机反馈数据"""
    angle: float = 0.0          # 角度 0-360度
    speed: int = 0              # 转速 RPM
    current: int = 0            # 电流 mA
    torque_current: int = 0     # 扭矩电流原始值
    temperature: int = 0        # 温度 摄氏度
    location: int = 0           # 多圈位置

class M3508Motor:
    """M3508电机类 - 支持PID速度闭环控制"""
    # 3508电机参数 (减速比19:1)
    REDUCTION_RATIO = 19.0
    MAX_CURRENT = 16384         # 最大电流值 (-16384 ~ 16384)

    def __init__(self, motor_id: int, kp: float = 1.0, ki: float = 0.1, kd: float = 0.0):
        """
        初始化电机
        motor_id: 电机ID 1-8
        kp, ki, kd: PID参数
        """
        if motor_id < 1 or motor_id > 8:
            raise ValueError("电机ID必须在1-8范围内")
        self.motor_id = motor_id
        self.can_id = 0x200 + motor_id  # 0x201-0x208
        self.feedback = MotorFeedback()

        # 目标速度 (电机转速RPM)
        self.target_speed = 0

        # PID控制器
        self.pid = PID(kp, ki, kd, setpoint=0,
                        output_limits=(-16384, 16384),
                        sample_time=0.01)  # 10ms采样周期
        self.pid_enabled = True

    def set_current(self, current: int) -> int:
        """
        设置电机电流
        current: 电流值 -16384 ~ 16384
        """
        # 限流
        if current > self.MAX_CURRENT:
            current = self.MAX_CURRENT
        elif current < -self.MAX_CURRENT:
            current = -self.MAX_CURRENT
        return current

    def set_motor_speed(self, speed: float):
        """
        设置目标速度 (电机转速 RPM)
        speed: 目标转速 RPM
        """
        self.target_speed = speed
        if self.pid:
            self.pid.setpoint = speed

    
    def get_output_speed(self) -> float:
        """
        获取减速箱输出速度 (RPM)
        计算公式: 输出速度 = 电机速度 / 减速比
        """
        return self.feedback.speed / self.REDUCTION_RATIO

    def compute_pid(self) -> int:
        """
        计算PID输出 (电流值)
        return: 控制电流
        """
        if self.pid and self.pid_enabled:
            current = int(self.pid(self.feedback.speed))
            return current
        return 0

    def update_feedback(self, data: bytes):
        """
        更新电机反馈数据
        data: 8字节CAN数据
        C620电调反馈数据格式:
            data[0-1]: 机械角度 (0-8191)
            data[2-3]: 转速 RPM
            data[4-5]: 扭矩电流
            data[6]: 温度
            data[7]: Null
        """
        if len(data) >= 7:
            angle = (data[0] << 8) | data[1]
            self.feedback.speed = struct.unpack('>h', bytes([data[2], data[3]]))[0]
            self.feedback.torque_current = struct.unpack('>h', bytes([data[4], data[5]]))[0]
            self.feedback.temperature = data[6]
            # 转换电流为mA (2e4*current/16384.0)
            self.feedback.current = int(20000 * self.feedback.torque_current / 16384.0)
            # 角度转换为度
            self.feedback.angle = angle / 8192.0 * 360.0

    def set_pid_tunings(self, kp: float, ki: float, kd: float):
        """
        设置PID参数
        kp, ki, kd: PID参数
        """
        if self.pid:
            self.pid.tunings = (kp, ki, kd)
            self.pid.reset()

    def enable_pid(self):
        """启用PID控制"""
        self.pid_enabled = True

    def disable_pid(self):
        """禁用PID控制"""
        self.pid_enabled = False


class USBCANDriver:
    """USB转CAN驱动类"""
    # USB转CAN帧头和帧尾
    FRAME_HEAD_TX = bytes([0x55, 0xAA])
    FRAME_HEAD_RX = bytes([0xAA])
    FRAME_TAIL = bytes([0x55, 0x88])  # 发送帧尾
    FRAME_TAIL_RX = bytes([0x55])     # 接收帧尾

    # CAN ID定义
    CAN_ID_CTRL_1_4 = 0x200   # 控制电机1-4
    CAN_ID_CTRL_5_8 = 0x1FF   # 控制电机5-8

    def __init__(self, port: str = "COM3", baudrate: int = 921600, timeout: float = 0.1):
        """
        初始化USB转CAN驱动
        port: 串口号
        baudrate: 波特率
        timeout: 超时时间
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None
        self.motors: list[M3508Motor] = []

        # 电机电流控制值 (对应CAN ID 0x200和0x1FF)
        self._currents_1_4 = [0, 0, 0, 0]  # 电机1-4的电流
        self._currents_5_8 = [0, 0, 0, 0]  # 电机5-8的电流

        # 反馈读取线程控制
        self._reading = False
        self._read_thread: Optional[threading.Thread] = None

    def connect(self) -> bool:
        """
        连接到USB转CAN模块
        """
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            print(f"串口 {self.ser.port} 已打开，波特率: {self.ser.baudrate}")
            # 连接成功后启动反馈读取线程
            self.start_reading()
            return True
        except serial.SerialException as e:
            print(f"串口连接失败: {e}")
            return False

    def disconnect(self):
        """断开连接"""
        # 先停止读取线程
        self.stop_reading()
        # 再关闭串口
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("串口已关闭")

    def create_motor(self, motor_id: int) -> M3508Motor:
        """
        创建并添加电机
        motor_id: 电机ID 1-8
        """
        motor = M3508Motor(motor_id)
        self.motors.append(motor)
        return motor

    def set_motor_current(self, motor_id: int, current: int):
        """
        设置并发送电机电流
        motor_id: 电机ID 1-8
        current: 电流值 -16384 ~ 16384
        """
        if motor_id < 1 or motor_id > 8:
            raise ValueError("电机ID必须在1-8范围内")

        # 查找并更新电机
        for motor in self.motors:
            if motor.motor_id == motor_id:
                current = motor.set_current(current)
                print(f"设置电机{motor_id}电流: {current}")
                break

        # 更新控制电流数组
        idx = motor_id - 1
        if motor_id <= 4:
            self._currents_1_4[idx] = current
        else:
            self._currents_5_8[idx - 4] = current
        self.send_control_command()

    def set_motor_speed(self, motor_id: int, speed: float):
        """
        设置电机目标速度 (PID控制)
        motor_id: 电机ID 1-8
        speed: 目标速度 (电机转速 RPM)
        """
        if motor_id < 1 or motor_id > 8:
            raise ValueError("电机ID必须在1-8范围内")

        # 查找电机并设置速度
        for motor in self.motors:
            if motor.motor_id == motor_id:
                # 设置目标速度
                motor.set_motor_speed(speed)
                # 计算PID输出
                current = motor.compute_pid()
                # 更新控制电流数组
                idx = motor_id - 1
                if motor_id <= 4:
                    self._currents_1_4[idx] = current
                else:
                    self._currents_5_8[idx - 4] = current
                # 发送控制命令
                self.send_control_command()
                break

    def set_motors_speed(self, speeds: dict[int, float]):
        """
        批量设置电机速度 (PID控制)
        speeds: {电机ID: 目标速度} 字典
        """
        for motor_id, speed in speeds.items():
            if motor_id < 1 or motor_id > 8:
                continue
            # 查找电机并设置速度
            for motor in self.motors:
                if motor.motor_id == motor_id:
                    motor.set_motor_speed(speed)
                    current = motor.compute_pid()
                    idx = motor_id - 1
                    if motor_id <= 4:
                        self._currents_1_4[idx] = current
                    else:
                        self._currents_5_8[idx - 4] = current
                    break
        # 发送控制命令
        self.send_control_command()

    def _build_can_frame(self, can_id: int, data: bytes) -> bytes:
        """
        构建USB转CAN发送帧
        帧结构:
        - 帧头: 0x55 0xAA
        - 帧长: 0x1E
        - 命令: 0x01 (转CAN发送帧) 0x02 (PC握手) 0x03 (非反馈CAN转发,不反馈发送状态)
        - 发送次数: 0x01 0x00 0x00 0x00
        - 时间间隔: 0x0A 0x00 0x00 0x00 
        - ID类型：0x00 00标准帧 01扩展帧
        - CAN ID (小端序，4字节) 
        - 帧类型: 0x00 00数据帧 01远程帧
        - 数据长度: 0x08
        - 预留: 0x00 0x00
        - CAN数据 (8字节)
        - 帧尾: 0x88

        can_id: CAN ID
        data: CAN数据 (最多8字节)
        return: 完整的USB转CAN帧
        """
        if len(data) > 8:
            raise ValueError("CAN数据最多8字节")

        # 构建帧
        frame = bytearray()
        frame.extend(self.FRAME_HEAD_TX)  # 0x55 0xAA
        frame.append(0x1E)                 # 帧类型
        frame.append(CANCMD.SEND_FRAME)    # 命令: 转CAN发送帧
        frame.append(0x01)                 # 发送次数 (1次)
        frame.extend([0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00])
        # CAN ID (小端序，4字节)
        frame.extend(can_id.to_bytes(4, byteorder='little'))
        # 帧类型 (1字节): 数据帧/远程帧
        frame.append(0x00)
        # 数据长度
        frame.append(len(data))
        # 预留 (2字节)
        frame.extend([0x00, 0x00])

        # CAN数据 (8字节)
        frame.extend(data.ljust(8, b'\x00'))

        # 帧尾
        frame.append(0x88)

        return bytes(frame)

    def send_control_command(self):
        """
        发送电机控制命令
        根据使用的电机发送对应的CAN帧 (0x200控制电机1-4, 0x1FF控制电机5-8)

        CAN数据格式 (大端序):
        CAN ID 0x200: [电机1高8位, 电机1低8位, 电机2高8位, 电机2低8位, 电机3高8位, 电机3低8位, 电机4高8位, 电机4低8位]
        CAN ID 0x1FF: [电机5高8位, 电机5低8位, 电机6高8位, 电机6低8位, 电机7高8位, 电机7低8位, 电机8高8位, 电机8低8位]
        """
        # 检查是否有电机1-4被创建
        has_motors_1_4 = any(m.motor_id <= 4 for m in self.motors)
        # 检查是否有电机5-8被创建
        has_motors_5_8 = any(m.motor_id > 4 for m in self.motors)

        # 发送控制电机1-4的命令 (CAN ID 0x200)
        if has_motors_1_4:
            data = bytearray()
            for current in self._currents_1_4:
                data.append((current >> 8) & 0xFF)  # 高8位
                data.append(current & 0xFF)          # 低8位
            frame = self._build_can_frame(self.CAN_ID_CTRL_1_4, bytes(data))
            if self.ser and self.ser.is_open:
                self.ser.write(frame)

        # 发送控制电机5-8的命令 (CAN ID 0x1FF)
        if has_motors_5_8:
            data = bytearray()
            for current in self._currents_5_8:
                data.append((current >> 8) & 0xFF)  # 高8位
                data.append(current & 0xFF)          # 低8位
            frame = self._build_can_frame(self.CAN_ID_CTRL_5_8, bytes(data))
            if self.ser and self.ser.is_open:
                self.ser.write(frame)

    def _parse_can_rx_frame(self, frame: bytes) -> Optional[Tuple[int, bytes]]:
        """
        解析CAN接收帧 (固定16字节)

        帧结构 (16字节):
        - Byte 0:  0xAA (帧头)
        - Byte 1:  CMD (命令状态: 0x11=接收成功)
        - Byte 2:  格式
        - Byte 3-6: CAN ID (4字节, 小端序)
        - Byte 7-14: CAN数据 (8字节)
        - Byte 15: 0x55 (帧尾)

        frame: 接收到的帧 (16字节)
        return: (CAN ID, CAN数据) 或 None
        """
        if len(frame) != 16 or frame[0] != 0xAA or frame[15] != 0x55:
            return None

        cmd = frame[1]
        # 检查是否接收成功 (0x11)
        if cmd == 0x11:
            # 解析Byte 2: 数据长度 + IDE + RTR
            byte2 = frame[2]
            data_len = byte2 & 0x3F          # 低6位: 数据长度
            ide = (byte2 >> 6) & 0x01        # bit6: 0=标准帧, 1=扩展帧
            rtr = (byte2 >> 7) & 0x01        # bit7: 0=数据帧, 1=远程帧

            # 解析CAN ID (Bytes 3-6, 小端序)
            can_id = int.from_bytes(frame[3:7], byteorder='little')

            # 提取CAN数据 (Bytes 7-14)
            can_data = frame[7:7 + data_len]

            return (can_id, can_data)

        return None

    def read_feedback(self, timeout_ms: int = 100) -> bool:
        """
        读取电机反馈数据
        CAN接收帧固定16字节 (0xAA开头 + 15字节)
        :param timeout_ms: 超时时间(毫秒)
        :return: 是否读取到有效数据
        """
        if not self.ser or not self.ser.is_open:
            return False

        # 查找帧头0xAA
        start_time = time.time()
        while (time.time() - start_time) * 1000 < timeout_ms:
            byte = self.ser.read(1)
            if not byte:
                return False
            if byte[0] == 0xAA:
                # 读取后续15字节，总共16字节
                remaining = self.ser.read(15)
                if len(remaining) < 15:
                    continue

                frame = bytes(byte) + remaining  # 总共16字节
                result = self._parse_can_rx_frame(frame)
                if result:
                    can_id, can_data = result
                    # 更新对应电机的反馈数据
                    for motor in self.motors:
                        if motor.can_id == can_id:
                            motor.update_feedback(can_data)
                            return True
        return False

    def _feedback_read_loop(self):
        """后台线程：持续读取电机反馈数据"""
        while self._reading:
            self.read_feedback(timeout_ms=100)

    def start_reading(self):
        """启动后台线程持续读取电机反馈"""
        if not self._reading:
            self._reading = True
            self._read_thread = threading.Thread(target=self._feedback_read_loop, daemon=True)
            self._read_thread.start()
            print("已启动反馈读取线程")

    def stop_reading(self):
        """停止后台读取线程"""
        if self._reading:
            self._reading = False
            if self._read_thread:
                self._read_thread.join(timeout=1)
                self._read_thread = None
            print("已停止反馈读取线程")

    def get_motor_feedback(self, motor_id: int) -> Optional[MotorFeedback]:
        """
        获取电机反馈数据
        motor_id: 电机ID
        :return: MotorFeedback或None
        """
        for motor in self.motors:
            if motor.motor_id == motor_id:
                return motor.feedback
        return None

    def stop_all_motors(self):
        """停止所有电机"""
        self._currents_1_4 = [0, 0, 0, 0]
        self._currents_5_8 = [0, 0, 0, 0]
        self.send_control_command()

# ========== 便捷接口 ==========
def create_driver(port: str = "COM3", baudrate: int = 921600) -> USBCANDriver:
    """
    创建并连接USB转CAN驱动
    :param port: 串口号
    :param baudrate: 波特率
    :return: USBCANDriver实例
    """
    driver = USBCANDriver(port, baudrate)
    if driver.connect():
        return driver
    raise ConnectionError(f"无法连接到串口 {port}")