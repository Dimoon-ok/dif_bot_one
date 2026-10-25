#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import serial.tools.list_ports

ROBOT_ID = "ROBOT_001"
BAUD = 115200
PROBE_TIMEOUT = 3.0   # сек на каждый порт

class RobotSerialController(Node):
    def __init__(self):
        super().__init__('robot_serial_controller')

        # Подписываемся на команду скорости
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

        # # Настройка Serial порта (замените на свой путь)
        # self.serial_port = serial.Serial(
        #     "/dev/serial/by-path/pci-0000:00:14.0-usb-0:3:1.0-port0",
        #     baudrate=115200,
        #     timeout=1
        # )
        self.serial_port = self.connect_robot()
        if self.serial_port is None:
            self.get_logger().error("Робот не найден ни на одном порту")
            raise RuntimeError("Robot not found")
        

    def connect_robot(self):
        ports = sorted(
            serial.tools.list_ports.comports(),
            key=lambda p: p.vid is None   # USB-порты первыми
        )
        for p in ports:
            self.get_logger().info(f"Проверяю {p.device} ...")
            ser = None
            try:
                ser = serial.Serial()
                ser.port = p.device
                ser.baudrate = BAUD
                ser.timeout = 0.1
                ser.dtr = False   # меньше шансов перезагрузить ESP32
                ser.rts = False
                ser.open()

                deadline = time.time() + PROBE_TIMEOUT
                next_ping = 0.0
                while time.time() < deadline:
                    if time.time() >= next_ping:
                        ser.write(b"PING\n")
                        next_ping = time.time() + 0.3
                    line = ser.readline().decode(errors="ignore").strip()
                    if ROBOT_ID in line:
                        ser.reset_input_buffer()
                        self.get_logger().info(f"Робот найден на {p.device}")
                        return ser
                ser.close()   # ответа не было, идём к следующему порту
            except (serial.SerialException, OSError):
                if ser is not None and ser.is_open:
                    ser.close()
        return None

    def listener_callback(self, msg):
        linear_velocity = msg.linear.x * 1000  # м/с в мм/с
        angular_velocity = msg.angular.z  # рад/с

        command = f"SET_ROBOT_VELOCITY {linear_velocity:.2f} {angular_velocity:.2f}\n"
        self.serial_port.write(command.encode())
        self.get_logger().info(f"Sent command: {command.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotSerialController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
