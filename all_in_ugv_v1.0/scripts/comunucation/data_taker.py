#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import String

SOH = 0x01
STX = 0x02
ETX = 0x03
EOT = 0x04

class UARTReceiverNode(Node):
    def __init__(self):
        super().__init__('uart_receiver_node')
        self.publisher_ = self.create_publisher(String, 'uart_data', 10)
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        if self.serial_port.in_waiting > 0:
            if self.serial_port.read(1)[0] == SOH:
                packet_length = self.serial_port.read(1)[0]
                if self.serial_port.read(1)[0] == STX:
                    data = self.serial_port.read(packet_length)
                    if self.serial_port.read(1)[0] == ETX:
                        crc1 = self.serial_port.read(1)[0]
                        crc2 = self.serial_port.read(1)[0]
                        if self.serial_port.read(1)[0] == EOT:
                            received_crc = (crc1 << 8) | crc2
                            calculated_crc = self.calculate_crc(data)
                            if received_crc == calculated_crc:
                                self.publish_data(data)

    def calculate_crc(self, data):
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if crc & 0x0001:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc

    def publish_data(self, data):
        float1, float2 = struct.unpack('ff', data)
        message = f"float1: {float1}, float2: {float2}"
        self.publisher_.publish(String(data=message))
        self.get_logger().info(f"Received data: {message}")

def main(args=None):
    rclpy.init(args=args)
    uart_receiver_node = UARTReceiverNode()
    rclpy.spin(uart_receiver_node)
    uart_receiver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()