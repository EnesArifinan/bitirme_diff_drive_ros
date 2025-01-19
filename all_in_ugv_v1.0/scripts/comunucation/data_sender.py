#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import struct
from std_msgs.msg import Float32MultiArray

SOH = 0x01  # Start of Heading
STX = 0x02  # Start of Text
ETX = 0x03  # End of Text
EOT = 0x04  # End of Transmission

class UARTSenderNode(Node):
    def __init__(self):
        super().__init__('uart_sender_node')
        self.get_logger().info('UARTSenderNode başlatıldı.')
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
            self.get_logger().info('Seri port açıldı: /dev/ttyUSB0')
        except serial.SerialException as e:
            self.get_logger().error(f'Seri port açılırken hata: {e}')
            return

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'motor_pwm',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.get_logger().info('Abonelik oluşturuldu: motor_pwm')

    def listener_callback(self, msg):
        try:
            float1 = msg.data[0]  # Göndermek istediğiniz ilk float
            float2 = msg.data[1]  # Göndermek istediğiniz ikinci float
            self.get_logger().info(f'Alınan veriler: float1={float1}, float2={float2}')
            
            # Verileri paketleyin
            data_packet = struct.pack('ff', float1, float2)
            crc = self.calculate_crc(data_packet)
            self.get_logger().info(f'Hesaplanan CRC: {crc:#04x}')
            
            # Veriyi UART üzerinden gönderin
            self.send_uart_packet(data_packet, crc)
            self.get_logger().info('Veri başarıyla gönderildi.')
        except Exception as e:
            self.get_logger().error(f'Listener callback içinde hata: {e}')

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
        self.get_logger().info(f'CRC hesaplandı: {crc}')
        return crc

    def send_uart_packet(self, data_packet, crc):
        try:
            packet_length = len(data_packet)
            
            packet = bytearray()
            packet.append(SOH)                   # Başlık
            packet.append(packet_length)         # Paket uzunluğu
            packet.append(STX)                   # Metin başlangıcı
            packet.extend(data_packet)           # Veri
            packet.append(ETX)                   # Metin sonu
            packet.append((crc >> 8) & 0xFF)     # CRC'nin üst byte'ı
            packet.append(crc & 0xFF)            # CRC'nin alt byte'ı
            packet.append(EOT)                   # İletim sonu
            
            self.serial_port.write(packet)       # Veriyi gönder
            self.get_logger().info(f'Paket gönderildi: {packet}')
        except Exception as e:
            self.get_logger().error(f'Paket gönderiminde hata: {e}')

def main(args=None):
    rclpy.init(args=args)
    uart_sender_node = UARTSenderNode()
    rclpy.spin(uart_sender_node)
    uart_sender_node.destroy_node()
    rclpy.shutdown()

if __name__== '__main__':
    main()