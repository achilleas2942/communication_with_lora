#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

# Include additional types of messages to publish


class LoRaClient(Node):
    def __init__(self):
        super().__init__('lora_client')
        self.subscription = self.create_subscription(
            String, 'ros_data_to_transmit', self.send_to_LoRa, 10)
        self.subscription

        # Initialize serial communication
        self.serial_port = '/dev/ttyUSB0'  # Update to match the Arduino's serial port
        self.baud_rate = 115200  # Update to match the Arduino's baud rate
        self.ser = serial.Serial(self.serial_port, self.baud_rate)

    def send_to_LoRa(self, msg):
        # Send data to LoRa from the serial port
        lora_data = msg.data
        self.get_logger().info(f'Sending data to LoRa: {lora_data}')
        self.ser.write(lora_data.encode('utf-8') + b'\n')


def main(args=None):
    rclpy.init(args=args)
    lora_client = LoRaClient()
    rclpy.spin(lora_client)
    lora_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
