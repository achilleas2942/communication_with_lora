#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

# Include additional types of messages to publish


class LoRaServer(Node):
    def __init__(self):
        super().__init__('lora_server')
        self.publisher = self.create_publisher(
            String, 'lora_received_data', 10)
        self.subscription = self.create_subscription(
            String, 'lora_received_data', self.receive_from_LoRa, 10)
        self.subscription

        # Initialize serial communication
        self.serial_port = '/dev/ttyUSB0'  # Update to match the Arduino's serial port
        self.baud_rate = 115200  # Update to match the Arduino's baud rate
        self.ser = serial.Serial(self.serial_port, self.baud_rate)

    def receive_from_LoRa(self, msg):
        self.get_logger().info(f'Received data from LoRa: {msg.data}')
        # Process the received data and publish them

    def read_and_publish_data(self):
        # Read data from LoRa from the serial port
        while rclpy.ok():
            # Read data from the serial port
            received_data = self.ser.readline().decode('utf-8').strip()

            if received_data:
                self.get_logger().info(
                    f'Received data from serial port: {received_data}')

                # Publish the received data to the "lora_received_data" topic
                lora_data = String()
                lora_data.data = received_data
                self.publisher.publish(lora_data)


def main(args=None):
    rclpy.init(args=args)
    lora_server = LoRaServer()
    lora_server.read_and_publish_data()  # Start reading and publishing data
    rclpy.spin(lora_server)
    lora_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
