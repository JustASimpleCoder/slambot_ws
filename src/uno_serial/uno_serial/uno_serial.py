import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Int32
from std_msgs.msg import String
import serial
import time

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # Set up the serial connection
        self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1) # Update the port as needed
        # time.sleep(2)  # Allow time for Arduino reset
        
        qos_settings = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.odom_reading_ = self.create_publisher(
            String, 
            'odometry',
            10,                 
        )
        self.timer = self.create_timer(0.1, self.talker_callback)
        
        # Subscriber to listen for messages
        self.subscription_ = self.create_subscription(
            String,
            'motor_control',
            self.listener_callback,
            qos_settings
        )
        self.get_logger().info('SerialNode initialized and listening on /motor_control')
        
    def talker_callback(self):
        try:
            data = self.serial_port.readline().decode().strip()  
            if data:
                msg = String()
                msg.data = data
                self.get_logger().info(f'publishing data to the odom topic: {msg.data}')
                self.odom_reading_.publish()
        except Exception as e:
            self.get_logger().error(f'Error communicating with Arduino: {e}')

    def listener_callback(self, msg):
        command = msg.data
        #self.get_logger().info(f'Sending Command: {command} to Arduino')
        try:
            self.serial_port.write(f'{command}\n'.encode())  # Send pin number to Arduino
            response = self.serial_port.read()  # Read Arduino response
            #self.get_logger().info(f'Arduino responded: {response}')
        except Exception as e:
            self.get_logger().error(f'Error communicating with Arduino: {e}')

    def destroy_node(self):
        # Close the serial port when the node is destroyed
        self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialNode()
    try:
        node.get_logger().info('Starting to spin the node')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()