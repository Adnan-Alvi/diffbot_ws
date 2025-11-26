import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisherNode(Node):
    def __init__(self):
        super().__init__("simple_publisher")
        self.frequency_ = 1.0
        self.counter_ = 0.0
        self.simple_publisher = self.create_publisher(String, "chatter", 10)
        self.timer_ = self.create_timer(self.frequency_, self.publishCallback)

    def publishCallback(self):
        msg = String()
        msg.data = "Hi Ros2 : counter " + str(self.counter_)
        self.counter_ += 1.0
        self.simple_publisher.publish(msg)

def main():
    rclpy.init()
    simple_publisher_node = SimplePublisherNode()
    rclpy.spin(simple_publisher_node)
    simple_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


