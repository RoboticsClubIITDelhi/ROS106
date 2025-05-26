import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA

class MyFirstSub(Node):
    def __init__(self):
        super().__init__('myfirstsub')
        self.subs1 = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10
        )
        self.subs1
        

    def listener_callback(self,msg):
        self.get_logger().info('I hear: %s' %msg.data)
        

def main(args=None):
    rclpy.init(args=args)
    mysub = MyFirstSub()
    rclpy.spin(mysub)

    mysub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()








