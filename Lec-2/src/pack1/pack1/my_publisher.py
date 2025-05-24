import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA

class MyFirstPub(Node):
    def __init__(self):
        super().__init__('myfirstpub')
        self.publisher1 = self.create_publisher(String,'topic',10) 
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' %self.i
        self.publisher1.publish(msg)
        self.get_logger().info('Publishing : %s' %msg.data)
        self.i +=1

def main(args=None):
    rclpy.init(args=args)
    mypub = MyFirstPub()
    rclpy.spin(mypub)

    mypub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()








