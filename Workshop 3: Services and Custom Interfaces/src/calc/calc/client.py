import sys
from customs.msg import Operation
from customs.srv import Calc
import rclpy
from rclpy.node import Node

class Client_node(Node):
    def __init__(self):
        super().__init__('client')
        self.cli=self.create_client(Calc,'calculate')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service calculate")
        self.req=Calc.Request()
    #f1,f2,f3
    def send_req(self,n1,n2,on):
        self.req.o.n1=float(n1)
        self.req.o.n2=float(n2)
        self.req.o.on=str(on)

        self.get_logger().info(f"User gave numbers {n1} and {n2} and asked to do operation {on}")
        future = self.cli.call_async(self.req)
        return future
def main(args=None):
    rclpy.init(args=args)
    client_node = Client_node()
    future = client_node.send_req(float(sys.argv[1]),float(sys.argv[2]),str(sys.argv[3]))
    rclpy.spin_until_future_complete(client_node,future)

    if future.done():
        response=future.result()
        if (response is not None and response.success):
            if(response.valid):
                client_node.get_logger().info(f"Result is {response.res}")
            else:
                client_node.get_logger().info("Incorrect operation type")
        else:
            if(response is None):
                client_node.get_logger().info("No response from service")
            else:
                client_node.get_logger().info("Operation correct but cant be performed on the given numbers")
        client_node.destroy_node()
        rclpy.shutdown()
    
if __name__=='__main__':
    main()

    

