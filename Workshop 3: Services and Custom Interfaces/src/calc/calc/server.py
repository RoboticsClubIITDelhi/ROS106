from customs.srv import Calc
import rclpy
from rclpy.node import Node

class Service_node(Node):
    def __init__(self):
        super().__init__('server_node')
        self.srv=self.create_service(Calc,'calculate',self.service_callback)

    def service_callback(self,request,response):
        operation_msg=request.o
        n1 = operation_msg.n1
        n2 = operation_msg.n2
        operation_name=operation_msg.on 
        operation_name=operation_name.lower()
        self.get_logger().info(f"perform {operation_name} on {n1} and {n2}")
        if(operation_name=='add'):
            final_res=n1+n2
            response.res=final_res
            response.success=True
            response.valid=True
        elif(operation_name=='subt'):
            final_res=n1-n2
            response.res=final_res
            response.success=True
            response.valid=True
        elif(operation_name=='mul'):
            final_res=n1*n2
            response.res=final_res
            response.success=True
            response.valid=True
        elif(operation_name=='div'):
            if(n2==0):
                response.success=False
            else:
            
                final_res=n1+n2
                response.res=final_res
                response.success=True
                response.valid=True
        else:
            response.valid=False
            response.success=True
        return response
def main(args=None):
    rclpy.init(args=args)
    service_node = Service_node()
    rclpy.spin(service_node)
    rclpy.shutdown()

if __name__=='__main__':
    main()

        