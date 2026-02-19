import socket 
import rclpy 
from rclpy.node  import Node 
# from std_msgs.msg  import String  # 假设发布字符串消息 
import ast
from std_msgs.msg  import Float32MultiArray
from rclpy.executors import MultiThreadedExecutor

"""
基于dw1000
"""
class UWBUDP(Node):
    def __init__(self):
        super().__init__('uwb1_node')
        # 创建ROS发布者 
        self.publisher  = self.create_publisher(Float32MultiArray,  '/uwb_data', 10)
        
        # 初始化UDP Socket 
        self.udp_socket  = socket.socket(socket.AF_INET,  socket.SOCK_DGRAM)
        self.udp_socket.bind(('', 8080))  # 绑定本地端口 
        self.udp_socket.settimeout(0.001)   # 非阻塞模式 
        
        # 启动UDP接收线程 
        self.timer  = self.create_timer(0.001,  self.udp_receive_callback) 
 
    def udp_receive_callback(self):
        try:
            rs_data = self.udp_socket.recvfrom(1024)   # 接收数据 
            rs_masg = rs_data[0]
            msg = rs_masg.decode('utf-8')

            x = ''
            y = ''
            z = ''

            j = 0

            for i in msg:           # 第40位：标签ID

                j = j+1
                if i == 'X':        # 第49位：X
                    n = j + 3
                    while msg[n] != ',':
                        x = x + msg[n]
                        n = n + 1

                if i == 'Y':        # 第49位：X
                    n = j + 3
                    while msg[n] != ',':
                        y = y + msg[n]
                        n = n + 1
                if i == 'Z':        # 第49位：X
                    n = j + 3
                    while msg[n] != ',':
                        z = z + msg[n]
                        n = n + 1

            id = float(msg[40])
            x = float(ast.literal_eval(x))
            y = float(ast.literal_eval(y))
            z = float(ast.literal_eval(z))

            msg = Float32MultiArray()
            # self.get_logger().info(f"Published:  {type(x)}")
            msg.data  = [id, x, y, z]
            self.publisher.publish(msg) 
            # self.get_logger().info(f"Published:  {msg.data}") 

        except socket.timeout: 
            pass  # 无数据时跳过 
 
def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor() 
    node = UWBUDP()
    rclpy.spin(node,executor=executor) 
    node.udp_socket.close()
    node.destroy_node()
    rclpy.shutdown() 
 
if __name__ == '__main__':
    main()