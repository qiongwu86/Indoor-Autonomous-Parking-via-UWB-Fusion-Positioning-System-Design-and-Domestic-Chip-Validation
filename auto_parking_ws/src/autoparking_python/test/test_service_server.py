import rclpy                                     # ROS2 Python接口库
from rclpy.node   import Node                    # ROS2 节点类
from example_interfaces.srv  import AddTwoInts    # 自定义的服务接口
from example_interfaces.msg import String

class adderServer(Node):
    def __init__(self, name):
        super().__init__(name)                                                           # ROS2节点父类初始化
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.adder_callback)  # 创建服务器对象（接口类型、服务名、服务器回调函数）
        self.on_parking = 0

        # self.timer = self.create_timer(0.5, self.timer_callback)            # 创建一个定时器（单位为秒的周期，定时执行的回调函数）

    def timer_callback(self):
        self.get_logger().info(f'当前id:{self.on_parking}')

    def adder_callback(self, request, response):   # 创建回调函数，执行收到请求后对数据的处理
        response.sum = request.a + request.b       # 完成加法求和计算，将结果放到反馈的数据中
        self.on_parking = response.sum
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))   # 输出日志信息，提示已经完成加法求和计算
        return response                          # 反馈应答信息

def main(args=None):                             # ROS2节点主入口main函数
    rclpy.init(args=args)                        # ROS2 Python接口初始化
    node = adderServer("service_adder_server")   # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                             # 循环等待ROS2退出
    node.destroy_node()                          # 销毁节点对象
    rclpy.shutdown()                             # 关闭ROS2 Python接口

if __name__=='__main__':
  main()