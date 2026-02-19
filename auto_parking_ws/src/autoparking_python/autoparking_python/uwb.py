import socket 
import rclpy 
from rclpy.node  import Node 
# from std_msgs.msg  import String  # 假设发布字符串消息 
import ast
from std_msgs.msg  import Float32MultiArray
from rclpy.executors import MultiThreadedExecutor

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
 
##########################################
# # 改进，还没测试：
# import re 
 
# # 预编译正则表达式（效率提升30%）
# COORD_PATTERN = re.compile( 
#     r'X:([+-]?\d+\.?\d*),Y:([+-]?\d+\.?\d*),Z:([+-]?\d+\.?\d*)', 
#     re.IGNORECASE  # 支持大小写混写 
# )
 
# ID_PATTERN = re.compile(r'ID:(\d+)')   # 标签ID提取 
 
# def parse_udp_message(msg: str) -> dict:
#     """安全解析UDP消息为结构化数据"""
#     result = {'id': None, 'x': None, 'y': None, 'z': None}
    
#     # 提取标签ID 
#     if id_match := ID_PATTERN.search(msg): 
#         result['id'] = int(id_match.group(1)) 
    
#     # 提取坐标 
#     if coord_match := COORD_PATTERN.search(msg): 
#         result.update(zip(('x',  'y', 'z'), map(float, coord_match.groups()))) 
    
#     return result 

# def udp_receive_callback(self):
#     try:
#         # 带超时的非阻塞接收（防止线程冻结）
#         data, addr = self.udp_socket.recvfrom(1024) 
#         msg_str = data.decode('utf-8',  errors='replace')  # 容错解码 
        
#         parsed = parse_udp_message(msg_str)
        
#         # 数据完整性校验 
#         if None in parsed.values(): 
#             missing = [k for k,v in parsed.items()  if v is None]
#             self.get_logger().error(f" 字段缺失：{missing}")
#             return 
            
#         # 数值范围校验（示例：-100~100米）
#         if not all(-100 <= v <= 100 for v in (parsed['x'], parsed['y'], parsed['z'])):
#             self.get_logger().warn(f" 坐标超限：{parsed}")
#             return 
            
#         # 类型安全转换 
#         msg = Float32MultiArray()
#         msg.data  = [parsed['id'], parsed['x'], parsed['y'], parsed['z']]
#         self.publisher.publish(msg) 
        
#     except socket.timeout: 
#         self.get_logger().debug(" 接收超时")
#     except UnicodeDecodeError as e:
#         self.get_logger().error(f" 编码错误：{e.object[e.start:e.end]}") 
#     except Exception as e:
#         self.get_logger().critical(f" 未处理异常：{str(e)}", exc_info=True)
##########################################

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