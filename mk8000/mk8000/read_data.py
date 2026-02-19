import serial
import time
from filter_dis import *
import ctypes

"""
可以用MK8000进行定位的代码
"""
class FrameParser:
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        # 初始化串口
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        print(f"Serial port opened: {self.ser.port}, baudrate: {baudrate}, timeout: {timeout}s")

        self.header = 0xf0
        self.tail = 0xaa
        self.payload_len = 0x05

    def read_frame(self):
        """
        读取并解析一帧数据，返回有效载荷字节列表
        如果帧格式不正确，则丢弃并继续读取
        """
        while True:
            # 寻找帧头
            byte = self.ser.read(2)
            # print(byte)
            if not byte:
                # 超时或无数据，继续等待
                print("Timeout waiting for header, retrying...")  # 可启用调试
                continue

            b = byte[0]
            # print(f"Read byte: {hex(b)}")  # 可启用调试
            if b != self.header:
                # 不是帧头，继续读取
                continue

            # 读取有效数据
            payload = self.ser.read(self.payload_len)
            if len(payload) != self.payload_len:
                # 数据不足，丢弃
                continue

            # 读取帧尾
            tail_byte = self.ser.read(1)
            # print(tail_byte)
            if not tail_byte or tail_byte[0] != self.tail:
                # 帧尾不匹配，丢弃
                continue

            # 成功读取一帧
            return list(payload)

    def close(self):
        print("Closing serial port...")
        self.ser.close()

def parse_payload(payload):
    """
    解析有效载荷字节：
    payload[0]: 发送地址低位
    payload[1]: 发送地址高位
    payload[2]: 距离低位
    payload[3]: 距离高位
    payload[4]: 信号强度

    返回：address, distance, signal_strength（均为十进制整数）
    """
    addr_lo, addr_hi, dist_lo, dist_hi, signal = payload
    address = (addr_hi << 8) | addr_lo
    distance = (dist_hi << 8) | dist_lo
    return address, distance, signal


if __name__ == '__main__':
    # 配置串口
    parser = FrameParser(port='/dev/ttyUSB0', baudrate=115200, timeout=1)

    # 定位算法初始配置
    # so文件在当前目录下，如trilatertion.so文件在/usr/lib下，请修改对应路径
    lib_so = ctypes.cdll.LoadLibrary('/home/www/test_ws/src/mk8000/mk8000/trilateration.so')
    result = 0

    class UWBMsg(ctypes.Structure):
        _fields_ = [("x", ctypes.c_double),
                    ("y", ctypes.c_double),
                    ("z", ctypes.c_double)]

    location = UWBMsg()
    anchorArray = (UWBMsg*8)()
    distanceArray = (ctypes.c_int*8)(-1)

    # 配置基站坐标
    anchorArray[0].x = 0
    anchorArray[0].y = 0
    anchorArray[0].z = 1.84

    anchorArray[1].x = 14.25
    anchorArray[1].y = 0
    anchorArray[1].z = 1.84

    anchorArray[2].x = 9.8
    anchorArray[2].y = 13.73
    anchorArray[2].z = 1.82

    anchorArray[3].x = 0
    anchorArray[3].y = 0.3
    anchorArray[3].z = 2

    # 无效的测距值一定给 -1，否则会用随机数进行运算
    # 毫米
    distanceArray[0] = 0
    distanceArray[1] = 0
    distanceArray[2] = 0
    distanceArray[3] = -1
    distanceArray[4] = -1
    distanceArray[5] = -1
    distanceArray[6] = -1
    distanceArray[7] = -1

    # 创建卡尔曼滤波
    kf = KalmanFilter(max_anchors=3, max_tags=1, Q=0.018, R=0.542)

    try:
        while True:
            frame = parser.read_frame()
            # 仅当成功读取到一帧时才处理
            if frame:
                # data = [hex(b) for b in frame]
                # print("Received payload:", data)
                # print("Received distance:", data[3],data[2])
                address, distance, signal = parse_payload(frame)
                # 打印十进制数值
                # 目标地址（1,2,3），距离cm，信号强度
                print(f"Address: {address}, Distance: {distance/100.0}m, Signal Strength: {signal-256}dBm")
                filtered_dis = kf.filter(distance/100.0, anc_idx=address-1, tag_idx=0)      # m
                distanceArray[address-1] = int(filtered_dis*1000)                           # mm
            
            print(distanceArray[0])
            print(distanceArray[1])
            print(distanceArray[2])
            # 定位
            # # result = so.GetLocation(ctypes.byref(location),ctypes.c_int(1), anchorArray, distanceArray)
            result = lib_so.GetLocation(ctypes.byref(location), anchorArray, distanceArray)

            print(location.x)
            print(location.y)
            print(location.z)

            print(result)

            # 防止CPU占用过高
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        parser.close()
