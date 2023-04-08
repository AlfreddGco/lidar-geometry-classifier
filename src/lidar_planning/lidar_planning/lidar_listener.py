import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2

import struct
import numpy as np

BLOCK_SIZE = 32;

def unpack_data_point(block):
    x = struct.unpack('f', block[:4])[0]
    block = block[4:]
    y = struct.unpack('f', block[:4])[0]
    block = block[4:]
    z = struct.unpack('f', block[:4])[0]
    block = [] 
    return (x, y, z)


class LidarPlanning(Node):
    def __init__(self):
        super().__init__('lidar_planning')
        self.lidar_subscription = self.create_subscription(
            PointCloud2, '/lidar/points', self.lidar_listener, 10)
        self.current_frame = []


    def print_frame(self, n=3):
        text = '\n'
        for i in range(n):
            (x, y, z) = self.current_frame[i]
            text += f'[{i}]: ({x:.2f}, {y:.2f}, {z:.2f})\n'
        self.get_logger().info(text)


    def lidar_listener(self, msg: PointCloud2):
        data = msg.data
        text = 'Lidar callback | '
        text += f'width: {msg.width} height: {msg.height} '
        text += f'data_size: {len(msg.data)} B '
        next_frame = []
        for i in range(len(data)//BLOCK_SIZE):
            block = BLOCK_SIZE*i
            point = unpack_data_point(data[block:][:BLOCK_SIZE])
            next_frame.append(point)
        self.current_frame = next_frame
        self.get_logger().info(text)
        #self.print_frame(4)
        self.save_frame()
        rclpy.shutdown()


    def save_frame(self):
        np_arr = []
        for point in self.current_frame:
            x, y, z = point
            np_arr.append([x, y, z])
        np_arr = np.array(np_arr)
        with open('cloud_points.npy', 'wb') as f:
            np.save(f, np_arr)


def main(args=None):
    rclpy.init(args=args)
    planner = LidarPlanning()
    rclpy.spin(planner)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

