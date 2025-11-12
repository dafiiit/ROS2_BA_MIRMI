import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class FakeCameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('fake_camera_info_publisher')
        self.publisher_ = self.create_publisher(CameraInfo, '/camera/camera_info', 10)
        self.timer = self.create_timer(0.5, self.publish_info)
        self.get_logger().info('Fake CameraInfo Publisher started.')

    def publish_info(self):
        msg = CameraInfo()
        msg.header.frame_id = 'camera_link'
        msg.width = 640
        msg.height = 480
        msg.k = [554.256, 0.0, 320.5,
                 0.0, 554.256, 240.5,
                 0.0, 0.0, 1.0]
        msg.p = [554.256, 0.0, 320.5, 0.0,
                 0.0, 554.256, 240.5, 0.0,
                 0.0, 0.0, 1.0, 0.0]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeCameraInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

