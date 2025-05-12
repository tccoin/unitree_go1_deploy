import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import roslibpy
import base64


class MultiTopicBridge(Node):
    def __init__(self):
        super().__init__('rosbridge_multi_topic_relay')
        self.color_frame_count = 0
        self.depth_frame_count = 0

        # publisher
        self.pub_color_compressed = self.create_publisher(CompressedImage, '/relay/rgb/compressed', 1)
        self.pub_depth_compressed = self.create_publisher(CompressedImage, '/relay/depth/compressed', 1)
        self.pub_odom = self.create_publisher(Odometry, '/relay/pose/odom', 1)

        # rosbridge websocket connect
        self.ros_client = roslibpy.Ros(host='35.3.98.240', port=9090)
        self.ros_client.run()

        # subscribe rgb image
        self.topic_color = roslibpy.Topic(self.ros_client,
                                          '/camera/color/image_raw/compressed',
                                          'sensor_msgs/CompressedImage',
                                          queue_length=1,
                                          throttle_rate=100)
        self.topic_color.subscribe(self.cb_color)

        # subscribe depth
        self.topic_depth = roslibpy.Topic(self.ros_client,
                                          '/camera/depth/image_rect_raw/compressedDepth',
                                          'sensor_msgs/CompressedImage',
                                          queue_length=1,
                                          throttle_rate=100)
        self.topic_depth.subscribe(self.cb_depth)

        # sub Odometry
        self.topic_odom = roslibpy.Topic(self.ros_client,
                                         '/camera/pose/sample',
                                         'nav_msgs/Odometry',
                                         queue_length=1)
        self.topic_odom.subscribe(self.cb_odom)

    def cb_color(self, msg):
        self.pub_color_compressed.publish(self._to_compressed_image(msg, 'camera_color_frame'))

    def cb_depth(self, msg):
        self.pub_depth_compressed.publish(self._to_compressed_image(msg, 'camera_depth_frame'))

    def cb_odom(self, msg):
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = msg['header'].get('frame_id', 'camera_frame')
        odom_msg.child_frame_id = msg.get('child_frame_id', '')

        pose = msg['pose']['pose']
        odom_msg.pose.pose.position.x = pose['position']['x']
        odom_msg.pose.pose.position.y = pose['position']['y']
        odom_msg.pose.pose.position.z = pose['position']['z']
        odom_msg.pose.pose.orientation.x = pose['orientation']['x']
        odom_msg.pose.pose.orientation.y = pose['orientation']['y']
        odom_msg.pose.pose.orientation.z = pose['orientation']['z']
        odom_msg.pose.pose.orientation.w = pose['orientation']['w']

        twist = msg['twist']['twist']
        odom_msg.twist.twist.linear.x = twist['linear']['x']
        odom_msg.twist.twist.linear.y = twist['linear']['y']
        odom_msg.twist.twist.linear.z = twist['linear']['z']
        odom_msg.twist.twist.angular.x = twist['angular']['x']
        odom_msg.twist.twist.angular.y = twist['angular']['y']
        odom_msg.twist.twist.angular.z = twist['angular']['z']

        self.pub_odom.publish(odom_msg)

    def _to_compressed_image(self, msg, frame_id='camera_frame'):
        image_msg = CompressedImage()
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = frame_id
        image_msg.format = msg['format']
        image_msg.data = base64.b64decode(msg['data'])
        return image_msg

    def destroy_node(self):
        self.topic_color.unsubscribe()
        self.topic_depth.unsubscribe()
        self.topic_odom.unsubscribe()
        self.ros_client.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiTopicBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
