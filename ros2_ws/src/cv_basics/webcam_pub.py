import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        Gst.init(None)

    def timer_callback(self):
        pipeline = Gst.parse_launch('v4l2src device=/dev/video2 ! video/x-raw,width=640,height=480 ! videoconvert ! x264enc ! rtph264pay ! appsink name=sink')
        sink = pipeline.get_by_name('sink')
        sample = sink.emit('pull-sample')
        buffer = sample.get_buffer()
        data = buffer.extract_dup(0, buffer.get_size())
        msg = Image()
        msg.data = data
        msg.width = 320
        msg.height = 240
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()