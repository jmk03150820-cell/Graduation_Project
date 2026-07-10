import time

import rclpy
from cits_rsu_msgs.msg import DetectedObject, DetectedObjectArray
from rclpy.node import Node

from fake_rsu.rsu_object_generator import generate_objects


class FakeRsuObjectsNode(Node):
    """Publishes fake RSU-detected objects matching the predicted
    cits_rsu_msgs/DetectedObjectArray schema (the C-ITS teammate's field
    table), so cits_adapter can be developed/tested before their real
    publisher exists.
    """

    def __init__(self):
        super().__init__('fake_rsu_objects_node')

        self.declare_parameter('topic', 'cits/rsu/detected_objects')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('frame_id', 'rsu')

        topic = self.get_parameter('topic').value
        rate_hz = self.get_parameter('publish_rate_hz').value
        self._frame_id = self.get_parameter('frame_id').value

        self.pub = self.create_publisher(DetectedObjectArray, topic, 10)
        self.create_timer(1.0 / rate_hz, self._on_timer)

        self.get_logger().info(f'fake_rsu_objects_node started: publishing on {topic}')

    def _on_timer(self):
        now = time.time()
        msg = DetectedObjectArray()
        msg.header.frame_id = self._frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        objects = []
        for obj_dict in generate_objects(now):
            obj = DetectedObject()
            obj.object_id = obj_dict['object_id']
            obj.object_type = obj_dict['object_type']
            obj.position_x = float(obj_dict['position_x'])
            obj.position_y = float(obj_dict['position_y'])
            obj.velocity = float(obj_dict['velocity'])
            obj.timestamp = obj_dict['timestamp']
            obj.confidence = float(obj_dict['confidence'])
            obj.heading = float(obj_dict['heading'])
            objects.append(obj)

        msg.objects = objects
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeRsuObjectsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
