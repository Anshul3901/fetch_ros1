import rosbag
from sensor_msgs.msg import PointCloud2

class MockCamera(object):
    def __init__(self):
        pass

    def read_cloud(self, path):
        print(f"[MockCamera] Opening bag file: {path}")
        try:
            with rosbag.Bag(path, 'r') as bag:
                for topic, msg, t in bag.read_messages():
                    msg_type = msg._type
                    print(f"  [MockCamera] Found message on topic '{topic}' with type '{msg_type}'")
                    if msg_type == 'sensor_msgs/PointCloud2':
                        print("  [MockCamera] --> Found PointCloud2 message!")
                        return msg
        except Exception as e:
            print(f"[MockCamera] ERROR reading bag: {e}")
        return None
