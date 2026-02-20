from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraStream(Node):
    def __init__(self, namespace, serial_no):
        super().__init__('camera_node', namespace=namespace)
        self.bridge = CvBridge()
        
        # Subscribe to the image topic
        self.image_sub = self.create_subscription(
            Image,
            f'/{namespace}/color/image_raw',
            self.image_callback,
            10 # QoS History depth
        )
        self.current_frame = None

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        self.current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Display the stream
        cv2.imshow(f"Stream: {self.get_namespace()}", self.current_frame)
        cv2.waitKey(1) # Necessary to refresh the window