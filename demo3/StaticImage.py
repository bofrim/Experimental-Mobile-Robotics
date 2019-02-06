import sys
import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image

if __name__ == "__main__":
    path = sys.argv[1]
    rospy.init_node("image_publisher")
    rate = rospy.Rate(10)
    pub = rospy.Publisher("/static_image", Image, queue_size=1)
    bridge = cv_bridge.CvBridge()
    image = cv2.imread(path)
    ros_image = bridge.cv2_to_imgmsg(image, encoding="bgr8")

    while not rospy.is_shutdown():
        pub.publish(ros_image)
        rate.sleep()
