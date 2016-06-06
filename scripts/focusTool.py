#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np;

class focusTool:

  def __init__(self, image_topic, focus_region = (100,100)):
    #self.image_pub = rospy.Publisher("/camera23/image_raw",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(image_topic,Image,self.callback)
    self.image_topic = image_topic;
    self.focus_region = focus_region;

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    focus_region = self.focus_region #(width, height) to clean up
    newImage = cv2.GaussianBlur(cv_image, (5,5), 0);
    height, width = newImage.shape[:2];

    subImg = newImage[height/2-focus_region[1]/2 : height/2+focus_region[1]/2, width/2 - focus_region[0]/2 : width/2 + focus_region[0]/2];

    cv2.rectangle(newImage,(width/2 - focus_region[0]/2, height/2-focus_region[1]/2),  (width/2 + focus_region[0]/2, height/2+focus_region[1]/2), 255);

    subImg64f = cv2.Laplacian(subImg, cv2.CV_64F);
    abs_subImg64f = np.absolute(subImg64f);
    total = np.sum(abs_subImg64f);
    subImg8u = np.uint8(abs_subImg64f);

    #just make it bigger so it can be seen more easily
    subImg8u = cv2.resize(subImg8u, (500,400));
 
    cv2.putText(subImg8u, str(total), (33,33), cv2.FONT_HERSHEY_COMPLEX, 1, (255,255,255) );
 
    cv2.imshow(str(self.image_topic) + "Image", newImage)
    cv2.imshow(str(self.image_topic) + "Sub-Image Laplacian", subImg8u)
    cv2.waitKey(3)

def main(args):
  image_topics = ["/camera2/image_raw/"];
  ft = [];
  for image_topic in image_topics: 
    ft.append(focusTool(image_topic));
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
