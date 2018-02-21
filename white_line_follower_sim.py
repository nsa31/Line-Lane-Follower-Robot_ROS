#!/usr/bin/env python
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

global perr, ptime, serr, dt,move
perr = 0
ptime = 0
serr = 0
dt = 0

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    cv2.namedWindow("win1", 1)
    self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                       Twist, queue_size=1)
    self.twist = Twist()

  	
  def image_callback(self, msg):
    global perr, ptime, serr, dt
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_yellow = numpy.array([ 10,  10,  10])
    upper_yellow = numpy.array([255, 255, 250])

    lower_white = numpy.array([100,100,200], dtype= "uint8")
    upper_white = numpy.array([255,255,255], dtype= "uint8")

    #threshold to get only white
    maskw = cv2.inRange(image, lower_white, upper_white)
    masky = cv2.inRange(hsv, lower_yellow, upper_yellow)

    mask_yw = cv2.bitwise_or(maskw, masky)    
    rgb_yw = cv2.bitwise_and(image, image, mask = mask_yw).astype(numpy.uint8)

    rgb_yw = cv2.cvtColor(rgb_yw, cv2.COLOR_RGB2GRAY)    
    
    kernel = numpy.ones((7,7), numpy.uint8)
    opening = cv2.morphologyEx(rgb_yw, cv2.MORPH_OPEN, kernel)
    rgb_yw2 = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)		
	
    h, w= rgb_yw2.shape 
    search_top = 7*h/8 #camera look directly beneath 
    search_bot = 7*h/8 + 20
    rgb_yw2[0:search_top, 0:w] = 0
    rgb_yw2[search_bot:h, 0:w] = 0
    M = cv2.moments(rgb_yw2)
    c_time = rospy.Time.now()
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(rgb_yw2, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - 1*w/2
      self.twist.linear.x = 1
      dt = rospy.get_time() - ptime
      self.twist.angular.z = (-float(err) / 100)*1 + ((err - perr)/(rospy.get_time() - 		ptime))*1/15/100 #+ (serr*dt)*1/20/100 #1 is best, starting 3 unstable
      serr = err + serr
      perr = err
      ptime = rospy.get_time()
      
      self.cmd_vel_pub.publish(self.twist)

      # END CONTROL
    cv2.imshow("win1", rgb_yw2)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
