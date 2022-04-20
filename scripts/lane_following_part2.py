#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
# from visualization_msgs.msg import Marker
import cv2.aruco

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('camera/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()

                self.K = numpy.mat([[265, 0, 160], [0, 265, 120], [0, 0, 1]])
                self.distCoeffs = numpy.ndarray([0])
                self.marker_len = 10
                t = numpy.mat([0, -0.5, 0.5]).T
                n = numpy.mat([0, -1, 0])
                R = numpy.mat([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
                d = 0.115

                self.H = self.K*(R-t*n/d)*numpy.linalg.inv(self.K)

                self.last_err = 0
                self.d_count = False

        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                # hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                h, w, d = image.shape

                bev_image = cv2.warpPerspective(image, self.H, (w,h))
                bev_image = cv2.flip(bev_image, 0)
                hsv = cv2.cvtColor(bev_image, cv2.COLOR_BGR2HSV)

                # lower_yellow = numpy.array([ 10, 10, 10])
                # upper_yellow = numpy.array([255, 255, 250])

                lower_yellow = numpy.array([ 26, 43, 46])
                upper_yellow = numpy.array([ 34, 255, 250])

                lower_white = numpy.array([0, 0, 160])
                upper_white = numpy.array([180, 43, 200])
                
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)

                # h, w, d = bev_image.shape
                search_top = 2*h/3
                search_bottom = 1*h/3
                # mask1[0:search_top, 0:w] = 0
                # mask2[0:search_top, 0:w] = 0
                mask1[search_bottom:h, 0:w] = 0
                mask2[search_bottom:h, 0:w] = 0

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)


                if M1['m00'] > 0:
                    cx1 = int(M1['m10']/M1['m00'])
                    cy1 = int(M1['m01']/M1['m00'])

                    cx2 = int(M2['m10']/M2['m00'])
                    cy2 = int(M2['m01']/M2['m00'])

                    fpt_x = (cx1 + cx2)/2
                    fpt_y = (cy1 + cy2)/2 + 2*h/3

                    cv2.circle(image, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(image, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(image, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    cv2.circle(bev_image, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(bev_image, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(bev_image, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x

                    linear_x = 0.7-abs(err)*0.02 
                    self.twist.linear.x = linear_x if linear_x > 0.3 else 0.3
                    self.twist.angular.z = (err*90.0/160)/15*1.0
                    if self.d_count:
                        self.twist.angular.z = self.twist.angular.z + 0.0008*(err-self.last_err)*90.0/160

                    
                    self.cmd_vel_pub.publish(self.twist)
                    self.last_err = err
                #     self.d_count = True
                    rospy.loginfo(self.twist.linear.x)

                
                cv2.imshow("window", image)
                cv2.imshow("BEV", bev_image)
                # cv2.imshow("mask", mask1+mask2)
                cv2.waitKey(1)

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
