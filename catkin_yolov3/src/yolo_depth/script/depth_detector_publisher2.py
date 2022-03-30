#!/usr/bin/env python
# -*- coding: utf-8 -*-

# This import is for general library
import os
import threading

# This import is for ROS integration
import rospy
from sensor_msgs.msg import Image,CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from darknet_ros_msgs.msg import BoundingBoxes,BoundingBox
import cv2
from std_msgs.msg import UInt16MultiArray #,Float32


class PersonDetector():
    probability = 0.0
    pre_probability = 0.0
    persondepth = 0
    persondetect = 0
    flag = False
    Hello = False

    def __init__(self):

        # cv_bridge handles
        self.cv_bridge = CvBridge()

        self.person_bbox = BoundingBox()
        self.m_person_depth = 0

        # ROS PARAM
        self.m_pub_threshold = rospy.get_param('~pub_threshold', 0.30)

        # Subscribe
        sub_camera_rgb     =  rospy.Subscriber('/camera/color/image_raw', Image, self.CamRgbImageCallback)
        sub_camera_depth   =  rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.CamDepthImageCallback)
        sub_darknet_bbox   =  rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.DarknetBboxCallback)

        return

    def CamRgbImageCallback(self, rgb_image_data):
        try:
            rgb_image = self.cv_bridge.imgmsg_to_cv2(rgb_image_data, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)

        rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
        PersonDetector.probability = self.person_bbox.probability
        PersonDetector.flag = True


        # 사람인식시
        if self.person_bbox.probability > 0.0 :

            PersonDetector.persondetect = 1
           # 일단 바운딩박스의 중심위치 depth 를 측정(향후 개선필요 - 정확도 up) 
            self.m_person_depth = self.m_depth_image[(int)(self.person_bbox.ymax+self.person_bbox.ymin)/2][(int)(self.person_bbox.xmax+self.person_bbox.xmin)/2]
            PersonDetector.persondepth = self.m_person_depth
            cv2.rectangle(rgb_image, (self.person_bbox.xmin, self.person_bbox.ymin), (self.person_bbox.xmax, self.person_bbox.ymax),(0,0,255), 3)
            rospy.loginfo('Class : person, Score: %.2f, Dist: %dmm ' %(self.person_bbox.probability, self.m_person_depth))
            text = "person " +('%dmm' % self.m_person_depth)
            text_top = (self.person_bbox.xmin, self.person_bbox.ymin - 10)
            text_bot = (self.person_bbox.xmin + 80, self.person_bbox.ymin + 5)
            text_pos = (self.person_bbox.xmin + 5, self.person_bbox.ymin)
            cv2.rectangle(rgb_image, text_top, text_bot, (0,0,255),4)
            cv2.putText(rgb_image, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 1)

        else:
            #self.person_bbox.probability <= 0.0
            PersonDetector.persondetect = 0
            PersonDetector.probability = 0
            PersonDetector.persondepth = 0
            rospy.loginfo('Class : None, Score: 0, Dist: 0mm ')

        RGBIMG = cv2.resize(rgb_image, dsize=(940, 530))
        cv2.namedWindow("Object_Depth_Detector", flags=cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow("Object_Depth_Detector", 940, 530)
        cv2.imshow("Object_Depth_Detector", RGBIMG)
        cv2.waitKey(10)

        #m_depth_image_color = self.m_depth_image
        #cv2.normalize(m_depth_image_color, m_depth_image_color, 0, 2048, cv2.NORM_MINMAX) #32768
        DEPTHIMG = cv2.resize(self.m_depth_image, dsize=(940,530))
        cv2.namedWindow("Object_Depth_Image", flags=cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow("Object_Depth_Image", 0, 530)
        cv2.imshow("Object_Depth_Image", DEPTHIMG)
        cv2.waitKey(10)

        STATUSINDICATOR = np.full((470,940,3),(0, 0, 0), np.uint8)
        text1 = "Person Detection : " + ('%d(1=T, 0=F)' % PersonDetector.persondetect)
        text2 = "Person Probability : " + ('%.4f' % (PersonDetector.probability * 100)) + ("%")
        text3 = "Person Distance : " + ('%dmm' %PersonDetector.persondepth)
        cv2.putText(STATUSINDICATOR, text1, (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(STATUSINDICATOR, text2, (50,100), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 1, cv2.LINE_AA)
        cv2.putText(STATUSINDICATOR, text3, (50,150), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255,255,255), 1, cv2.LINE_AA)
        cv2.namedWindow("Status_Indicator", flags=cv2.WINDOW_AUTOSIZE)
        cv2.moveWindow("Status_Indicator", 1000, 0)
        cv2.imshow("Status_Indicator", STATUSINDICATOR)
        cv2.waitKey(10)

        return


    def CamDepthImageCallback(self, depth_image_data):
        try:
            self.m_depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image_data, 'passthrough')
        except CvBridgeError, e:
            rospy.logerr(e)
        self.m_camdepth_height, self.m_camdepth_width = self.m_depth_image.shape[:2]
        return

    def DarknetBboxCallback(self, darknet_bboxs):
        bboxs = darknet_bboxs.bounding_boxes
        person_bbox = BoundingBox()
        if len(bboxs) != 0 :
            for i, bb in enumerate(bboxs) :
                if bboxs[i].Class == 'person' and bboxs[i].probability >= self.m_pub_threshold:
                    person_bbox = bboxs[i]
        self.person_bbox = person_bbox

            

def PublishProcessedData():
    rate = rospy.Rate(20)
    pub = rospy.Publisher('ProcessedData', UInt16MultiArray, queue_size=10)
    while not rospy.is_shutdown():
        if PersonDetector.flag is True:
            data_to_send = UInt16MultiArray()
            data_to_send.data = [0,0,0]
            data_to_send.data[0] = PersonDetector.persondetect
            data_to_send.data[1] = int(PersonDetector.probability * 100)
            data_to_send.data[2] = PersonDetector.persondepth
            rospy.loginfo(data_to_send)
            pub.publish(data_to_send)
            PersonDetector.flag = False
        rate.sleep()




if __name__ == '__main__':
    try:
        rospy.init_node('person_detector', anonymous=True)
        Detector = threading.Thread(target=PersonDetector)
        Publisher = threading.Thread(target=PublishProcessedData)
        rospy.loginfo('Detector Initialized')
        rospy.loginfo('Publisher Initialized')
        Detector.start()
        Publisher.start()
        rospy.spin()
        #Detector.finish()
	#Publisher.finish()

    except rospy.ROSInterruptException:
        pass
