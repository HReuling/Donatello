#!/usr/bin/env python3
# =================================================
# edited WHS, OJ , 14.1.2022 #
#
# line_detection_sw2_with mask
# Ein Bild der raspicam vom ROS raspicam_node holen
# und den Ort der weißen Linie bestimmen
# -------------------------------------------------------
import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
import std_msgs.msg


class PiCam:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("window", 1)
        self.count = 0
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed',
                                          CompressedImage, self.image_callback,
                                          queue_size=1)
        
        self.cloud_pub = rospy.Publisher('camera/point_cloud',
                                         PointCloud,
                                         queue_size=10)
        


    def image_callback(self, msg_img):
        # converts compressed image to opencv image
        np_image_original = np.frombuffer(msg_img.data, np.uint8)
        cv2_img = cv2.imdecode(np_image_original, cv2.IMREAD_COLOR)

        # waehle eine Region of Interest an Punkt:
        # 410x308 Pixel
        img = cv2_img[460:][:]  # [y...] [x..]

        # konvertiere das Bild in Graustufen
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("ROI gray", img)
        cv2.waitKey(3)

        #ret, thresh = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY)
        #cv2.imshow('Binary image', thresh)
        #cv2.waitKey(3)

        #contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
        image_copy = img.copy()
        #cv2.drawContours(image=image_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

        #cv2.line(image_copy, (0,0), (100,10), (255,0,0), 5)

        obstacles = [[],[]]

        pixel_mapping = {0: 0.32,
                         8: 0.315,
                         20: 0.31,
                         32: 0.305}
        
        dst = cv2.Canny(img, 50, 200, None, 3)

        cdstP = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)

        # source, rho, theta, treshhold=50, minLineLength=50, maxLineGap=10 
        linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 140, None, 50, 40)
    
        if linesP is not None:

            closestLine = -1

            highestY = 0
            
            for i in range(0, len(linesP)):
                
                l = linesP[i][0]
                color = list(np.random.random(size=3) * 256)
                #cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), color, 3, cv2.LINE_AA)

                if (l[1]> highestY):
                    highestY = l[1]
                    closestLine = i
                if (l[3]> highestY):
                    highestY = l[3]
                    closestLine = i

            l = linesP[closestLine][0]

            cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), color, 3, cv2.LINE_AA)

            cloud = PointCloud()

            for i in range(0,2):

                point = Point32()
                point.z = 0.01
                # Y entspricht vom roboter aus links, x vorne
                point.x = (0.315-l[1+2*i]*0.000345)
                point.y = 0.00574-0.02356*point.x-0.000011*(640-l[0+2*i])+0.000728*(640-l[0+2*i])*point.x
                
                cloud.points.append(point)

            point = Point32()
            point.z = 0.01
            point.x = (cloud.points[0].x + cloud.points[1].x)/2
            point.y = (cloud.points[0].y + cloud.points[1].y)/2
            cloud.points.append(point)

            header = std_msgs.msg.Header()  # Leere Instanz
            header.stamp = rospy.Time.now()  # Fülle Zeitstempel
            header.frame_id = 'base_link'
            cloud.header = header
            # Senden
            self.cloud_pub.publish(cloud)

        cv2.imshow("Detected Lines (in red) - Probabilistic Line Transform", cdstP)
    

        cv2.imshow('None approximation', image_copy)

        # if (contours):
        #     #print(contours)
        #     self.count = 1
        #     for contour in contours:
        #         for value in contour:
        #             obstacles[0].append(value[0][0])
        #             obstacles[1].append(value[0][1])

        #     maxObst = max(obstacles[1])

        #     obstacles_max = [[],[]]

        #     for x, y in zip(obstacles[0],obstacles[1]) :
        #         if (y >= maxObst-5):
        #             obstacles_max[0].append(x)
        #             obstacles_max[1].append(y)

        #     #print(obstacles_max)
        #     #print(maxObst)
        #     #print(min(obstacles_max[0]))
        #     #print(max(obstacles_max[0]))
        #     cv2.line(image_copy, (min(obstacles_max[0]), maxObst), (max(obstacles_max[0]), maxObst), (255,0,0), 5)
        #     cv2.imshow('None approximation', image_copy)

        #     obstacles_m = [[],[]]

        #     cloud = PointCloud()

        #     for x, y in zip(obstacles_max[0],obstacles_max[1]):
        #         obstacles_m[0].append((x-204)*0.0009375)
        #         obstacles_m[1].append(0.3-y*0.0016667)
        #         point = Point32()
        #         point.z = 0.1
        #         point.y = -(x-204)*0.0009375
        #         point.x = (0.3-y*0.0016667)
        #         cloud.points.append(point)
        #         #print(x)
        #         #print(y)

        #     #print(obstacles_max)
        #     #print(obstacles_m)

            
        #     header = std_msgs.msg.Header()  # Leere Instanz
        #     header.stamp = rospy.Time.now()  # Fülle Zeitstempel
        #     header.frame_id = 'base_link'
        #     cloud.header = header
        #     cloud.points.append(point)
        #     # Senden
        #     self.cloud_pub.publish(cloud)

        #cv2.imshow("Img", img)
        #cv2.imshow("Img2", img2)
        #cv2.imshow("Mask", mask)


rospy.init_node('follower')
follower = PiCam()
rospy.spin()
