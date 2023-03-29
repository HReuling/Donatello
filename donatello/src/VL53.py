#!/usr/bin/env python3

import rospy
import std_msgs.msg
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, LaserScan
from sensor_msgs.msg import PointCloud  # Message für die Sonar-Hindernisse
from nav_msgs.msg import Path

dist_max = 0.2
dist_min = 0.05
dist_critical = 0.15
turn = rospy.Duration(1)
forward = rospy.Duration(0)
max_change = 5

middle_length = 5


class VL53_to_Point_Cloud():
    def __init__(self):
        rospy.loginfo("Publishing PointCloud")

        self.cloud_pub = rospy.Publisher('VL53/point_cloud',
                                         PointCloud,
                                         queue_size=10)
        self.twist = Twist()
        
        self.cmd_pub = rospy.Publisher('obst_vel', Twist, queue_size=10)

        # receiving sonar_left and sonar_right
        self.sonar_sub_left = rospy.Subscriber('VL53_left',
                                               Range,
                                               self.get_sonar_left,
                                               queue_size=1)
        self.sonar_sub_right = rospy.Subscriber('VL53_right',
                                                Range,
                                                self.get_sonar_right,
                                                queue_size=1)
        
        self.scan_sub = rospy.Subscriber('scan',LaserScan,
                                                self.get_min_scan,
                                                queue_size=1)
        self.dist_left = 0.0
        self.dist_right = 0.0
        self.dist_left_arr = [0]
        self.dist_right_arr = [0]
        self.dist_left_before = 0.0
        self.dist_right_before = 0.0
        self.busy = False
        self.scan = False
        self.point_cloud = PointCloud()
        self.scan_min = 0
        
        # Senden
        self.cloud_pub.publish(self.point_cloud)

        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.rate.sleep()

    def get_sonar_left(self, sensor_data_left):
        self.dist_left_before = self.dist_left
        if sensor_data_left.range > dist_min:
            self.dist_left_arr.append(sensor_data_left.range)
            if len(self.dist_left_arr)>middle_length:
                self.dist_left_arr.pop(0)
        else:
            self.dist_left_arr = [9]
        self.dist_left = sum(self.dist_left_arr)/len(self.dist_left_arr)
        if (self.dist_left < dist_max and 
        self.dist_left > dist_min and
        self.dist_left_before - self.dist_left < max_change):
            rospy.loginfo("Left Distance:" + str(self.dist_left))
            # print("Left Distance:" + str(self.dist_left))
            if not self.busy:
                self.avoid_obstacle()

    def get_sonar_right(self, sensor_data_right):
        self.dist_right_before = self.dist_right
        if sensor_data_right.range > dist_min:
            self.dist_right_arr.append(sensor_data_right.range)
            if len(self.dist_right_arr)>middle_length:
                self.dist_right_arr.pop(0)
        else:
            self.dist_right_arr = [9]
        self.dist_right = sum(self.dist_right_arr)/len(self.dist_right_arr)
        if (self.dist_right < dist_max and 
        self.dist_right > dist_min and
        self.dist_right_before - self.dist_right < max_change):
            rospy.loginfo("Right Distance:" + str(self.dist_right))
            # print("Right Distance:" + str(self.dist_right))
            if not self.busy:
                self.avoid_obstacle()


    def get_min_scan(self, data):
        self.scan_min = min([x for x in data.ranges if x > 0.001])
        print(self.dist_left)
        print(self.dist_right)


    def avoid_obstacle(self):
        if self.dist_left < dist_critical:  # Hindernis links erkannt
            # self.busy = True
            if self.dist_left < self.dist_right and self.scan_min > 0.25:
                self.dist_left_arr = [9]
                self.dist_right_arr = [9]
                rospy.loginfo("detects obstacle in "
                              + str(self.dist_left)
                              + " m distance Left")
                
                header = std_msgs.msg.Header()  # Leere Instanz
                header.stamp = rospy.Time.now()  # Fülle Zeitstempel
                header.frame_id = 'base_link'
                self.point_cloud.header = header
                for i in range(10):
                    point = Point32()
                    point.x = self.dist_left * 0.5 + 0.05  # Offset Sensor Montagepunkt
                    point.y =((i-5)*0.03) + 0.06-(self.dist_left * 0.866) # Offset Sensor Montagepunkt
                    point.z = 0.06
                    self.point_cloud.points.append(point)
                self.cloud_pub.publish(self.point_cloud)
                # zuruecksetzen und rechts drehen
                self.twist.linear.x = -0.5
                # minus ist rechtsherun, getestet mit rqt
                self.twist.angular.z = -2
                timer = rospy.Time.now()
                self.busy = True 
                while rospy.Time.now() < timer + turn:
                    self.cmd_pub.publish(self.twist)
                    rospy.Rate(10).sleep()
                    print('turn')
                self.scan = True
                
                self.twist.linear.x = 1
                self.twist.angular.z = 0
                timer = rospy.Time.now() 
                while rospy.Time.now() < timer + forward:
                    self.cmd_pub.publish(self.twist)
                    rospy.Rate(10).sleep()
                self.busy = False 
                return

        if self.dist_right < dist_critical:  # Hindernis rechts erkannt
            if self.dist_right < self.dist_left and self.scan_min > 0.25:
                self.dist_left_arr = [9]
                self.dist_right_arr = [9]
                rospy.loginfo("detects obstacle in "
                              + str(self.dist_right)
                              + " m distance right")
                
                header = std_msgs.msg.Header()  # Leere Instanz
                header.stamp = rospy.Time.now()  # Fülle Zeitstempel
                header.frame_id = 'base_link'
                self.point_cloud.header = header
                for i in range(10):
                    point = Point32()
                    point.x = self.dist_right * 0.5 + 0.05  # Offset Sensor Montagepunkt
                    point.y =((i-5)*0.03) + 0.06-(self.dist_right * 0.866) # Offset Sensor Montagepunkt
                    point.z = 0.06
                    self.point_cloud.points.append(point)
                self.cloud_pub.publish(self.point_cloud)
                # zuruecksetzen und links drehen
                self.twist.linear.x = -0.5
                self.twist.angular.z = 2
                timer = rospy.Time.now()
                self.busy = True 
                while rospy.Time.now() < timer + turn:
                    self.cmd_pub.publish(self.twist)
                    rospy.Rate(10).sleep()
                    print('turn')
                self.scan = True

                self.twist.linear.x = 1
                self.twist.angular.z = 0
                timer = rospy.Time.now() 
                while rospy.Time.now() < timer + forward:
                    self.cmd_pub.publish(self.twist)
                    rospy.Rate(10).sleep()
                self.busy = False 
        return


if __name__ == '__main__':
    rospy.init_node('VL53_controller', anonymous=True)
    try:
        #dist_max = rospy.get_param("~max_dist_vl53")
        pass
    except rospy.ROSException:
        pass
    try:
        vl53 = VL53_to_Point_Cloud()
    except rospy.ROSInterruptException:
        pass
