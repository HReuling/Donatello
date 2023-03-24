#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range, LaserScan

dist_max = 0.3
# minimal distance to obstacle
dist_min = 0.2
# when laser scan is under this value no obstacles are avoided
min_scan_dist = 0.3

# number of distances that are middled
middle_length = 3

# k-factors for controller
k_angular = 1
k_linear = 0.5


class VL53_Regler():
    def __init__(self):
        self.twist = Twist()

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
        self.scan_min = 0


        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # overwrite nav_cmd to avoid obstacle
            if self.twist.angular.z != 0 or self.twist.linear.x !=0:
                cmd_pub.publish(self.twist)
                print("Avoid obstacle")
                self.twist.angular.z = 0
                self.twist.linear.x = 0
            self.rate.sleep()

    def get_sonar_left(self, sensor_data_left):
        self.dist_left_arr.append(sensor_data_left.range)
        if len(self.dist_left_arr)>middle_length:
            self.dist_left_arr.pop(0)

        self.dist_left = sum(self.dist_left_arr)/len(self.dist_left_arr)

        # only compare with max, when distance is smaller robot drives backwards
        if (self.dist_left < dist_max):
            self.avoid_obstacle()

    def get_sonar_right(self, sensor_data_right):
        self.dist_right_arr.append(sensor_data_right.range)
        if len(self.dist_right_arr)>middle_length:
            self.dist_right_arr.pop(0)

        self.dist_right = sum(self.dist_right_arr)/len(self.dist_right_arr)

        # only compare with max, when distance is smaller robot drives backwards
        if (self.dist_right < dist_max):
            self.avoid_obstacle()


    def get_min_scan(self, data):
        self.scan_min = min([x for x in data.ranges if x > 0.001])
        #print(self.dist_left)
        #print(self.dist_right)


    def controller(self, dist):
        # returns linear and angular vecolity depending on distance
        cmd_angular = (dist_max-dist)/(dist_max-dist_min) * k_angular 
        cmd_linear = (1-(dist_max-dist)/(dist_max-dist_min)) * k_linear
        return cmd_angular, cmd_linear


    def avoid_obstacle(self):
        if self.dist_left < self.dist_right and self.scan_min > min_scan_dist:
            cmds = self.controller(self.dist_left)
            
            # in this case the robot turns right -> angular velocity negative
            self.twist.angular.z = - cmds[0] 
            self.twist.linear.x = cmds[1]
            return

        if self.dist_left > self.dist_right and self.scan_min > min_scan_dist:
            cmds = self.controller(self.dist_right)

            # in this case the robot turns left -> angular velocity positive
            self.twist.angular.z = cmds[0] 
            self.twist.linear.x = cmds[1]
        return


if __name__ == '__main__':
    # initialize node 
    rospy.init_node('VL53Regler', anonymous=True)

    # initialize publisher maybe now it will also work when it is declared in the class
    cmd_pub = rospy.Publisher('obst_vel', Twist, queue_size=10)
    try:
        vl53 = VL53_Regler()
    except rospy.ROSInterruptException:
        pass
