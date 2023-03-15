#!/usr/bin/env python3

import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import PointCloud

delayVL = 15
delayClear = 2

class clearCostmap():

    def __init__(self) -> None:

        rospy.wait_for_service('/move_base/clear_costmaps')
        print("Service gestartet")
        self.proxy = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        
        self.timerVL = rospy.get_time()
        self.timerClear = self.timerVL

        self.pointCloud = rospy.Subscriber('VL53/point_cloud',
                                            PointCloud,
                                            self.clearMap,
                                            queue_size=10)
        
        #TODO: Nur clearen wenn Objekte nah sind
        
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.rate.sleep()
            if (rospy.get_time() - self.timerVL > delayVL and rospy.get_time() - self.timerClear > delayClear):
                self.timerClear = rospy.get_time()
                self.proxy()
            elif (rospy.get_time() - self.timerClear > 10):
                self.timerClear = rospy.get_time()
                self.proxy()
        
    def clearMap(self, data):
        if(data.points):
            self.timerVL = rospy.get_time()
            
    

if __name__ == '__main__':

    rospy.init_node('CoastmapClearer', anonymous=True)

    # try:
    #     delayVL = rospy.get_param("~delayVL")
    #     delayClear = rospy.get_param("~delayClear")
    # except rospy.ROSException:
    #     pass
    
    clearcostmap = clearCostmap()

