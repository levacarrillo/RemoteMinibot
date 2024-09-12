#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16

class RobotStatus:
    def __init__(self):
        rospy.init_node('robot_status_node', anonymous=True)

        self.sub = rospy.Subscriber('/battery_data', Int16, self.callback)
        self.pubBatt = rospy.Publisher('/battery_percent', String, queue_size=10)
        self.pubTemp = rospy.Publisher('/cpu_temp', String, queue_size=10)
        self.rate = rospy.Rate(10)

    def callback(self, msg):
        min_perc = 20#%
        max_perc = 99#%
        min_analog = 418
        max_analog = 528

        batt_percent = min_perc + (msg.data - min_analog) * (max_perc - min_perc) / (max_analog - min_analog)
        batt_percent_str = str(round(-batt_percent, 1))+'%'
        self.pubBatt.publish(batt_percent_str)

    def get_cpu_temp(self):
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as file:
                temp_str = file.read().strip()
                temp_c = int(temp_str) / 1000.0
            return round(temp_c, 1)
        except FileNotFoundError:
            return None 

    def run(self):
        while not rospy.is_shutdown():
            temperature = str(self.get_cpu_temp())
            self.pubTemp.publish(temperature)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        print('Running robot_status_node.py by Luis Nava...')
        node = RobotStatus()
        node.run()
    except rospy.ROSInterruptException as error:
        print('There was an error: ', error)
        pass
        