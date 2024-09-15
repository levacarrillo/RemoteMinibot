#!/usr/bin/env python3

import rospy
import subprocess
from std_msgs.msg import Int16
from std_msgs.msg import String

class RobotStatus:
    def __init__(self):
        rospy.init_node('robot_status_node', anonymous=True)

        self.sub = rospy.Subscriber('/battery_data', Int16, self.callback)
        self.pubBatt = rospy.Publisher('/battery_percent', String, queue_size=10)
        self.pubTemp = rospy.Publisher('/cpu_temp', String, queue_size=10)
        self.pubVStatus = rospy.Publisher('/voltage_status', String, queue_size=10)
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
        except FileNotFoundError as error:
            print(f"There was an error getting processor's temperature: {error}")
            return None 

    def get_throttled_status(self):
        try:
            output = subprocess.check_output(["vcgencmd", "get_throttled"]).decode("utf-8")
            throttled_value = output.split("=")[1].strip()
            throttled_int = int(throttled_value, 16)
            issues_found = []
            status_scanned = {
                "Under voltage now": bool(throttled_int & 0x00001),
                "Frequency capped now": bool(throttled_int & 0x00002),
                "Throttling now": bool(throttled_int & 0x00004),
                "Under voltage occurred": bool(throttled_int & 0x10000),
                "Frequency capped occurred": bool(throttled_int & 0x20000),
                "Throttling occurred": bool(throttled_int & 0x40000)
            }
            for status in status_scanned:
                if status_scanned[status]:
                    issues_found.append(status)
            issues_found_str = ','.join(issues_found)
            return issues_found_str
        except subprocess.CalledProcessError as error:
            print(f"There was an error vcgencmd: {error}")
            return None

    def run(self):
        while not rospy.is_shutdown():
            temperature = str(self.get_cpu_temp())
            voltage_status = self.get_throttled_status()
            self.pubVStatus.publish(voltage_status)
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
        