#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import math
import time
import csv
import rospkg


class SubThenFilter:
    def __init__(self):
        self.loadCsv()

        self.sub = rospy.Subscriber("loomo/goal_string", String, self.callback, queue_size=1)
        self.pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

    def loadCsv(self):
        self.dict_label2id = {}
        self.dict_id2pos = {}

        rospack = rospkg.RosPack()
        path = rospack.get_path('automap')

        # Read id2label
        with open(path+"/datastore/id2label.csv", 'rb') as csvfile:
            file_id2label = csv.reader(csvfile, delimiter=',', quotechar='|')

            for i, row in enumerate(file_id2label):
                if i == 0:
                    continue

                id = int(row[0])
                label = row[1]

                self.dict_label2id[label] = id

        #  Read pos2id
        with open(path+"/datastore/id2pos.csv", 'rb') as csvfile:
            file_pos2id = csv.reader(csvfile, delimiter=',', quotechar='|')

            for i, row in enumerate(file_pos2id):
                if i == 0:
                    continue

                id, x, y, z, w = map(float, row)
                self.dict_id2pos[int(id)] = [x, y, z, w]


    def pushToRtabmap(self, marker, dist):
        msg = UserData()
        msg.header.frame_id = marker.header.frame_id
        msg.header.stamp = marker.header.stamp
        msg.data = [marker.id, dist, dist * 100 % 100]  # id, meter, centimeter
        msg.rows = 1
        msg.cols = 3
        msg.type = cv2.CV_32F
        msg.data = np.ones((1, 3), dtype=np.dtype('Float32'))

        self.pub.publish(msg)

    def callback(self, data):
        label = data.data

        try:
            pos = self.dict_id2pos[self.dict_label2id[label]]
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.pose.position.x = pos[0]
            msg.pose.position.y = pos[1]
            msg.pose.orientation.z = pos[2]
            msg.pose.orientation.w = pos[3]

            self.pub.publish(msg)
        except:
            print("Does not found required pos for label ", label)


if __name__ == "__main__":
    rospy.init_node("label2goal", anonymous=True)

    sf = SubThenFilter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
