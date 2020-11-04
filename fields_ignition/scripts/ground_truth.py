#!/usr/bin/env python3

import rospy
from rospy import loginfo
from pathlib import Path
import numpy as np
import json
import struct
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header
from fields_ignition_msgs.msg import Detections

rospy.init_node('ground_truth', anonymous=True)

use_rviz = rospy.get_param('~use_rviz', True)
world_dir = rospy.get_param('~world_dir')

loginfo('use_rviz={}'.format(use_rviz))
loginfo('world_dir={}'.format(world_dir))


class TracePoints():
    def __init__(self, topic="debug_points", frame_id="field"):
        self.publisher = rospy.Publisher(topic, PointCloud2, queue_size=1)
        self.frame_id = frame_id

        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1),
                       # PointField('rgb', 12, PointField.UINT32, 1),
                       PointField('rgba', 12, PointField.UINT32, 1),
                       ]
        self.reset()

    def reset(self):
        self.points = []

    def add_point(self, point, color):
        x = point[0]
        y = point[1]
        z = point[2]
        r = int(color[0] * 255.0)
        g = int(color[1] * 255.0)
        b = int(color[2] * 255.0)
        a = 255
        rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
        pt = [x, y, z, rgb]
        self.points.append(pt)

    def publish(self):
        if not use_rviz or not self.fields:
            return
        header = Header()
        header.frame_id = self.frame_id
        msg = pc2.create_cloud(header, self.fields, self.points)
        self.publisher.publish(msg)

tp_fruits = TracePoints('/fields/fruits')
tp_plants = TracePoints('/fields/plants')
tp_detections = TracePoints('/fields/detections')
id_matched_fruit_markers = set()
fruits = {
    'positions': [],
    'ids': []
}

with open(Path(world_dir) / 'markers.json', 'r') as markers:
    markers = markers.read()
    rospy.set_param('~markers', markers)
    markers = json.loads(markers)

def update_marker_traces():
    tp_fruits.reset()
    tp_plants.reset()

    fruits['positions'] = []
    fruits['ids'] = []
    for marker in markers:
        if marker['marker_type'] == 'FRUIT':
            color = [0, 1, 0] if marker['id'] in id_matched_fruit_markers else [1, .3, .3]
            tp_fruits.add_point(marker['translation'], color)
            fruits['positions'].append(marker['translation'])
            fruits['ids'].append(marker['id'])
        elif marker['marker_type'] == 'PLANT':
            tp_plants.add_point(marker['translation'], [.3, .1, .3])
    
    fruits['positions'] = np.array(fruits['positions'])


update_marker_traces()


def handle_detection(data: Detections):
    tp_detections.reset()
    for d in data.detections:
        x = d.position.pose.position.x
        y = d.position.pose.position.y
        z = d.position.pose.position.z
        tp_detections.add_point([x, y, z], [0, 0, 1])

        pos = np.array([x, y, z])
        dists = np.sqrt(np.sum((fruits['positions'] - pos) ** 2, axis=1))
        match = np.argmax(dists < 0.12)
        if match >= 0:
            id_matched_fruit_markers.add(fruits['ids'][match])
                    
    update_marker_traces()


# rospy.Subscriber("/detections", Detections, handle_detection, queue_size=1)

try:
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            handle_detection(rospy.wait_for_message('/detections', Detections, 1))
        except rospy.exceptions.ROSException:
            pass
        tp_fruits.publish()
        tp_plants.publish()
        tp_detections.publish()
        rate.sleep()
except rospy.ROSInterruptException:
    pass
