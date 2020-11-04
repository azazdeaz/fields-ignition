#!/usr/bin/env python3

import rospy
from rospy import loginfo
from pathlib import Path
import json
import struct
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2 as pc2
from std_msgs.msg import Header

rospy.init_node('scoring')

use_rviz = rospy.get_param('~use_rviz', True)
world_dir = rospy.get_param('~world_dir')

loginfo('use_rviz={}'.format(use_rviz))
loginfo('world_dir={}'.format(world_dir))


class TracePoints():
    def __init__(self, topic="debug_points", frame_id="map"):
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
        if not use_rviz:
            return
        header = Header()
        header.frame_id = self.frame_id
        msg = pc2.create_cloud(header, self.fields, self.points)
        self.publisher.publish(msg)

tp_fruits = TracePoints('/fields/fruits')
tp_plants = TracePoints('/fields/plants')

with open(Path(world_dir) / 'markers.json', 'r') as markers:
    markers = markers.read()
    rospy.set_param('~markers', markers)
    markers = json.loads(markers)

    for marker in markers:
        if marker['marker_type'] == 'FRUIT':
            tp_fruits.add_point(marker['translation'], [1, .3, .3])
        elif marker['marker_type'] == 'PLANT':
            tp_plants.add_point(marker['translation'], [.3, .1, .3])
    
try:
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        tp_fruits.publish()
        tp_plants.publish()
        rate.sleep()
except rospy.ROSInterruptException:
    pass
