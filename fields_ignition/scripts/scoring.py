#!/usr/bin/env python3

import rospy
from pathlib import Path

rospy.init_node('scoring')

use_rviz = rospy.get_param('~use_rviz', True)
world_dir = rospy.get_param('~world_dir')

with open(Path(world_dir) / 'markers.json', 'r') as markers:
    rospy.set_param('~markers', markers)
