#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import numpy as np 

def build_markers(position_x, position_y, position_z):
    marker = Marker()

    marker.header.frame_id = "base"
    marker.header.stamp = rospy.Time.now()
    scale_factor = 0.1

    # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
    marker.type = 2
    marker.id = 0

    # Set the scale of the marker
    marker.scale.x = scale_factor
    marker.scale.y = scale_factor
    marker.scale.z = scale_factor

    # Set the color
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Set the pose of the marker
    marker.pose.position.x = position_x
    marker.pose.position.y = position_y
    marker.pose.position.z = position_z
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    return marker


rospy.init_node('rviz_marker')

marker_pub = rospy.Publisher("/visualization_marker_Array", MarkerArray, queue_size = 2)


# markerArray = MarkerArray()

MARKERS_MAX = 100
count = 0


base_dir = rospy.get_param('~data_position')
res = np.loadtxt(base_dir)

while not rospy.is_shutdown():
  markerArray = MarkerArray()
  
#   marker = build_markers(0,0,0.5)
#   markerArray.markers.append(marker)

  for i in range(res.shape[0]//48):
      marker = build_markers(res[48*i+6],res[48*i+7],res[48*i+8])
      markerArray.markers.append(marker)
        
#   if (count > MARKERS_MAX):
#       markerArray.markers.pop(0)
  count += 1
  id = 0
  for m in markerArray.markers:
    m.id = id
    id += 1    
    
    
  marker_pub.publish(markerArray)
  # rospy.rostime.wallsleep(1.0)