#!/usr/bin/env python3
from __future__ import print_function
from shutil import move

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *


def processFeedback(feedback):
    p = feedback.pose.position
    print(feedback.marker_name + " is now at " +
          str(p.x) + ", " + str(p.y) + ", " + str(p.z))


def makeBox(msg):
    box_marker = Marker()

    box_marker.type = Marker.CYLINDER
    box_marker.scale.x = 1.0
    box_marker.scale.y = 1.0
    box_marker.scale.z = 0.01
    box_marker.color.r = 0.0
    box_marker.color.g = 1.0
    box_marker.color.b = 0.0
    box_marker.color.a = 0.6

    return box_marker


if __name__ == "__main__":
    rospy.init_node("simple_marker")

    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("simple_marker")

    # create an interactive marker for our server
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.name = "my_marker"
    int_marker.description = "Simple 1-DOF Control"

    # create a grey box marker
    # box_marker = Marker()
    # box_marker.type = Marker.CYLINDER
    # box_marker.scale.x = 1.0
    # box_marker.scale.y = 1.0
    # box_marker.scale.z = 0.01
    # box_marker.color.r = 0.0
    # box_marker.color.g = 0.5
    # box_marker.color.b = 0.5
    # box_marker.color.a = 1.0

    # create a non-interactive control which contains the box
    box_control = InteractiveMarkerControl()
    # box_control.always_visible = True
    # box_control.markers.append(box_marker)

    # add the control to the interactive marker
    int_marker.controls.append(box_control)

    # create a control which will move the box
    # this control does not contain any markers,
    # which will cause RViz to insert two arrows
    rotate_control = InteractiveMarkerControl()
    rotate_control.name = "move_x"
    rotate_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS

    # add the control to the interactive marker
    # int_marker.controls.append(rotate_control)

    # write
    move_y_control = InteractiveMarkerControl()
    move_y_control.name = "move_y"
    move_y_control.orientation.w = 1
    move_y_control.orientation.x = 0
    move_y_control.orientation.y = 1
    move_y_control.orientation.z = 0
    # move_y_control.interaction_mode = InteractiveMarkerControl.MOVE_3D
    move_y_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    move_y_control.orientation_mode = InteractiveMarkerControl.INHERIT
    # move_y_control.independent_marker_orientation = True
    # int_marker.controls.append(copy.deepcopy(move_y_control))

    move_y_control.markers.append(makeBox(int_marker))
    # move_y_control.always_visible =
    move_y_control.always_visible = False
    int_marker.controls.append(move_y_control)

    # add the interactive marker to our collection &
    # tell the server to call processFeedback() when feedback arrives for it
    server.insert(int_marker, processFeedback)

    # 'commit' changes and send to all clients
    server.applyChanges()

    rospy.spin()
