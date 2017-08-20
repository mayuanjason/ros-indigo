#!/usr/bin/env python

import rospy
import cv2
import cv2.cv as cv


class Test(object):

    def __init__(self, node_name):
        self.node_name = node_name

        rospy.init_node(self.node_name)
        rospy.loginfo("Starting node " + str(node_name))

        rospy.on_shutdown(self.cleanup)

        self.selected_point = None
        self.selection = None
        self.drag_start = None
        self.track_box = None
        self.display_box = None

        # Create the main display window
        self.cv_window_name = self.node_name
        cv2.namedWindow(self.cv_window_name, cv2.WINDOW_NORMAL)

        # Set a callback on mouse clicks on the image window
        cv.SetMouseCallback(self.cv_window_name, self.on_mouse_click, None)

        # Update the image display
        self.keystroke = cv2.waitKey(0)

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()

    def on_mouse_click(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN and not self.drag_start:
            self.track_box = None
            self.detect_box = None
            self.selected_point = (x, y)
            self.drag_start = (x, y)

        if event == cv.CV_EVENT_LBUTTONUP:
            self.drag_start = None

        if self.drag_start:
            rospy.loginfo("drag_start")
            xmin = max(0, min(x, self.drag_start[0]))
            ymin = max(0, min(y, self.drag_start[1]))


if __name__ == '__main__':
    try:
        node_name = "test"
        Test(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down test node."
        cv.DestroyAllWindows()
