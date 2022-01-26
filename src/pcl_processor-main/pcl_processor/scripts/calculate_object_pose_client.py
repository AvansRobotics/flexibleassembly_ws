#!/usr/bin/env python

import sys
import rospy
from pcl_processor_msgs.srv import CalculateObjectposeFromPointcloud, CalculateObjectposeFromPointcloudRequest, CalculateObjectposeFromPointcloudResponse

def calculate_object_pose_client():
    # First wait for the service to become available.
    rospy.loginfo("Waiting for service...")
    rospy.wait_for_service("/pcl_processor/calculate_object_pose")
    try:
        # Create a service proxy.
        calculate_object_pose = rospy.ServiceProxy('/pcl_processor/calculate_object_pose', CalculateObjectposeFromPointcloud)

        request = CalculateObjectposeFromPointcloudRequest()
        # Call the service here.
        rospy.loginfo("Calculating...")
        service_response = calculate_object_pose(request)
        rospy.loginfo("Done calculating")

        # Return the response to the calling function.
        return service_response

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":

    # Initialize the client ROS node.
    rospy.init_node("calculate_object_pose_client", anonymous = False)

    # Call the service client function.
    service_response = calculate_object_pose_client()

    # Process the service response and display log messages accordingly.
    if(not service_response.success):
        rospy.logerr("Calculate objectpose unsuccessful!")
    else:
       rospy.loginfo("Calculate objectpose successful!")
       rospy.loginfo(service_response.object_pose)
#       print(service_response.camera_pose)
