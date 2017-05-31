#!/usr/bin/python
#-*- encoding: utf8 -*-

import json

import rospy
import tf
import tf2_ros
from mhri_msgs.srv import ReadData, ReadDataRequest
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray, Marker


class ObjectTFPublisherNode:
    def __init__(self):
        rospy.init_node('object_tf_publisher', anonymous=False)

        rospy.loginfo('wait for bringup environmental_memory...')
        rospy.wait_for_service('/environmental_memory/read_data')
        self.rd_srv = rospy.ServiceProxy('/environmental_memory/read_data', ReadData)

        self.pub_vis_msg = rospy.Publisher('visualization_objects', MarkerArray, queue_size=10)

        refresh_period = rospy.get_param('~refresh_period', 0.5)
        rospy.Timer(rospy.Duration(refresh_period), self.handle_periodic_publish)

        rospy.loginfo('initialized...')
        rospy.spin()

    def handle_periodic_publish(self, event):
        req = ReadDataRequest()
        req.perception_name = 'objects'
        req.query = '{}'
        req.data = ['~']

        result = self.rd_srv(req)
        if not result.result:
            return

        vis_msg = MarkerArray()

        ret_data = json.loads(result.data)

        for obj in ret_data:
            br = tf2_ros.TransformBroadcaster()
            transformed_msg = TransformStamped()

            transformed_msg.header.stamp = rospy.Time.now()
            transformed_msg.header.frame_id = obj['frame_id']
            transformed_msg.child_frame_id = obj['name']

            transformed_msg.transform.translation.x = obj['xyz'][0]
            transformed_msg.transform.translation.y = obj['xyz'][1]
            transformed_msg.transform.translation.z = obj['xyz'][2]

            q = tf.transformations.quaternion_from_euler(obj['rpy'][0], obj['rpy'][1], obj['rpy'][2])

            transformed_msg.transform.rotation.x = q[0]
            transformed_msg.transform.rotation.y = q[1]
            transformed_msg.transform.rotation.z = q[2]
            transformed_msg.transform.rotation.w = q[3]

            br.sendTransform(transformed_msg)


            marker = Marker()
            marker.header.frame_id = obj['frame_id']
            marker.header.stamp = rospy.Time.now()
            marker.ns = obj['name']
            marker.id = 0

            obj_type = obj['geometry'].keys()[0]
            if obj_type == 'box':
                marker.type = Marker.CUBE
            elif obj_type == 'cylinder':
                marker.type = Marker.CYLINDER
            elif obj_type == 'sphere':
                marker.type = Marker.SPHERE

            marker.action = Marker.ADD
            marker.pose.position.x = obj['xyz'][0]
            marker.pose.position.y = obj['xyz'][1]
            marker.pose.position.z = obj['xyz'][2]

            orientation = tf.transformations.quaternion_from_euler(obj['rpy'][0], obj['rpy'][1], obj['rpy'][2])

            marker.pose.orientation.x = orientation[0]
            marker.pose.orientation.y = orientation[1]
            marker.pose.orientation.z = orientation[2]
            marker.pose.orientation.w = orientation[3]

            marker.scale.x = obj['geometry'][obj_type][0]
            marker.scale.y = obj['geometry'][obj_type][1]
            marker.scale.z = obj['geometry'][obj_type][2]

            marker.color.r = obj['material']['color'][0]
            marker.color.g = obj['material']['color'][1]
            marker.color.b = obj['material']['color'][2]
            marker.color.a = obj['material']['color'][3]

            vis_msg.markers.append(marker)

        self.pub_vis_msg.publish(vis_msg)



if __name__ == '__main__':
    m = ObjectTFPublisherNode()
