#!/usr/bin/env python

# Software License Agreement (BSD 3-Clause License)
#
# Copyright (c) 2018, Kai Krückel (krukai@veemo.ink)
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import InteractiveMarkerInit
from visualization_msgs.msg import InteractiveMarkerUpdate


class InteractiveMarkerTransformBroadcaster:
    def __init__(self):
        # Retrieve parameters from rosparam
        self.broadcast_erased = rospy.get_param('~broadcast_erased', False)
        rospy.loginfo('Broadcast erased markers: {0}'.format(self.broadcast_erased))
        self.prefix = rospy.get_param('~prefix', '')
        if self.prefix != '':
            rospy.loginfo('Transform name prefix: {0}'.format(self.prefix))
        self.server_id = rospy.get_param('~server_id', None)
        if self.server_id:
            rospy.loginfo('Only accepted server_id: {0}'.format(self.server_id))
        self.server_seq_num = rospy.get_param('~seq_num', 0)
        if self.server_seq_num != 0:
            rospy.loginfo('Initial seq_num: {0}'.format(self.server_seq_num))
        self.suffix = rospy.get_param('~suffix', '')
        if self.suffix != '':
            rospy.loginfo('Transform name suffix: {0}'.format(self.suffix))

        # Create transform broadcaster
        self.br = tf2_ros.TransformBroadcaster()

        # Set up subscribers
        self.init_sub = rospy.Subscriber('int_server/update_full', InteractiveMarkerInit, self.on_init_msg)
        self.update_sub = None

        self.int_markers = {}

    def on_init_msg(self, data):
        if self.server_id and self.server_id != data.server_id:
            rospy.logwarn('Ignoring initial marker positions from {0}, expected {1}'.format(data.server_id,
                                                                                            self.server_id))
            return
        if self.server_seq_num is not None and data.seq_num < self.server_seq_num:
            rospy.logerr('seq_num from init message is lower than expected'.format(self.server_id, data.server_id))
        self.server_seq_num = data.seq_num
        rospy.loginfo('Received initial marker positions from {0}'.format(data.server_id))

        for int_marker in data.markers:
            self.int_markers[int_marker.name] = int_marker
            self.fix_invalid_quaternion(int_marker.name)

        self.broadcast_transforms()

        # Subscribe to update instead of init messages
        self.init_sub.unregister()
        self.update_sub = rospy.Subscriber('int_server/update', InteractiveMarkerUpdate, self.on_update_msg)

    def on_update_msg(self, data):
        # Ignore keep-alive messages
        if data.type == InteractiveMarkerUpdate.KEEP_ALIVE:
            return

        if self.server_id and self.server_id != data.server_id:
            rospy.logwarn('Ignoring marker positions update from {0}, expected {1}'.format(data.server_id,
                                                                                           self.server_id))
            return

        if data.seq_num == self.server_seq_num:
            rospy.loginfo('Received an update known through an init message ({0})'.format(data.seq_num))
            return
        elif data.seq_num < self.server_seq_num:
            rospy.loginfo(
                'Received an outdated update (got {0}, expected {1})'.format(data.seq_num, self.server_seq_num + 1))
            return
        elif data.seq_num > self.server_seq_num + 1:
            rospy.logerr(
                'Missed an update, waiting for new init message ({0}, expected {1})'.format(data.seq_num,
                                                                                            self.server_seq_num + 1))
            # Subscribe to init instead of update messages
            self.update_sub.unregister()
            self.init_sub = rospy.Subscriber('int_server/update_full', InteractiveMarkerInit, self.on_init_msg)
            return
        self.server_seq_num = data.seq_num

        # Add new interactive markers (this also includes updating properties other than pose)
        for int_marker in data.markers:
            self.int_markers[int_marker.name] = int_marker
            self.fix_invalid_quaternion(int_marker.name)

        # Update poses
        for pose_msg in data.poses:
            self.int_markers[pose_msg.name].header = pose_msg.header
            self.int_markers[pose_msg.name].pose = pose_msg.pose
            self.fix_invalid_quaternion(pose_msg.name)

        # Erase interactive markers
        if self.broadcast_erased:
            for name in data.erases:
                try:
                    del self.int_markers[name]
                except KeyError:
                    pass

        # Publish tf data
        self.broadcast_transforms()
        rospy.loginfo('Broadcast updated transforms')

    def fix_invalid_quaternion(self, name):
        # Correct zero quaternion
        if (self.int_markers[name].pose.orientation.x == 0.0
                and self.int_markers[name].pose.orientation.y == 0.0
                and self.int_markers[name].pose.orientation.z == 0.0
                and self.int_markers[name].pose.orientation.w == 0.0):
            self.int_markers[name].pose.orientation.w = 1.0

    def broadcast_transforms(self):
        t = TransformStamped()
        t.header.seq = self.server_seq_num
        for _, int_marker in self.int_markers.items():
            if int_marker.header.stamp.to_sec() or int_marker.header.stamp.to_nsec():
                t.header.stamp = int_marker.header.stamp
            else:
                t.header.stamp = rospy.Time.now()
            t.header.frame_id = int_marker.header.frame_id
            t.child_frame_id = self.prefix + int_marker.name + self.suffix
            t.transform.translation.x = int_marker.pose.position.x
            t.transform.translation.y = int_marker.pose.position.y
            t.transform.translation.z = int_marker.pose.position.z
            t.transform.rotation = int_marker.pose.orientation
            self.br.sendTransform(t)


if __name__ == '__main__':
    rospy.init_node('int_marker_to_tf')
    rate_hz = rospy.get_param('~rate', 0)
    int_tf = InteractiveMarkerTransformBroadcaster()

    if rate_hz > 0:
        rate = rospy.Rate(rate_hz)
        try:
            while not rospy.is_shutdown():
                int_tf.broadcast_transforms()
                rate.sleep()
        except rospy.ROSInterruptException:
            pass
    else:
        rospy.spin()
