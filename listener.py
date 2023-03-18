#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy
#from std_msgs.msg import String
from sensor_msgs.msg import Joy

class subscr:
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        self.controls = ((0,0,0,0,0,0,0,0), (0,0,0,0,0,0,0,0,0,0,0,0,0,0,0))
        rospy.Subscriber('/joy', Joy, self.callback)
        
    def callback(self, data):
        self.controls = (data.axes, data.buttons)
        #print(self.controls)

    def get_data(self):
        return self.controls


#def callback(data):
#    #print("hello\n")
#    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
#    #print(data.axes, data.buttons)
#    if data.axes == None or data.buttons == None:
#        controls = ((0,0,0,0,0,0,0,0), (0,0,0,0,0))
#    else:
#        controls = (data.axes, data.buttons)

#def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
#    rospy.init_node('listener', anonymous=True)

#    controls = ((0,0,0,0,0,0,0,0), (0,0,0,0,0))

#    ret = rospy.Subscriber("/joy", Joy, callback)

#    return callback("/joy")

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

#if __name__ == '__main__':
#    listener()
