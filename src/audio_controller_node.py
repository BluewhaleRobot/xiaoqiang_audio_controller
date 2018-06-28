#!/usr/bin/env python
# encoding=utf-8
# The MIT License (MIT)
#
# Copyright (c) 2018 Bluewhale Robot
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Author: Randoms
#

import rospy
from std_msgs.msg import String
import time


TALK_PUB = None
CURRENT_COMMAND = ""
PROCESSING_FLAG = False
GALILEO_STATUS = None

def update_command(cmd):
    global CURRENT_COMMAND
    CURRENT_COMMAND = cmd.data

def update_galileo_status(status):
    pass


if __name__ == "__main__":
    global TALK_PUB, CURRENT_COMMAND, PROCESSING_FLAG
    rospy.init_node("xiaoqiang_audio_controller", anonymous=True)
    TALK_PUB = rospy.Publisher("~talk", String, queue_size=10)

    rospy.Subscriber("~listen", String, update_command)

    while not rospy.is_shutdown():
        time.sleep(0.1)
        if PROCESSING_FLAG:
            continue
        if CURRENT_COMMAND == "":
            continue
        PROCESSING_FLAG = True
        if "开启导航" in CURRENT_COMMAND:
            response = String()
            response.data = "好的，正在开启导航"
            TALK_PUB.publish(response)
        if "关闭导航" in CURRENT_COMMAND:
            response = String()
            response.data = "好的, 正在关闭导航"
        
        
        PROCESSING_FLAG = False


