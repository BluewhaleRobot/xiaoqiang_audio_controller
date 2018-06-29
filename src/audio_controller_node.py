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
from galileo_serial_server.msg import GalileoNativeCmds, GalileoStatus
import re
from utils import hans2num


TALK_PUB = None
CURRENT_COMMAND = ""
PROCESSING_FLAG = False
GALILEO_STATUS = GalileoStatus()


def update_command(cmd):
    global CURRENT_COMMAND
    CURRENT_COMMAND = cmd.data


def update_galileo_status(status):
    global GALILEO_STATUS
    GALILEO_STATUS = status


def check_cmd(cmd, patterns):
    for partern in patterns:
        if re.match(partern, cmd) is not None or cmd.find(partern) != -1:
            return True
    return False


if __name__ == "__main__":
    # global TALK_PUB, CURRENT_COMMAND, PROCESSING_FLAG
    rospy.init_node("xiaoqiang_audio_controller", anonymous=True)
    TALK_PUB = rospy.Publisher("~talk", String, queue_size=10)
    chat_pub = rospy.Publisher("~chat", String, queue_size=10)
    cmd_pub = rospy.Publisher("~cmd", GalileoNativeCmds, queue_size=10)

    rospy.Subscriber("~listen", String, update_command)
    rospy.Subscriber("~status", GalileoStatus, update_galileo_status)

    while not rospy.is_shutdown():
        time.sleep(0.1)
        if PROCESSING_FLAG:
            continue
        if CURRENT_COMMAND == "":
            continue
        PROCESSING_FLAG = True
        # 汉字数字转成阿拉伯数字
        CURRENT_COMMAND = hans2num(CURRENT_COMMAND)

        if check_cmd(CURRENT_COMMAND, ["开启导航", "开始导航", "开启视觉导航", "开始视觉导航", "开启视觉"]):
            response = String()
            if GALILEO_STATUS.navStatus == 1:
                # 已经开启导航功能
                response.data = "已经开启了导航功能，你可以开始对我说要去的目标点了"
            elif GALILEO_STATUS.navStatus == 0 and GALILEO_STATUS.visualStatus != 0:
                # 处于建图状态
                response.data = "我现在正在建立地图，无法进行导航"
            else:
                response.data = "好的，正在开启导航"
                cmd = GalileoNativeCmds()
                cmd.data = [ord("m"), 0]
                cmd_pub.publish(cmd)
            TALK_PUB.publish(response)

        elif check_cmd(CURRENT_COMMAND, ["关闭导航", "关闭视觉导航", "停止"]):
            response = String()
            if GALILEO_STATUS.navStatus != 1 or GALILEO_STATUS.visualStatus == 0:
                response.data = "现在没有开启视觉"
            else:
                response.data = "好的, 正在关闭导航"
                cmd = GalileoNativeCmds()
                cmd.data = [ord("m"), 4]
                cmd_pub.publish(cmd)
            TALK_PUB.publish(response)

        elif check_cmd(CURRENT_COMMAND, ["继续"]):
            response = String()
            if GALILEO_STATUS.targetNumID == -1 or GALILEO_STATUS.targetStatus != 2:
                response.data = "当前目标点并没有被暂停"
            else:
                response.data = "正在继续前往当前导航目标"
                cmd = GalileoNativeCmds()
                cmd.data = [ord("i"), 1]
                cmd_pub.publish(cmd)
            TALK_PUB.publish(response)

        elif check_cmd(CURRENT_COMMAND, ["暂停"]):
            response = String()
            if GALILEO_STATUS.targetNumID == -1 or GALILEO_STATUS.targetStatus != 1:
                response.data = "现在并没有前往目标点"
            else:
                cmd = GalileoNativeCmds()
                cmd.data = [ord("i"), 0]
                response.data = "已经暂停当前导航目标"
                cmd_pub.publish(cmd)
            TALK_PUB.publish(response)

        elif check_cmd(CURRENT_COMMAND, ["取消"]):
            response = String()
            if GALILEO_STATUS.targetNumID == -1:
                response.data = "当前并没有需要取消的任务"
            else:
                cmd = GalileoNativeCmds()
                cmd.data = [ord("i"), 2]
                cmd_pub.publish(cmd)
                response.data = "已经取消当前导航目标"
            TALK_PUB.publish(response)

        elif check_cmd(CURRENT_COMMAND, ["(\S+)?到(?P<num>\d+)号(目标)?(点)?"]):
            response = String()
            if GALILEO_STATUS.navStatus != 1:
                response.data = "现在还没有开始导航，请先说开始导航"
            elif GALILEO_STATUS.targetStatus != 0:
                response.data = "当前导航任务还没有完成，请先取消任务"
            else:
                num_res = re.match(
                    "(\S+)?到(?P<num>\d+)号(目标)?(点)?", CURRENT_COMMAND)
                rospy.loginfo("CMD: Goal " + str(num_res.group("num")))
                cmd = GalileoNativeCmds()
                cmd.data = [ord("g"), int(num_res.group("num"))]
                cmd_pub.publish(cmd)
                response.data = "好的，正在前往{num}号点".format(num=num_res.group("num"))
            TALK_PUB.publish(response)

        elif check_cmd(CURRENT_COMMAND, ["电压", "电池", "电量"]):
            response = String()
            response.data = "当前的电压是{power}伏".format(power=round(GALILEO_STATUS.power, 2))
            TALK_PUB.publish(response)

        elif check_cmd(CURRENT_COMMAND, ["关机"]):
            response = String()
            response.data = "好的，再见"
            cmd = GalileoNativeCmds()
            cmd.data = [0xaa, 0x44]
            TALK_PUB.publish(response)
            time.sleep(3)
            cmd_pub.publish(cmd)

        else:
            response = String()
            response.data = CURRENT_COMMAND
            chat_pub.publish(response)
        CURRENT_COMMAND = ""
        PROCESSING_FLAG = False
