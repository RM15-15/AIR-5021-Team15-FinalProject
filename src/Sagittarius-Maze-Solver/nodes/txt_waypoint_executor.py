#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
按 TXT 坐标顺序搬运物体：
1. 在 pick_pose 处抓取（夹爪闭合）
2. 依次经过每一行坐标，最后一个点松开夹爪
3. 返回 pick_pose，保持夹爪打开
"""

import math, rospy, actionlib, tf.transformations as tfm
import geometry_msgs.msg as gm
from sgr_msgs.msg import SGRCtrlAction, SGRCtrlGoal

def euler_to_quat(r, p, y):
    q = tfm.quaternion_from_euler(r, p, y)
    return gm.Quaternion(*q)

class TxtWaypointExecutor(object):
    def __init__(self):
        self.txt_path  = rospy.get_param("~txt_path")
        pick_pose_str  = rospy.get_param("~pick_pose")
        self.pick_xyz  = [float(v) for v in pick_pose_str.split()]
        ns             = rospy.get_param("~arm_ns")

        self.cli = actionlib.SimpleActionClient(
            "{}/sgr_ctrl".format(ns), SGRCtrlAction)
        rospy.loginfo("等待 sgr_ctrl 服务器上线...")
        self.cli.wait_for_server()
        rospy.loginfo("已连接到 sgr_ctrl 服务器")

    # ============== 主流程 ==============
    def run(self):
        # 1) 在固定点抓取
        self.send_goal(*self.pick_xyz, grasp=True)

        # 2) 读取所有路径点
        with open(self.txt_path) as f:
            lines = [ln for ln in f if ln.strip()]   # 去掉空行
        total = len(lines)

        for idx, line in enumerate(lines, 1):
            x, y = [float(v) for v in line.split()[:2]]
            z    = self.pick_xyz[2]

            # 若是最后一个点，松开夹爪
            grasp_flag = False if idx == total else True
            self.send_goal(x, y, z, grasp=grasp_flag)

            rospy.loginfo("到达第 %d/%d 点 (%.3f, %.3f, %.3f)，grasp=%s",
                          idx, total, x, y, z, grasp_flag)

        # 3) 回到起始点（夹爪已打开，不再变化）
        self.send_goal(*self.pick_xyz, grasp=False)
        rospy.loginfo("任务完成，已回到起始点")

    # ============== 统一封装动作发送 ==============
    def send_goal(self, x, y, z, grasp):
        goal = SGRCtrlGoal()
        goal.goal_pose.position.x = x
        goal.goal_pose.position.y = y
        goal.goal_pose.position.z = z
        goal.goal_pose.orientation = euler_to_quat(math.pi, 0, 0)
        goal.is_grasp = grasp        # True=夹取，False=松开

        self.cli.send_goal(goal)
        self.cli.wait_for_result()

# ---------------- 入口 ----------------
if __name__ == "__main__":
    rospy.init_node("txt_waypoint_executor")
    try:
        TxtWaypointExecutor().run()
    except rospy.ROSInterruptException:
        pass
