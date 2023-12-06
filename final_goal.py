#!/usr/bin/env python3
import sys
import argparse
import rospy
import math
import actionlib
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def check_goal_reached(cur_x, cur_y, goal_x, goal_y, bias):
    if(cur_x > goal_x - bias and cur_x < goal_x + bias\
        and cur_y > goal_y - bias and cur_y < goal_y + bias):
        return True
    else:
        return False


# 控制机器人移动到下一个目标点
def move_robot(init_position, target_position, off_set_x, off_set_y):
    for target_pos in target_position:
        init_pose = rospy.wait_for_message('/id701/aruco_single/pose', PoseStamped)
        cur_x = init_pose.pose.position.x
        cur_y = init_pose.pose.position.y
        target_x = (target_pos[0] - 5) * off_set_x
        target_y = (target_pos[1] - 5) * off_set_y
        while not check_goal_reached(cur_x, cur_y, target_x, target_y, 0.05):
            init_pose = rospy.wait_for_message('/id701/aruco_single/pose', PoseStamped)

            orientation_q = init_pose.pose.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            Orientation = yaw

            dx = target_x - cur_x
            dy = target_y - cur_y
            distance = math.dist([cur_x, cur_y],[target_x, target_y])
            goal_direct = math.atan2(dy,dx)

            print("init_pose", [cur_x, cur_y])
            print("goal_pose", [target_x, target_y])
            print("Orientation", Orientation)
            print("goal_direct", goal_direct)

            if(Orientation < 0):
                Orientation = Orientation + 2 * math.pi
            if(goal_direct < 0):
                goal_direct = goal_direct + 2 * math.pi

            theta = goal_direct - Orientation

            if theta < 0 and abs(theta) > abs(theta + 2 * math.pi):
                theta = theta + 2 * math.pi
            elif theta > 0 and abs(theta - 2 * math.pi) < theta:
                theta = theta - 2 * math.pi
    
            print("theta:", theta)
            
            k2 = 2
            linear = 0.5
            angular = k2 * theta
            twist.linear.x = linear * distance * math.cos(theta)
            twist.angular.z = -angular
            cmd_pub.publish(twist)     

def main(agent_number):
    rospy.init_node('robot_mover')
    global cmd_pub
    cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    init_pose = rospy.wait_for_message('/id701/aruco_single/pose', PoseStamped)

    if agent_number == ‘1’:
        init_position = [9,9]
        target_position = [[8,9],[7,9],[7,8],[7,7],[8,7],[8,6],[8,5],[7,5],[7,4],[7,3],[6,3],[5,3],[4,3],[3,3],[3,2],[3,1],[4,1],[5,1],[6,1],[7,1],[8,1],[8,0]]
        off_set_x = abs(init_pose.pose.position.x/4)
        off_set_y = abs(init_pose.pose.position.y/4)
    else:
        init_position = [0,9]
        target_position = [[0,9],[1,9],[2,9],[2,8],[2,7],[3,7],[4,7],[4,6],[4,5],[3,5],[2,5],[2,4],[2,3],[2,3],[3,3],[3,2],[3,1],[4,1],[5,1],[6,1],[7,1],[8,1],[9,1],[9,0]]
        off_set_x = abs(init_pose.pose.position.x/5)
        off_set_y = abs(init_pose.pose.position.y/4)
    #target_position = [[8,9],[7,9],[7,8],[7,7],[8,7],[8,6],[8,5],[7,5],[7,4],[7,3],[6,3],[5,3],[4,3],[3,3],[3,2],[3,1],[4,1],[5,1],[6,1],[7,1],[8,1],[8,0]]

    global twist
    twist = Twist()
    
    move_robot(init_position, target_position, off_set_x, off_set_y)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("agent", help="agnet number")
    #parser.add_argument("output", help="output file with the schedule")
    args = parser.parse_args()
    main(args.agent)