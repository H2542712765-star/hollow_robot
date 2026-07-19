#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import (
    BasicNavigator,
    TaskResult
)


class WaypointNavigator(Node):

    def __init__(self):

        super().__init__('waypoint_navigator')

        self.navigator = BasicNavigator()

        self.frame_id = 'map'

        self.get_logger().info(
            'Waypoint Navigator initialized'
        )


    def create_pose(
        self,
        x,
        y,
        yaw=0.0
    ):
        """
        创建导航目标点

        参数:
            x   : map坐标系X
            y   : map坐标系Y
            yaw : 机器人目标朝向(rad)
        """

        pose = PoseStamped()

        pose.header.frame_id = self.frame_id

        pose.header.stamp = (
            self.navigator
            .get_clock()
            .now()
            .to_msg()
        )

        pose.pose.position.x = x
        pose.pose.position.y = y

        # 平面机器人四元数
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0


        return pose



    def create_waypoints(self):

        """
        创建路点列表

        后续修改路径只需要改这里
        """

        waypoint_list = [

            (0.0, 0.0),

            (2.0, 0.0),

            (2.0, 2.0),

        ]


        poses = []

        for point in waypoint_list:

            pose = self.create_pose(
                point[0],
                point[1]
            )

            poses.append(pose)


        return poses



    def navigate(self):

        """
        执行路点导航
        """


        self.get_logger().info(
            'Waiting for Nav2 active...'
        )


        self.navigator.waitUntilNav2Active()


        waypoints = self.create_waypoints()


        self.get_logger().info(
            f'Sending {len(waypoints)} waypoints'
        )


        self.navigator.followWaypoints(
            waypoints
        )


        while not self.navigator.isTaskComplete():

            feedback = (
                self.navigator
                .getFeedback()
            )


            if feedback:

                self.get_logger().info(
                    f'Current waypoint: '
                    f'{feedback.current_waypoint}'
                )



        result = (
            self.navigator
            .getResult()
        )


        self.show_result(result)



    def show_result(self,result):

        if result == TaskResult.SUCCEEDED:

            self.get_logger().info(
                'Navigation succeeded'
            )


        elif result == TaskResult.CANCELED:

            self.get_logger().warning(
                'Navigation canceled'
            )


        elif result == TaskResult.FAILED:

            self.get_logger().error(
                'Navigation failed'
            )


        else:

            self.get_logger().error(
                'Unknown result'
            )



def main():

    rclpy.init()


    node = WaypointNavigator()


    try:

        node.navigate()


    except KeyboardInterrupt:

        node.get_logger().warning(
            'Navigation interrupted'
        )


    finally:

        node.destroy_node()

        rclpy.shutdown()



if __name__ == '__main__':

    main()