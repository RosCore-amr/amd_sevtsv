#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
from robot_interfaces.srv import CommandApi
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from datetime import datetime, timedelta, timezone
import ast
import yaml
import json
import random
import dateutil.parser


class RobotControlSystem(Node):

    def __init__(self):
        super().__init__("robot_control")
        # self.srv = self.create_service(
        #     AddTwoInts, "add_two_ints", self.add_two_ints_callback
        # )
        self.timer_cb = MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(
            5.0, self.main_loop, callback_group=self.timer_cb
        )

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.loop_timer)
        self.subscriber_ = self.create_subscription(
            String, "robot_status", self.robot_status_callback, 1
        )
        self.cli_robot_update_status = self.create_client(
            CommandApi, "update_standard_robot_status"
        )
        self.response_all_robot = {}

    def calc_diff(self, time_start, time_end):

        d_start = datetime.strptime(time_start, "%Y-%m-%d %H:%M:%S.%f")
        d_end = datetime.strptime(time_end, "%Y-%m-%d %H:%M:%S.%f")
        time_result = d_start - d_end
        # self.data_log[2] = d_end - d_start
        if d_start < d_end:
            return False
        return True
        return d_start - d_end

    def update_status_robot(self, response):

        self.get_logger().info('result : "%s"' % (response))

        req = CommandApi.Request()
        while not self.cli_robot_update_status.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        req.url = "update_robotStatus"
        req.msg_request = str(response)
        future = self.cli_robot_update_status.call_async(req)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

        return None

    def robot_status_callback(self, msg):
        self.response_all_robot = eval(msg.data)

    def robot_status_processing(self, response_all_robot):
        try:
            for i in range(len(response_all_robot)):
                robot_time = response_all_robot[i]["lastUpdate"]["date"]["$date"]
                robot_datetime_obj = (dateutil.parser.parse(robot_time)).strftime(
                    "%Y-%m-%d %H:%M:%S.%f"
                )
                # self.get_logger().info(f"robot_datetime_obj data: {robot_datetime_obj}")

                if not self.calc_diff(robot_datetime_obj, self.expire_now):
                    if response_all_robot[i]["status"] != 0:
                        _robot_update_status = {
                            "robot_code": response_all_robot[i]["robot_code"],
                            "status": 0,
                        }
                        update_status = self.update_status_robot(_robot_update_status)
                        robot_code_update = eval(update_status.msg_response)
                        # self.get_logger().info(
                        #     '_robot_update_status : "%s"' % (robot_code_update)
                        # )
                        return robot_code_update

            return True

        except Exception as e:
            self.get_logger().info(e)
            return None

    def loop_timer(self):
        self.expire_now = (datetime.now() - timedelta(minutes=1)).strftime(
            "%Y-%m-%d %H:%M:%S.%f"
        )
        self.get_logger().info('time end : "%s"' % (self.expire_now))

    def main_loop(self) -> None:

        # if self.query_mission_take_cart_empty:
        #     new_mission_code = self.creat_mission_take_empty_cart()
        #     self.get_logger().info(str(new_mission_code))
        process = self.robot_status_processing(self.response_all_robot)
        self.get_logger().info('response_all_robot end : "%s"' % (process))

        self.get_logger().info("loop run")


def main(args=None):
    rclpy.init(args=args)

    robot_control = RobotControlSystem()
    executor = MultiThreadedExecutor()
    executor.add_node(robot_control)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
