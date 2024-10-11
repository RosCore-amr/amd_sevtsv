#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rclpy
from rclpy.node import Node
from robot_interfaces.srv import CommandApi, GetInformation
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
            1.0, self.main_loop, callback_group=self.timer_cb
        )
        self.pub_robot_ability_workl_ = self.create_publisher(
            String, "robot_work_ability", 10
        )

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.loop_timer)
        self.cli_data_update_status = self.create_client(
            CommandApi, "update_data_database"
        )

        self.cli_get2system = self.create_client(GetInformation, "get_from_system")
        self.dict_robot_work = {}
        # self.list_robot_work = []

    def calc_diff(self, time_start, time_end):

        d_start = datetime.strptime(time_start, "%Y-%m-%d %H:%M:%S.%f")
        d_end = datetime.strptime(time_end, "%Y-%m-%d %H:%M:%S.%f")
        time_result = d_start - d_end
        # self.data_log[2] = d_end - d_start
        if d_start < d_end:
            return False
        return True
        return d_start - d_end

    def processing_update_client(self, _url, request_body):

        req = CommandApi.Request()
        while not self.cli_data_update_status.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        req.url = _url
        req.msg_request = str(request_body)
        future = self.cli_data_update_status.call_async(req)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

        return None

    def get_inform_system_client(self, _url):
        req = GetInformation.Request()
        while not self.cli_get2system.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        req.url = _url
        future = self.cli_get2system.call_async(req)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
        return None

    def pub_robot_status(self, response):
        msg = String()
        msg.data = str(response)
        return msg

    def consider_ability_robot(self, robot):
        self.dict_robot_work.update(robot)
        self.pub_robot_ability_workl_.publish(
            self.pub_robot_status(self.dict_robot_work)
        )
        # self.get_logger().info(
        #     'dict_robot_work: "%s"' % self.msg2json(self.dict_robot_work)
        # )

    def robot_status_processing(self, response_all_robot):
        list_robot_work = list(self.dict_robot_work.keys())

        try:
            for i in range(len(response_all_robot)):

                robot_time = response_all_robot[i]["lastAT"]["$date"]
                robot_datetime_obj = (dateutil.parser.parse(robot_time)).strftime(
                    "%Y-%m-%d %H:%M:%S.%f"
                )
                if not self.calc_diff(robot_datetime_obj, self.expire_now):
                    if response_all_robot[i]["robot_connect"]:
                        _robot_update_status = {
                            "ip_machine": response_all_robot[i]["ip_machine"],
                            "robot_connect": False,
                            "map_code": "minhdeptrai",
                        }
                        update_status = self.processing_update_client(
                            "update_robotStatus", _robot_update_status
                        )
                        robot_code_update = eval(update_status.msg_response)
                        self.get_logger().info(
                            '_robot_update_status : "%s"' % (robot_code_update)
                        )
                        return robot_code_update

                # self.get_logger().info('list_robot_work : "%s"' % (list_robot_work))

                if not response_all_robot[i]["robot_connect"]:
                    _dict_robot = {
                        response_all_robot[i]["ip_machine"]: {
                            "robot_code": response_all_robot[i]["robot_code"],
                            "map_code": response_all_robot[i]["map_code"],
                            "robot_status": response_all_robot[i]["robot_status"],
                        }
                    }
                    self.consider_ability_robot(_dict_robot)
                else:
                    if response_all_robot[i]["ip_machine"] in list_robot_work:
                        self.dict_robot_work.pop(response_all_robot[i]["ip_machine"])

            return True

        except Exception as e:
            self.get_logger().info(e)
            return None

    def loop_timer(self):
        # self.get_logger().info("loop run")
        pass

    def main_loop(self) -> None:

        self.expire_now = (datetime.now() - timedelta(seconds=40)).strftime(
            "%Y-%m-%d %H:%M:%S.%f"
        )
        _robot_status = self.get_inform_system_client("all_robot")
        list_robot_status = eval(_robot_status.msg_response)
        # self.get_logger().info('list_robot_status: "%s"' % list_robot_status)
        self.robot_status_processing(list_robot_status)

    def msg2json(self, msg):
        # y = json.load(str(msg))
        return json.dumps(msg, indent=4)


def main(args=None):
    rclpy.init(args=args)

    robot_control = RobotControlSystem()
    executor = MultiThreadedExecutor()
    executor.add_node(robot_control)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
