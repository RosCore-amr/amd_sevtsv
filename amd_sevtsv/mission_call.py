#!/usr/bin/env python
# -*- coding: utf-8 -*-

# from query_db import ServerControl
from .query_db import ServerControl

import rclpy
from rclpy.node import Node
from robot_interfaces.srv import SearchStock
from example_interfaces.srv import AddTwoInts


class MissionControlSystem(Node):

    def __init__(self):
        super().__init__("mission_control")
        # self.srv = self.create_service(
        #     AddTwoInts, "add_two_ints", self.add_two_ints_callback
        # )
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.loop_timer)
        self.cli_2point = self.create_client(AddTwoInts, "add_two_ints")
        self.get_logger().info("minh dep trai...")

    def loop_timer(self):
        # mx = ServerControl.minhdeptrai(self, "find_cart_empty6")
        # self.get_logger().info(mx)
        self.test()

    def test(self):
        while not self.cli_2point.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return True

        req = AddTwoInts.Request()
        req.a = 4
        req.b = 6
        future = self.cli_2point.call_async(req)
        self.get_logger().info(str(future))
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(str(future))
        try:
            response = future.result()
            self.get_logger().info(str(response.sum))
            # if response is not None:
            # self.hhhmmm(response)
            # hh = "asdasdasda"
            return response
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e))
        return response


def main(args=None):
    rclpy.init(args=args)

    mission_control = MissionControlSystem()

    rclpy.spin(mission_control)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
