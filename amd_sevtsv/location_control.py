import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime, timedelta, timezone

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from example_interfaces.srv import AddTwoInts
from functools import partial
from robot_interfaces.msg import MissionTransport, MissionCurrent
from robot_interfaces.srv import GetInformation, CommandApi


class LocationRequestControl(Node):

    def __init__(self):
        super().__init__("location_control_rmf")
        self.publisher_ = self.create_publisher(String, "demotest", 10)
        timer_period = 0.5  # seconds
        self.timer_cb = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            timer_period, self.main_loop, callback_group=self.timer_cb
        )

    def main_loop(self):
        # msg = String()
        # self.get_logger().info(
        #     '_mission_next_transport_empty_cart: "%s"'
        #     % self._mission_next_transport_empty_cart
        # )
        # self.mission_current_pub()
        pass


def main(args=None):
    rclpy.init(args=args)
    location_control_rmf = LocationRequestControl()
    executor = MultiThreadedExecutor()
    executor.add_node(location_control_rmf)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
