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


class MissionRequestControl(Node):

    def __init__(self):
        super().__init__("mission_control_rmf")
        self.publisher_ = self.create_publisher(String, "demotest", 10)
        timer_period = 0.5  # seconds
        self.timer_cb = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            timer_period, self.main_loop, callback_group=self.timer_cb
        )
        self.subscription_transport_goods = self.create_subscription(
            MissionTransport, "transport_goods", self.transport_goods_callback, 10
        )
        self.subscription_transport_empty_cart = self.create_subscription(
            MissionTransport,
            "transport_empty_cart",
            self.transport_empty_cart_callback,
            10,
        )
        self.subscription_transport_goods  # prevent unused variable warning
        self.subscription_transport_empty_cart

        self._next_mission_transport_goods = None
        self._next_mission_transport_empty_cart = None
        self._current_mission_transport_goods = None
        self.time_transport_callback = datetime.now(timezone.utc)
        self.cli_mission_inforation = self.create_client(
            GetInformation, "get_mission_current"
        )
        self.pub_misison_current = self.create_publisher(
            MissionCurrent, "mission_current_request", 10
        )
        self.cli_data_update_status = self.create_client(
            CommandApi, "update_data_database"
        )
        self._mission_next_transport_goods = "1"
        self._mission_next_transport_empty_cart = "1"
        self.msg_mission_current = MissionCurrent()

    def transport_goods_callback(self, msg):

        # self._mission_next_transport_goods = msg.mission_next
        self.time_transport_callback = datetime.now(timezone.utc) + timedelta(seconds=5)
        if len(msg.mission_excute) == 0:
            self._mission_next_transport_goods = 0
        else:
            self._mission_next_transport_goods = msg.mission_excute[0]

    def transport_empty_cart_callback(self, msg):
        # self.get_logger().info(msg.mission_excute)

        if len(msg.mission_excute) == 0:
            self._mission_next_transport_empty_cart = 0
        else:
            self._mission_next_transport_empty_cart = msg.mission_excute[0]

        # self.get_logger().info(
        #     '_mission_next_transport_empty_cart: "%s"'
        #     % (self._mission_next_transport_empty_cart)
        # )

    def main_loop(self):
        # msg = String()
        # self.get_logger().info(
        #     '_mission_next_transport_empty_cart: "%s"'
        #     % self._mission_next_transport_empty_cart
        # )
        self.mission_current_pub()

    def mission_current_pub(self):
        now_time = datetime.now(timezone.utc)
        if now_time > self.time_transport_callback:
            self.get_logger().info("misison callback died")

        else:

            self.get_logger().info("on looop")

            current_misison_infor = self.process_information_misison(
                self._mission_next_transport_goods, "transport_empty_cart"
            )
            self.get_logger().info(
                'current_misison_infor: "%s"' % (current_misison_infor)
            )

            # if self._next_mission_transport_goods != self._mission_next_transport_goods:

            #     # if self._mission_next_transport_goods is None:
            #     # self.get_logger().info(
            #     #     'I _mission_next_transport_goods: "%s"'
            #     #     % len(self._mission_next_transport_goods)
            #     # )
            #     current_misison_infor = self.process_information_misison(
            #         self._mission_next_transport_goods, "transport_goods"
            #     )
            #     # self.get_logger().info(current_misison_infor)
            #     # self.get_logger().info(
            #     #     'current_misison_infor: "%s"' % current_misison_infor
            #     # )
            #     # self.msg_mission_current.transport_goods = current_misison_infor
            #     self._next_mission_transport_goods = self._mission_next_transport_goods
            # if (
            #     self._next_mission_transport_empty_cart
            #     != self._mission_next_transport_empty_cart
            # ):
            #     # self.get_logger().info(
            #     #     ' _mission_next_transport_empty_cart: "%s"'
            #     #     % len(self._mission_next_transport_empty_cart)
            #     # )
            #     current_misison_infor = self.process_information_misison(
            #         self._mission_next_transport_empty_cart, "transport_empty_cart"
            #     )
            #     # self.msg_mission_current.transport_empty_cart = current_misison_infor
            #     self._next_mission_transport_empty_cart = (
            #         self._mission_next_transport_empty_cart
            #     )
            #     # self.get_logger().info('I heard: "%s"' % current_misison_infor)

            # # self.get_logger().info("sever control")
            # self.pub_misison_current.publish(self.msg_mission_current)

        # self.get_logger().info("loop mission control")

    def process_information_misison(self, _mission_next, _excute_code):

        if not _mission_next:
            return {"code": 0}

        _url = "query_mission/" + _mission_next
        current_misison_infor = self.information_mission_current_client(_url)
        # current_misison_response = current_misison_infor.msg_response
        self.get_logger().info('current_misison_infor: "%s"' % current_misison_infor)
        # _url_pop = "missions_excute_pop"
        # request_pop = {"excute_code": _excute_code}
        # if not current_misison_response["code"]:
        #     self.processing_update_client(_url_pop, request_pop)

        # return current_misison_response

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

    def information_mission_current_client(self, _url):
        req = GetInformation.Request()
        while not self.cli_mission_inforation.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            return False

        req.url = _url
        future = self.cli_mission_inforation.call_async(req)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
        return None


def main(args=None):
    rclpy.init(args=args)
    mission_control_rmf = MissionRequestControl()
    executor = MultiThreadedExecutor()
    executor.add_node(mission_control_rmf)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
