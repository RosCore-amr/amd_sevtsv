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
import requests


class MissionRequestControl(Node):

    def __init__(self):
        super().__init__("mission_control_rmf")
        # self.publisher_ = self.create_publisher(String, "demotest", 10)
        timer_period = 0.5  # seconds
        self.timer_cb = MutuallyExclusiveCallbackGroup()
        self.timer = self.create_timer(
            timer_period, self.main_loop, callback_group=self.timer_cb
        )

        self.subscription_robot_ability_workl_ = self.create_subscription(
            String, "robot_work_ability", self.robot_work_callback, 10
        )

        self.pub_misison_current = self.create_publisher(
            String, "mission_current_request", 10
        )
        self.time_transport_callback = datetime.now(timezone.utc)

        self.cli_data_update_status = self.create_client(
            CommandApi, "update_data_database"
        )
        self.cli_mission_inforation = self.create_client(
            GetInformation, "get_mission_current"
        )
        self.cli_get2system = self.create_client(GetInformation, "get_from_system")

        # self.msg_mission_current = MissionCurrent()
        self.number_map = 2
        self._dict_excute_task_code = {}

    def robot_work_callback(self, msg):
        robot_list = eval(msg.data)
        if bool(self._dict_excute_task_code):
            # self.get_logger().info('robot_list : "%s"' % (robot_list))
            for ip_machine, robot_status in robot_list.items():
                # self.get_logger().info('robot_list : "%s"' % (robot_status))
                if robot_status["robot_status"] == "RUN":
                    self.mission_processing(robot_list[ip_machine], ip_machine)
                # self.get_logger().info('ip_machine : "%s"' % (robot_list[ip_machine]))

    def mission_processing(self, robot, ip_machine):
        _url_request = "http://" + ip_machine + ":2000/" + "query_mission"
        _mission = self.find_mission_for_robot(robot["map_code"], self.number_map)
        if _mission["excute_code"] is None:
            # self.get_logger().info('none mission : "%s"' % (_mission))
            return True
        # self.get_logger().info(' _url_request : "%s"' % (_url_request))
        # send_task_target = self.sent_post(_url_request, _mission)

    def sent_post(self, _url, _request_body):
        # request_body = {"status": status}

        try:
            res = requests.post(
                _url,
                # headers=self.__token_db,
                json=_request_body,
                timeout=3,
            )
            response = res.json()
            if response["metaData"]:
                return None
            return True
        except Exception as e:
            return None

    def process_information_misison(self, _mission_next, _excute_code):

        if _mission_next is None:
            return {"code": 0}

        _url = str("query_mission/" + _mission_next)
        current_misison_infor = self.information_mission_current_client(_url)
        current_misison_response = eval(current_misison_infor.msg_response)

        # self.get_logger().info(
        #     'current_misison_response: "%s"' % current_misison_response["mission_state"]
        # )

        _url_pop = "missions_excute_pop"
        request_pop = {"excute_code": _excute_code}
        if (
            not int(current_misison_response["code"])
            or current_misison_response["mission_state"] != 1
        ):
            self.processing_update_client(_url_pop, request_pop)

        return str(current_misison_response)

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

    def find_bulletin_system_client(self, _url):
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

    def tracking_system_mission(self, task_code):
        _url_task_code = "excute_mission/" + task_code
        mission_transport = self.find_bulletin_system_client(_url_task_code)
        _mission_transport = eval(mission_transport.msg_response)
        # return _mission_transport
        # self.get_logger().info('_mission_transport: "%s"' % _mission_transport)
        _task = None
        _first_mission = _mission_transport["mission_excute"]
        if len(_first_mission) != 0:
            _task = self.process_information_misison(
                _first_mission[0], _mission_transport["excute_code"]
            )

        mission_depend_map = {
            _mission_transport["map_code"]: {
                "excute_code": _mission_transport["excute_code"],
                "task_mission": _task,
            }
        }
        return mission_depend_map

    def pub_mission_current(self, response):
        msg = String()
        msg.data = str(response)
        return msg

    def find_mission_for_robot(self, _map_code, n):

        empty_mission = {
            "excute_code": None,
            "task_mission": {"mission_code": 0, "code": "dont have map code"},
        }

        _list_map_excutecode = ["pickup_locations", "return_locations"]
        if _map_code not in _list_map_excutecode:
            _reason = {"excute_code": {"code": "dont have map code"}}
            # empty_mission.update(_reason)
            return empty_mission
            # return "dont have map code"

        if n == 0:
            return empty_mission
        mission_infor = self._dict_excute_task_code[_map_code]
        # self.get_logger().info('mission_infor: "%s"' % mission_infor["task_mission"])
        if mission_infor["task_mission"] is None:
            _list_map_excutecode.remove(_map_code)
            return self.find_mission_for_robot(_list_map_excutecode[0], n - 1)
        return mission_infor

    def consider_and_publish_mission(self):
        dict_mission_current = {}
        for map_code, mission_excute in self._dict_excute_task_code.items():
            dict_misison_excute = {
                mission_excute["excute_code"]: mission_excute["task_mission"]
            }
            dict_mission_current.update(dict_misison_excute)
        self.pub_misison_current.publish(self.pub_mission_current(dict_mission_current))
        # self.get_logger().info('dict_mission_current: "%s"' % dict_mission_current)

    def main_loop(self):
        _excute_task_code = ["transport_empty_cart", "transport_goods"]
        # _excute_task_code = ["transport_empty_cart"]
        for task_code in _excute_task_code:

            mission_track = self.tracking_system_mission(task_code)
            self._dict_excute_task_code.update(mission_track)

        #     _list_excute_task_code.append(mission_track)
        # mission_robot = self.find_mission_for_robot("pickup_locations", 2)
        self.consider_and_publish_mission()
        # self.get_logger().info(
        #     'self._dict_excute_task_code: "%s"' % self._dict_excute_task_code
        # )


def main(args=None):
    rclpy.init(args=args)
    mission_control_rmf = MissionRequestControl()
    executor = MultiThreadedExecutor()
    executor.add_node(mission_control_rmf)
    executor.spin()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
