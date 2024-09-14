import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_interfaces.msg import InforSevt, Token, LocationStock
import requests
from datetime import datetime, timedelta, timezone
import ast
import yaml
import json
import random
import dateutil.parser


class ApiName:
    get_all_robot = "all_robot"


class MockupControl(Node):

    def __init__(self):
        super().__init__("mockupTest")
        self.sub_token = self.create_subscription(
            Token, "token", self.token_callback, 10
        )
        self.infor_severcontrol = self.create_subscription(
            InforSevt, "infor_value", self.infor_sever_callback, 10
        )
        # self.pub_infor = self.create_publisher(LocationStock, "infor_value", 10)

        self.sub_token  # prevent unused variable warning
        self.infor_severcontrol
        timer_period = 3  # seconds
        self._token_request = ""
        self._url = ""
        self._update_status = ""
        self.token = False
        self.infor_sever = False
        self.timer = self.create_timer(timer_period, self.mockup_timer)
        self.main_loop = self.create_timer(1, self.loop_timer)

    def calc_diff(self, time_start, time_end):

        d_start = datetime.strptime(time_start, "%Y-%m-%d %H:%M:%S.%f")
        d_end = datetime.strptime(time_end, "%Y-%m-%d %H:%M:%S.%f")
        time_result = d_start - d_end
        # self.data_log[2] = d_end - d_start
        if d_start < d_end:
            return False
        return True
        return d_start - d_end

    def mockup_timer(self):
        # self.get_logger().info('I heard: "%s"' % self.token_request)

        # self.pub_infor.publish(self.load_infor_server())
        # self.publisher_.publish(self.load_token())

        if self.token and self.infor_sever:

            # for i in range(2, 4):
            request_body = {
                "robot_code": "robot_" + str(random.randint(1, 5)),
                "battery": random.randint(5, 100),
                "status": random.randint(1, 6),
                "mission": "",
                "type": "amr",
            }
            self.update_status_robot(request_body)

    def loop_timer(self):
        # return True
        # self.now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        self.expire_now = (datetime.now() - timedelta(minutes=1)).strftime(
            "%Y-%m-%d %H:%M:%S.%f"
        )
        if self.infor_sever:
            self.query_allstatusrobot()

    def query_allstatusrobot(self):
        try:
            res = requests.get(
                self._urlmockup + ApiName.get_all_robot,
                headers=self._token_request,
                # json=request_body,
                # timeout=3,
            )
            response = res.json()
            for i in range(len(response)):
                robot_time = response[i]["lastUpdate"]["date"]["$date"]
                robot_datetime_obj = (dateutil.parser.parse(robot_time)).strftime(
                    "%Y-%m-%d %H:%M:%S.%f"
                )
                # count_time = datetime.strptime(
                #     some_datetime_obj, "%Y-%m-%d %H:%M:%S.%f"
                # )
                # isoTime = datetime.fromisoformat(time_date)
                # tinhtoan = self.now - convert_date
                # calculator = datetime.now(timezone.utc) - time_date
                if not self.calc_diff(robot_datetime_obj, self.expire_now):
                    if response[i]["status"] != 0:
                        request_body = {
                            "robot_code": response[i]["robot_code"],
                            "status": 0,
                        }
                        self.update_status_robot(request_body)
                        # self.get_logger().info('time end : "%s"' % (robot_datetime_obj))
                # self.get_logger().info('result : "%s"' % (jj))

        except Exception as e:
            self.get_logger().info(e)
            return None

    def update_status_robot(self, _request_body):
        # request_body = {
        #     "robot_code": status_code["robot_code"],
        #     "status": 0,
        # }
        # self.get_logger().info('result : "%s"' % (request_body))
        try:
            res = requests.patch(
                self._urlmockup + self._update_status,
                headers=self._token_request,
                json=_request_body,
                timeout=3,
            )
            response = res.json()
            self.get_logger().info(
                'robot_update : "%s"' % (_request_body["robot_code"])
            )
            self.get_logger().info('result : "%s"' % (response))
            if response["code"] != "0":
                return None
            # print("reponse updateStatusMission", response)
            return response["code"]
        except Exception as e:
            print("error update status mission")

    # def mockup_status(self):
    #     for i in range(1, 5):
    #         # self.get_logger().info('funtion here: "%s"' % i)
    #         request_body = {
    #             "robot_code": "robot_" + str(i),
    #             "battery": random.randint(5, 100),
    #             "status": random.randint(0, 6),
    #             "mission": "",
    #             "type": "amr",
    #         }

    #         try:
    #             # self.get_logger().info(self._url)
    #             res = requests.patch(
    #                 self._urlmockup + self._update_status,
    #                 headers=self._token_request,
    #                 json=request_body,
    #                 timeout=3,
    #             )
    #             response = res.json()
    #             self.get_logger().info('funtion here: "%s"' % response)
    #         except Exception as e:
    #             self.get_logger().info(e)
    #             return None

    def infor_sever_callback(self, msg):
        self._update_status = str(msg.updaterobotstatus)
        self._urlmockup = str(msg.url)
        self.infor_sever = True

    def token_callback(self, msg):
        self._token_request = eval(msg.bearer)
        self.token = True


def main(args=None):
    rclpy.init(args=args)
    mockup_test = MockupControl()
    rclpy.spin(mockup_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    mockup_test.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
