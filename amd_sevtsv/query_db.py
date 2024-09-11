import rclpy
from rclpy.node import Node
import yaml
import json
import os
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import requests
from tutorial_interfaces.msg import InforSevt, Token, LocationSent
from tutorial_interfaces.srv import LocationAsk

# from rclpy_message_converter import message_converter

# from rospy_message_converter import json_message_converter


class ServerControl(Node):

    def __init__(self):
        super().__init__("ServerSEVT")
        self.publisher_ = self.create_publisher(Token, "token", 10)
        self.pub_infor = self.create_publisher(InforSevt, "infor_value", 10)
        self.pub_location_value = self.create_publisher(
            LocationSent, "stock_location", 10
        )
        self.mission_transport_goods = self.create_publisher(
            Token, "mission_transport_goods", 10
        )

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.loop_timer)
        # self.declare_parameter("username", "admin")
        # self.declare_parameter("password", "admin")
        self.key_token = False
        self.token = ""
        self.harsh_token = ""
        self.i = 0
        config_path = os.path.join(
            get_package_share_directory("amd_sevtsv"), "config", "simul_robot.json"
        )
        self.config_file = self.load_config(config_path)
        self.srv_location_available = self.create_service(
            LocationAsk, "check_location_available", self.manifest_location_srv
        )

        self._url = self.config_file["url"]

    def loop_timer(self):

        self.publisher_.publish(self.load_token())
        if not self.key_token:
            self.get_logger().error("SEVER NOT WORKING")
            exit()

        # mx = self.searching_information_db("excute_mission/transport_goods")
        # self.get_logger().info('Publishing: "%s"' % mx)
        # self.get_logger().info("minh dep trai - ---")
        self.pub_infor.publish(self.load_infor_server())
        self.pub_location_value.publish(self.pub_demo_location())

    def mockup_status(self):
        request_body = {
            "robot_name": "robot_1",
            "battery": 100,
            "status": "ide",
            "mission": "",
            "type": "amr",
        }
        self.get_logger().info('funtion here: "%s"' % type(self.token))

        try:
            # self.get_logger().info(self._url)
            url = "http://192.168.1.6:8000/update_robotStatus"
            res = requests.patch(
                url,
                headers=self.token,
                json=request_body,
                timeout=3,
            )
            response = res.json()
        except Exception as e:
            self.get_logger().info(e)
            return None

    def load_token(self):
        msg = Token()
        if not self.key_token:
            self.token = self.account_token(
                self.config_file["username"], self.config_file["password"]
            )

        msg.bearer = str(self.token)
        msg.harsh = str(self.harsh_token)
        return msg

    def pub_demo_location(self):
        msg = LocationSent()
        msg.entry_location.location_code = "zone4"
        msg.entry_location.map_code = "pickup_location"
        msg.end_location.location_code = "zone9"
        msg.end_location.map_code = "return_location"
        return msg

    def load_infor_server(self):
        infor_msg = InforSevt()
        infor_msg.username = self.config_file["username"]
        infor_msg.password = self.config_file["password"]
        infor_msg.url = self.config_file["url"]
        infor_msg.login = self.config_file["login"]
        infor_msg.updaterobotstatus = self.config_file["robot_status"]
        return infor_msg

    def msg2json(self, msg):
        y = yaml.load(str(msg))
        return json.dumps(y, indent=4)

    def load_config(self, path):
        config = {}
        with open(path, "r") as stream:
            try:
                d = yaml.load(stream, Loader=yaml.FullLoader)
                for key, val in d.items():
                    config[key] = val
                return config
            except yaml.YAMLError as e:
                print(e)

    def account_token(self, username, password):
        request_body = {"username": username, "password": password, "role": 0}
        purpose = self.config_file["login"]
        try:
            res = requests.post(
                self._url + purpose,
                json=request_body,
            )

            response = res.json()
            if response is not None:
                self.key_token = True
            self.harsh_token = response["token"]
            token_request = {"Authorization": "Bearer {}".format(response["token"])}

            return token_request

            return response["token"]
        except Exception as e:
            return None

    def searching_information_db(self, url):
        # self.get_logger().info("minh dep trai - ---")
        try:
            res = requests.get(
                "http://127.0.0.1:8000/" + url, headers=self.token, timeout=3
            )
            response_restapi = res.json()
            # print("response_restapi", response_restapi)
            # self.get_logger().info('funtion here: "%s"' % response_restapi)
            return response_restapi
            # response.code = response_restapi["code"]
            # response.msg = response_restapi["msg"]
        except Exception as e:
            print("error update status mission")
        return False

    def tracking_system_mission_srv(self, request, response):
        track = self.searching_information_db(request.url_track)
        # p

    def manifest_location_srv(self, request, response):
        # response.sum = request.a + request.b
        self.get_logger().info('I heard: "%s"' % request.location_code)
        self.get_logger().info('I heard: "%s"' % request.map_code)
        self.get_logger().info('I heard: "%s"' % request.occupy_code)
        request_db = {
            "location_code": request.location_code,
            "map_code": request.map_code,
            "occupy_code": request.occupy_code,
        }
        try:
            res = requests.post(
                self._url + "manifest_location",
                json=request_db,
                timeout=3,
            )
            response_restapi = res.json()
            response.code = response_restapi["code"]
            response.msg = response_restapi["msg"]
        except Exception as e:
            print("error update status mission")
        return response
        # pass


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ServerControl()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
