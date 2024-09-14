import rclpy
from rclpy.node import Node
import yaml
import json
import os
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import requests
from robot_interfaces.msg import InforSevt, Token, LocationSent, MissionTransport
from robot_interfaces.srv import CreatMission, SearchStock, ExcuteMission
from example_interfaces.srv import AddTwoInts

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
        self._transport_goods = self.create_publisher(
            MissionTransport, "transport_goods", 10
        )
        self._transport_empty_cart = self.create_publisher(
            MissionTransport, "transport_empty_cart", 10
        )

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.loop_timer)
        self.timer_query_mission = self.create_timer(10, self.mission_timer)

        self.key_token = False
        self.token = ""
        self.harsh_token = ""
        self.i = 0
        config_path = os.path.join(
            get_package_share_directory("amd_sevtsv"), "config", "simul_robot.json"
        )
        self.config_file = self.load_config(config_path)
        self._url = self.config_file["url"]

        self.srv_new_mission = self.create_service(
            CreatMission, "creation_mission", self.creat_mission_srv
        )
        self.srv_search_empty_cart = self.create_service(
            SearchStock, "search_empty_cart", self.search_location_srv
        )
        self.srv_process_excute_mission = self.create_service(
            ExcuteMission, "processing_excute_mission", self.excute_misison_srv
        )

    def loop_timer(self):

        self.publisher_.publish(self.load_token())
        if not self.key_token:
            self.get_logger().error("SEVER NOT WORKING")
            exit()

        # mx = self.searching_information_db("excute_mission/transport_goods")
        # self.get_logger().info('Publishing: "%s"' % mx)
        # self.get_logger().info("minh dep trai - ---")
        # self.tracking_system_mission()
        self.pub_infor.publish(self.load_infor_server())
        self.pub_location_value.publish(self.pub_demo_location())

    def mission_timer(self):
        self.tracking_system_mission()

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

    def tracking_system_mission(self):

        mission_transport = self.res_mission_excute(
            self.searching_information_db("excute_mission/transport_goods")
        )
        mission_empty_cart = self.res_mission_excute(
            self.searching_information_db("excute_mission/transport_empty_cart")
        )

        self._transport_empty_cart.publish(mission_empty_cart)
        self._transport_goods.publish(mission_transport)

    def res_mission_excute(self, response):
        msg = MissionTransport()
        msg.excute_code = response["excute_code"]
        msg.mission_type = response["mission_type"]
        msg.mission_excute = response["mission_excute"]
        msg.mission_next = response["mission_next"]
        return msg

    def search_location_srv(self, request, response):
        req_api = self.searching_information_db(request.url)
        if not req_api or len(req_api) == 0:
            response.name = ""
            response.map_code = ""
            response.code = 0
            return response

        response.name = req_api[0]["name"]
        response.map_code = req_api[0]["map_code"]
        response.code = 1
        # response.name = "Asdasdas"
        # response.map_code = "asdasdasd"
        return response

    def creat_mission_srv(self, request, response):
        # response.sum = request.a + request.b
        # self.get_logger().info('I heard: "%s"' % request.location_code)
        # self.get_logger().info('I heard: "%s"' % request.map_code)
        # self.get_logger().info('I heard: "%s"' % request.occupy_code)
        request_db = {
            "entry_location": {
                "location_code": request.entry_location.location_code,
                "map_code": request.entry_location.map_code,
            },
            "end_location": {
                "location_code": request.end_location.location_code,
                "map_code": request.end_location.map_code,
            },
        }
        response_api = self.post_data_to_database("creat_mission", request_db)
        self.get_logger().info('I heard: "%s"' % response_api)
        response.mission_code = response_api["mission_code"]
        response.msg = str(response_api["mission_state"])
        return response

    def excute_misison_srv(self, request, response):
        request_db = {
            "excute_code": request.excute_code,
            "mission_excute": request.value,
        }
        response_api = self.patch_data_to_database(request.url, request_db)
        self.get_logger().info('response_api: "%s"' % response_api)
        response.code = "1"
        return response

    def searching_information_db(self, url):
        # self.get_logger().info("minh dep trai - ---")
        try:
            res = requests.get(self._url + url, headers=self.token, timeout=3)
            response_restapi = res.json()
            # print("response_restapi", response_restapi)
            # self.get_logger().info('funtion here: "%s"' % response_restapi)
            return response_restapi
            # response.code = response_restapi["code"]
            # response.msg = response_restapi["msg"]
        except Exception as e:
            print("error update status mission")
        return False

    def post_data_to_database(self, url, value):
        try:
            res = requests.post(
                self._url + url,
                json=value,
                headers=self.token,
                timeout=3,
            )

            response_post_data = res.json()
            return response_post_data
        except Exception as e:
            print("error update status mission")
        return False

    def patch_data_to_database(self, url, value):
        try:
            res = requests.patch(
                self._url + url,
                json=value,
                headers=self.token,
                timeout=3,
            )

            response_post_data = res.json()
            return response_post_data
        except Exception as e:
            print("error update status mission")
        return False


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ServerControl()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
