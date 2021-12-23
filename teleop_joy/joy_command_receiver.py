import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped
# from teleop_joy.nav2_action_class import Nav2ActionNodeExample


CONTROLLER_MSG = '''
---------------------------------------
               increase               -----------
               linear_x               |RB button| is emergency stop.
                 +---+                -----------
                 | △ |                |LB button| is Safety_button
   increase  +---+---+---+ decrease   -----------
   angular_y | ◁ |   | ▷ | agular_y   ----------------------
             +---+---+---+            |FollowWaypoints 1 button Y
                 | ▽ |                |FollowWaypoints 2 button X
                 -----                |FollowWaypoints 3 button B
                decrease              |FollowWaypoints 4 button A
                linear_x              ----------------------
---------------------------------------
Remote_joy Controller
Enjoy your Robot Control
---------------------------------------
'''


class CommandReceiver(Node, threading.Thread):

    def __init__(self):
        # rclpy.init()
        super().__init__('command_receiver')
        threading.Thread.__init__(self)
        qos_profile = QoSProfile(depth=10)
        self._joy_sub = self.create_subscription(Joy,                  # 조이스틱을 사용하기 위한 섭스크라이브
                                                 'joy',
                                                 self.joy_callback,
                                                 qos_profile)

        # publisher
        self._twist_pub = self.create_publisher(Twist,
                                                '/cmd_vel',      # 'turtle1/cmd_vel' 터틀심test
                                                qos_profile)            # 조종을 위한 퍼블리셔

        self.button_move_pub_ = self.create_publisher(PoseStamped,      # 네비게이션을 통해 목표지점으로 가기 위힘 퍼블리셔
                                                      '/goal_pose',
                                                      qos_profile)

        self.setDaemon(True)

        self.BUTTON_A = 0 # 버튼을 누를떄 인덱스 값정의
        self.BUTTON_B = 1
        self.BUTTON_UNKWON_1 = 2
        self.BUTTON_X = 3
        self.BUTTON_Y = 4
        self.BUTTON_UNKWON_2 = 5
        self.BUTTON_LB = 6
        self.BUTTON_RB = 7
        self.BUTTON_LT = 8
        self.BUTTON_RT = 9
        self.BUTTON_BACK = 10
        self.BUTTON_START = 11
        self.BUTTON_XBOX = 12
        self.BUTTON_L3 = 13
        self.BUTTON_R3 = 14

        self.__BUTTON_INDEX_LAST = self.BUTTON_R3

        self.button_a_status = False # 버튼값 초기화
        self.button_b_status = False
        self.button_unkwon_1_status = False
        self.button_x_status = False
        self.button_y_status = False
        self.button_unkwon_2_status = False
        self.button_lb_status = False
        self.button_rb_status = False
        self.button_lt_status = False
        self.button_rt_status = False
        self.button_back_status = False
        self.button_start_status = False
        self.button_xbox_status = False
        self.button_l3_status = False
        self.button_r3_status = False

        self.__STATE_PUSHED = 1
        self.selected_command = None
        self.tw = Twist

        self.button_status_list = [self.button_a_status, # 버튼의 값을 리스트로 만들어 줍니다.
                                   self.button_b_status,
                                   self.button_unkwon_1_status,
                                   self.button_x_status,
                                   self.button_y_status,
                                   self.button_unkwon_2_status,
                                   self.button_lb_status,
                                   self.button_rb_status,
                                   self.button_lt_status,
                                   self.button_rt_status,
                                   self.button_back_status,
                                   self.button_start_status,
                                   self.button_xbox_status,
                                   self.button_l3_status,
                                   self.button_r3_status]

        self.__button_name_list = ["BUTTON A",     # 버튼의 이름 정의
                                   "BUTTON B",
                                   "BUTTON UNKWON 1",
                                   "BUTTON X",
                                   "BUTTON Y",
                                   "BUTTON UNKWON 2",
                                   "BUTTON LB",
                                   "BUTTON RB",
                                   "BUTTON LT",
                                   "BUTTON RT",
                                   "BUTTON BACK",
                                   "BUTTON START",
                                   "BUTTON XBOX",
                                   "BUTTON L3",
                                   "BUTTON R3"]

    def __del__(self):
        # self.__is_alive = False
        self.destroy_node()
        rclpy.shutdown()

    def joy_callback(self, joy_msg): # 콜백 함수
        axes_index = 0 # 조이스틱의 컨트롤 메세지
        for axes_value in joy_msg.axes:
            self.axes_value_list[axes_index] = axes_value
            axes_index += 1

        index = 0 # 버튼 인덱스 컨트롤 메세지
        for button_status in joy_msg.buttons:
            if button_status is self.__STATE_PUSHED:
                print(self.__button_name_list[index] + " pushed!!")
                self.button_status_list[index] = True
            else:
                self.button_status_list[index] = False
            index += 1


    def run(self) -> None:
        self.node_main()
        # self.__is_alive = self.is_alive()
        # while self.__is_alive:

    def node_main(self): # 출력을 조절하기 위한 슬립
        while True:
            time.sleep(1)
        # rclpy.spin(self)

    def ros_spin_once(self):
        rclpy.spin_once(self)
        '''
        spin 과 spin_once 의 차이점
         공통점 : 큐에 요청된 콜백함수를 처리함
         차이점 : [spin] = 프로그램이 종료될 때 까지 반복,
                 [spin_once] = 호출 시점까지 요청된 콜백함수를 처리
        '''


if __name__ == '__main__':
    node_instance = CommandReceiver()
    node_instance.start()
    while True:
        node_instance.ros_spin_once()
        time.sleep(0.0125)
