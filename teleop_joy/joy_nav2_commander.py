from rclpy.qos import QoSProfile
from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import NavigateToPose

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped, Pose

from action_msgs.msg import GoalStatus

from rcl_interfaces.srv import SetParameters

import rclpy
import threading
import time


CONTROLLER_MSG = '''
---------------------------------------
               increase               -----------
               linear_x               |RB button| is emergency stop.
                 +---+                -----------
                 | △ |                |LB button| is Safety_button
   increase  +---+---+---+ decrease   -----------
   angular_z | ◁ |   | ▷ | agular_x   ----------------------
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

class JoyNav2Commander(Node, threading.Thread):
    def __init__(self, area_setting_path=None):
        rclpy.init()
        super().__init__('joy_nav2_commander')
        threading.Thread.__init__(self)

        qos_profile = QoSProfile(depth=10)
        self._joy_sub = self.create_subscription(Joy,     # joy 메세지 섭스크라이브
                                                 'joy',
                                                 self.joy_callback,
                                                 qos_profile)

        self._twist_pub = self.create_publisher(Twist,    # wist_mgs 퍼블리셔
                                                'turtle1/cmd_vel',   # 'turtle1/cmd_vel' 터틀심test
                                                qos_profile)

        self.button_move_pub_ = self.create_publisher(PoseStamped,   # PoseStamped 퍼블리셔 목표로 보내기 위한 퍼블리셔
                                                      '/goal_pose',
                                                      qos_profile)

        self.__qos_profile = QoSProfile(depth=10)
        self.srv_param = self.create_client(srv_type=SetParameters, # 클라이언트 생성
                                            srv_name='/controller_server/set_parameters',
                                            qos_profile=self.__qos_profile)

        self.action_client = ActionClient(self, action_type=NavigateToPose,  #네이게이션을 위한 포즈 메세지 생성
                                            action_name='/navigate_to_pose')

        self.sub_example_member_var = None

        self.setDaemon(True)
        self.tw = Twist

        self.__STATE_PUSHED = 1
        self.selected_command = None

        # nav2 action
        self.action_goal_handle = None
        self.action_result_future = None
        self.action_feedback = None
        self.action_status = None

        # joy stick 버튼 인덱스
        self.BUTTON_A = 0
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

        self.axes_value_list = [-0.0, -0.0, -0.0, -0.0, 1.0, 1.0, 0.0, 0.0]  # 버튼 입력상태 전의 초기값 정의

        self.__BUTTON_INDEX_LAST = self.BUTTON_R3

        self.button_a_status = False  # 버튼 초기화
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

        self.button_status_list = [self.button_a_status, #버튼 리스트 정의
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

        self.__button_name_list = ["BUTTON A", # 각 버튼의 이름 정의
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

        self.destroy_node()
        rclpy.shutdown()

    def joy_callback(self, joy_msg): # 버튼과 "joy"의 메세지를 받아서 인덱스에 있는 버튼값을 불러 옴

        axes_index = 0 # 조이스틱의 컨트롤 메세지
        for axes_value in joy_msg.axes:
            self.axes_value_list[axes_index] = axes_value
            axes_index += 1

        button_index = 0
        for button_status in joy_msg.buttons: # 반복문을 사용해서 버튼 값을 계속 받아옴
            if button_status is self.__STATE_PUSHED:
                print(self.__button_name_list[button_index] + " pushed!!")
                self.button_status_list[button_index] = True
            else:
                self.button_status_list[button_index] = False
            button_index += 1

    def example_sub_callback(self, msg):
        print(msg)
        print(msg.data)
        self.sub_example_member_var = msg.data

    def go_to_goal(self, goal_pose: list, orientation: list): #좌표값을 저장해두는 리스트
        # while self.action_client.wait_for_server(timeout_sec=1.0):
        #     print("wait navigate to pose action server...")
        #     time.sleep(1)

        print("목표로 이동을 시작합니다")
        goal_msg = NavigateToPose.Goal()
        to_pose_msg = PoseStamped()
        to_pose_msg.pose.position.x = goal_pose[0] # 리스트에 저장된 좌표(포즈 정보)를 가져옴
        to_pose_msg.pose.position.y = goal_pose[1]
        to_pose_msg.pose.position.z = goal_pose[2]
        to_pose_msg.pose.orientation.x = orientation[0]
        to_pose_msg.pose.orientation.y = orientation[1]
        to_pose_msg.pose.orientation.z = orientation[2]
        to_pose_msg.pose.orientation.w = orientation[3]
        goal_msg.pose = to_pose_msg

        future = self.action_client.send_goal_async(goal_msg, self.__navi_action_feedback_callback) # 클라이언트를 통해서 콜백으로 액션을 불러옴
        print("자리를 잡기 위하여 회전중")
        rclpy.spin_until_future_complete(self, future)
        print("회전중")
        self.action_goal_handle = future.result()
        if not self.action_goal_handle.accepted:
            print('목표도착에 실패 했습니다!')
            return False
        self.action_result_future = self.action_goal_handle.get_result_async()
        return True

    def cancel_move_to_goal(self): #목표로 이동중 목표를 취소하고자 할때
        try:
            print('현재 목표를 취소했습니다 ')
            if self.action_result_future:
                future = self.action_goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, future=future, timeout_sec=3.0)
            return True
        except Exception as e:
            print(str(e))
            return False

    def is_move_to_goal_complete(self): # 목표 지점에 도착 하면 실행됨
        if not self.action_result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.action_result_future, timeout_sec=0.10)
        if self.action_result_future.result():
            self.action_status = self.action_result_future.result().status
            if self.action_status != GoalStatus.STATUS_SUCCEEDED:
                print('목표 도착에 실패했습니다.!!!: {0}'.format(self.action_status))
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        print(' 목표도착 성공!!! ')
        return True

    def __navi_action_feedback_callback(self, msg): # 액션 콜백 메세지
        self.feedback = msg.feedback
        return

    def get_go_to_goal_result(self):
        return self.action_status

    def run(self) -> None:
        self.node_main()
        # self.__is_alive = self.is_alive()
        # while self.__is_alive:

    def node_main(self):
        while True:
            time.sleep(1)

    def ros_spin_once(self):
        rclpy.spin_once(self)


if __name__ == '__main__':
    node_instance = JoyNav2Commander()
    node_instance.start()
    while True:
        node_instance.ros_spin_once()
