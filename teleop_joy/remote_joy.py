from typing import Counter
import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped


Controller_msg = '''
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

class Remote_joy(Node):

    def __init__(self):
      
        super().__init__('remote_joy')
        qos_profile = QoSProfile(depth=10)
        self._joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            qos_profile)
        
        self._twist_pub = self.create_publisher(
            Twist,
            'turtle1/cmd_vel',      # '/cmd_vel' 로 변경하면 turtlebot3를 직접 조종 할 수 있다
            qos_profile)   

        self.action_client = ActionClient(                #액션 클라이언트 생성
            self, action_type = NavigateToPose,
            action_name = '/navigate_to_pose')

    def __navi_action_feedback_callback(self, msg):
        self.feedback = msg.feedback
        print(self.feedback)
        
        return

    def joy_callback(self, joy_msg):                     # 콜백 함수
        tw = Twist() 
        self.to_pose_msg = PoseStamped()
        self.goal_msg = NavigateToPose.Goal()            # nav2를 위한 pose 정보 정의 
                                                         # 조이스틱의 버튼값 정의
        BUTTON_INDEX_AXES_up_down   = joy_msg.axes[1]
        BUTTON_INDEX_AXES_Rotation  = joy_msg.axes[2]
        #BUTTON_INDEX_AXES_left     = joy_msg.axes[3]
        BUTTON_INDEX_AXES_right     = joy_msg.axes[0]

        BUTTON_INDEX_Safety_button  = joy_msg.buttons[6]
        BUTTON_INDEX_Emergency      = joy_msg.buttons[7]

        Follow_Waypoints_button_A   = joy_msg.buttons[0]
        Follow_Waypoints_button_B   = joy_msg.buttons[1]
        Follow_Waypoints_button_X   = joy_msg.buttons[3]
        Follow_Waypoints_button_Y   = joy_msg.buttons[4]
        Emergency_Speed = 0.0

        if BUTTON_INDEX_Safety_button == 1:                     # 로봇이 갑자기 움직이는것을 막기 위한 안전버튼 # 버튼을 누르고 조이스틱을 움직이세요

            tw.angular.z  = BUTTON_INDEX_AXES_Rotation  # 회전
            tw.linear.x   = BUTTON_INDEX_AXES_up_down   # 전진
           #tw.angular.x  = BUTTON_INDEX_AXES_left      # 죄측
            tw.linear.y   = BUTTON_INDEX_AXES_right     # 우측
            self._twist_pub.publish(tw)

        elif BUTTON_INDEX_Emergency == 1:           # 비상정지 모든 속도를 0 으로 만든다, 이동중 가속도의 대한 것은 없다, 로봇에 별도의 브레이크가 있지않음
            print("비상정지 버튼 작동")
            tw.linear.x  = Emergency_Speed
            tw.linear.y  = Emergency_Speed
            tw.angular.z = Emergency_Speed
            self._twist_pub.publish(tw)

        elif Follow_Waypoints_button_A == 1:      # 좌표가 저장된 버튼 (A)
            print("A지점으로 이동")
            self.to_pose_msg.pose.position.x    = -1.7981275173738125
            self.to_pose_msg.pose.position.y    = -0.5336102444608362
            self.to_pose_msg.pose.orientation.x = 0.025922690997958475
            self.to_pose_msg.pose.orientation.y = 0.9996639505811062
            self.goal_msg.pose = self.to_pose_msg

            self.action_client.send_goal_async(self.goal_msg,    #  self.goal_msg = NavigateToPose.Goal()로 정의      
            self.__navi_action_feedback_callback)

        elif Follow_Waypoints_button_B == 1:     # 좌표가 저장된 버튼 (B)
            print("B지점으로 이동")
            self.to_pose_msg.pose.position.x    = 1.628069394226383
            self.to_pose_msg.pose.position.y    = -0.5859367258993522
            self.to_pose_msg.pose.orientation.x = 0.005805025721225324
            self.to_pose_msg.pose.orientation.y = 0.9999831506962384
            self.goal_msg.pose = self.to_pose_msg

            self.action_client.send_goal_async(self.goal_msg,
            self.__navi_action_feedback_callback)

        elif Follow_Waypoints_button_X == 1:      # 좌표가 저장된 버튼 (X)
            print("X지점으로 이동")
            self.to_pose_msg.pose.position.x    = 0.7122737616003704
            self.to_pose_msg.pose.position.y    = 1.734632969013493
            self.to_pose_msg.pose.orientation.z = 0.977773461969733
            self.to_pose_msg.pose.orientation.w = 0.20966415303461622
            self.goal_msg.pose = self.to_pose_msg

            self.action_client.send_goal_async(self.goal_msg,
            self.__navi_action_feedback_callback)

        elif Follow_Waypoints_button_Y == 1:      # 좌표가 저장된 버튼 (Y)
            print("Y지점으로 이동")
            self.to_pose_msg.pose.position.x    = 0.6102191064004384
            self.to_pose_msg.pose.position.y    = -1.6064363532412094
            self.to_pose_msg.pose.orientation.z = -0.6873677076180454
            self.to_pose_msg.pose.orientation.w = 0.7263095996363488

            self.goal_msg.pose = self.to_pose_msg
            self.action_client.send_goal_async(self.goal_msg,
            self.__navi_action_feedback_callback)


def main(args=None):

    rclpy.init(args=args)
    remote_joy = Remote_joy()
    rclpy.spin(remote_joy)
    remote_joy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



