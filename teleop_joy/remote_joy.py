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
            'cmd_vel',      # 'turtle1/cmd_vel' 터틀심test
            qos_profile)

        self.action_client = ActionClient(  #액션 클라이언트 생성
            self, action_type = NavigateToPose,
            action_name = '/navigate_to_pose')

    def __navi_action_feedback_callback(self, msg):
        self.feedback = msg.feedback
        return

    def joy_callback(self, joy_msg):

        self.to_pose_msg = PoseStamped()
        tw = Twist()
        self.goal_msg = NavigateToPose.Goal()
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

        if BUTTON_INDEX_Safety_button == 1:

            tw.angular.z  = BUTTON_INDEX_AXES_Rotation  # 회전
            tw.linear.x   = BUTTON_INDEX_AXES_up_down   # 전진
           #tw.angular.x  = BUTTON_INDEX_AXES_left      # 죄측
            tw.linear.y   = BUTTON_INDEX_AXES_right     # 우측
            self._twist_pub.publish(tw)

        elif BUTTON_INDEX_Emergency == 1:           # 비상정지
            print("비상정지 버튼 작동")
            tw.linear.x  = Emergency_Speed
            tw.linear.y  = Emergency_Speed
            tw.angular.z = Emergency_Speed
            self._twist_pub.publish(tw)

        elif Follow_Waypoints_button_A == 1:      # 좌표가 저장된 버튼 (a)
            print("A지점으로 이동")
            self.to_pose_msg.pose.position.x    = 1.78
            self.to_pose_msg.pose.position.y    = 0.46
            self.to_pose_msg.pose.orientation.x = 0.0
            self.to_pose_msg.pose.orientation.y = 0.0
            self.goal_msg.pose = self.to_pose_msg

            self.action_client.send_goal_async(self.goal_msg,
            self.__navi_action_feedback_callback)

        elif Follow_Waypoints_button_B == 1:     # 좌표가 저장된 버튼 (b)
            print("B지점으로 이동")
            self.to_pose_msg.pose.position.x    = 0.615
            self.to_pose_msg.pose.position.y    = -1.84
            self.to_pose_msg.pose.orientation.x = 0.0
            self.to_pose_msg.pose.orientation.y = 0.0
            self.goal_msg.pose = self.to_pose_msg

            self.action_client.send_goal_async(self.goal_msg,
            self.__navi_action_feedback_callback)

        elif Follow_Waypoints_button_X == 1:      # 좌표가 저장된 버튼 (x)
            print("X지점으로 이동")
            self.to_pose_msg.pose.position.x    = -1.98
            self.to_pose_msg.pose.position.y    = -0.011
            self.to_pose_msg.pose.orientation.x = 0.0
            self.to_pose_msg.pose.orientation.y = 0.0
            self.goal_msg.pose = self.to_pose_msg

            self.action_client.send_goal_async(self.goal_msg,
            self.__navi_action_feedback_callback)

        elif Follow_Waypoints_button_Y == 1:      # 좌표가 저장된 버튼 (y)
            print("Y지점으로 이동")
            self.to_pose_msg.pose.position.x    = 0.554
            self.to_pose_msg.pose.position.y    = 1.81
            self.to_pose_msg.pose.orientation.x = 0.0
            self.to_pose_msg.pose.orientation.y = 0.0

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



