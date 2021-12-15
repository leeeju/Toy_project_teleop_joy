import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

Controller_msg = '''
---------------------------------------
               increase               -----------
               linear_x               |RB button| is emergency stop.
                 +---+                -----------
                 | △ |                |LB button| is Safety_button
   increase  +---+---+---+ decrease   -----------
   angular_z | ◁ |   | ▷ | agular_x
             +---+---+---+
                 | ▽ |
                 -----
                decrease
                linear_x
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
            msg_type = Joy,
            topic = 'joy',
            callback = self.joy_callback,
            qos_profile = qos_profile)


        self._twist_pub = self.create_publisher(
            msg_type = Twist,
            topic = 'cmd_vel',      # 'turtle1/cmd_vel' 터틀심test
            qos_profile = qos_profile)

        self.action_client = ActionClient(
            self, action_type = NavigateToPose,
            action_name='/navigate_to_pose')
        
    def joy_callback(self, joy_msg):

        tw = Twist()

        BUTTON_INDEX_AXES_up_down   = joy_msg.axes[1]
        BUTTON_INDEX_AXES_Rotation  = joy_msg.axes[2]
        #BUTTON_INDEX_AXES_left      = joy_msg.axes[3]
        BUTTON_INDEX_AXES_right     = joy_msg.axes[0]

        BUTTON_INDEX_Safety_button  = joy_msg.buttons[6]
        BUTTON_INDEX_Emergency      = joy_msg.buttons[7]
        Emergency_Speed = 0.0
        print(Controller_msg)

        if BUTTON_INDEX_Safety_button == 1:
           # print(Controller_msg)

            tw.angular.z  = BUTTON_INDEX_AXES_Rotation  # 회전

            tw.linear.x   = BUTTON_INDEX_AXES_up_down   # 전진

            #tw.angular.x  = BUTTON_INDEX_AXES_left     # 죄측
            tw.linear.y   = BUTTON_INDEX_AXES_right     # 우측

            self._twist_pub.publish(tw)

        elif BUTTON_INDEX_Emergency == 1:           # 비상정지
              print("Emergency_button_on")
              tw.linear.x  = Emergency_Speed
              tw.linear.y  = Emergency_Speed
              tw.angular.z = Emergency_Speed
              self._twist_pub.publish(tw)

def main(args=None):

    rclpy.init(args=args)

    remote_joy = Remote_joy()

    rclpy.spin(remote_joy)

    remote_joy.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()


