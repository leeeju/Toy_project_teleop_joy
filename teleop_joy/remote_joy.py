import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist



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
            'turtle1/cmd_vel',      # 'turtle1/cmd_vel' 터틀심test
            qos_profile)

        #self.__BUTTON_INDEX_AXES_UP   = axes[0]
        #self.__BUTTON_INDEX_AXES_DOWN = [1]
        #self.__BUTTON_INDEX_AXES_LT   = [2]
        #self.__BUTTON_INDEX_AXES_RT   = [3]

    def joy_callback(self, joy_msg):

        tw = Twist()

        BUTTON_INDEX_AXES_up_down   = joy_msg.axes[1]
        BUTTON_INDEX_AXES_Rotation  = joy_msg.axes[2]
        #BUTTON_INDEX_AXES_left      = joy_msg.axes[3]
        BUTTON_INDEX_AXES_right     = joy_msg.axes[0]

        BUTTON_INDEX_Safety_button  = joy_msg.buttons[6]
        BUTTON_INDEX_Emergency      = joy_msg.buttons[7]
        Emergency_Speed = 0.0


        if BUTTON_INDEX_Safety_button == 1:

            tw.angular.z  = BUTTON_INDEX_AXES_Rotation  # 회전
            tw.linear.x   = BUTTON_INDEX_AXES_up_down   # 전진
            #tw.angular.x  = BUTTON_INDEX_AXES_left     # 죄측
            tw.linear.y   = BUTTON_INDEX_AXES_right     # 우측
            self._twist_pub.publish(tw)

    #elif
        elif BUTTON_INDEX_Emergency == 1:           # 비상정지
              tw.linear.x  = Emergency_Speed
              tw.angular.z = Emergency_Speed
              self._twist_pub.publish(tw)
    '''
    BUTTON_INDEX_AXES_NUMBER [0] = 좌측 좌,우
    BUTTON_INDEX_AXES_NUMBER [1] = 좌측 상,하
    BUTTON_INDEX_AXES_NUMBER [2] = 우측 좌,우
    BUTTON_INDEX_AXES_NUMBER [3] = 우측 상,하
    '''
def main(args=None):

    rclpy.init(args=args)

    remote_joy = Remote_joy()

    rclpy.spin(remote_joy)

    remote_joy.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()


