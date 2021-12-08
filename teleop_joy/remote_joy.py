import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class remotejoy(Node):

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
            '/cmd_vel',      #  'turtle1/cmd_vel' 터틀심test
            qos_profile)

        self.tw = Twist()
        self.joy = Joy()


    def joy_callback(self, joy_msg):

            if joy_msg.buttons[6] == 1:
                self.tw.angular.z = joy_msg.axes[2]  # 회전
                self.tw.linear.x  = joy_msg.axes[1]  # 전진
                self.tw.angular.x = joy_msg.axes[3]
                #self.tw.linear.y  = joy_msg.axes[2]
                self._twist_pub.publish(self.tw)

            elif joy_msg.buttons[7] == 1: # <-- 이거잘 안됨 (비상정지)
                self.tw.linear.x = self.tw.angular.z =  0.0
    '''
        BUTTON_INDEX_AXES_NUMBER [0] = 좌측 좌,우
        BUTTON_INDEX_AXES_NUMBER [1] = 좌측 상,하
        BUTTON_INDEX_AXES_NUMBER [2] = 우측 좌,우
        BUTTON_INDEX_AXES_NUMBER [3] = 우측 상,하
    '''
def main(args=None):

    rclpy.init(args=args)

    remote_joy = remotejoy()

    rclpy.spin(remote_joy)

    remote_joy.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
