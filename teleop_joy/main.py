#!/usr/bin/env python3

from teleop_joy.joy_nav2_commander import JoyNav2Commander


POSITION_X_A = -1.7981275173738125        # 목표지점 좌표입력 (A)
POSITION_Y_A = -0.5336102444608362
POSITION_Z_A = 0.0
ORIENTATION_X_A = 0.0
ORIENTATION_Y_A = 0.0
ORIENTATION_Z_A = 0.025922690997958475    # 목표지점 좌표입력 (B)
ORIENTATION_W_A = 0.9996639505811062

POSITION_X_B = 1.628069394226383
POSITION_Y_B = -0.5859367258993522
POSITION_Z_B = 0.0
ORIENTATION_X_B = 0.0
ORIENTATION_Y_B = 0.0
ORIENTATION_Z_B = 0.005805025721225324
ORIENTATION_W_B = 0.9999831506962384

POSITION_X_X = 0.7122737616003704         # 목표지점 좌표입력 (X)
POSITION_Y_X = 1.734632969013493
POSITION_Z_X = 0.0
ORIENTATION_X_X = 0.0
ORIENTATION_Y_X = 0.0
ORIENTATION_Z_X = 0.977773461969733
ORIENTATION_W_X = 0.20966415303461622

POSITION_X_Y = 0.6102191064004384         # 목표지점 좌표입력 (Y)
POSITION_Y_Y = -1.6064363532412094
POSITION_Z_Y = 0.0
ORIENTATION_X_Y = 0.0
ORIENTATION_Y_Y = 0.0
ORIENTATION_Z_Y = -0.6873677076180454
ORIENTATION_W_Y = 0.7263095996363488

POSITION_A = [POSITION_X_A,
              POSITION_Y_A,
              POSITION_Z_A]
ORIENTATION_A = [ORIENTATION_X_A,
                 ORIENTATION_Y_A,
                 ORIENTATION_Z_A,
                 ORIENTATION_W_A]

POSITION_B = [POSITION_X_B,
              POSITION_Y_B,
              POSITION_Z_B]
ORIENTATION_B = [ORIENTATION_X_B,
                 ORIENTATION_Y_B,
                 ORIENTATION_Z_B,
                 ORIENTATION_W_B]

POSITION_X = [POSITION_X_X,
              POSITION_Y_X,
              POSITION_Z_X]
ORIENTATION_X = [ORIENTATION_X_X,
                 ORIENTATION_Y_X,
                 ORIENTATION_Z_X,
                 ORIENTATION_W_X]

POSITION_Y = [POSITION_X_Y,
              POSITION_Y_Y,
              POSITION_Z_Y]
ORIENTATION_Y = [ORIENTATION_X_Y,
                 ORIENTATION_Y_Y,
                 ORIENTATION_Z_Y,
                 ORIENTATION_W_Y]

LEFT_TRIGGER_THRESHOLD = 0.0
RIGHT_TRIGGER_THRESHOLD = 0.0

node_instance = JoyNav2Commander()


def nav2_goal_command(pose: list, orient: list): #목표지점으로 이동을 위한 죄표값이 들어 있는 [ pose & orient ]
    if node_instance.go_to_goal(pose, orient):
        print("go pose :", pose)
        print("go orient :", orient)
        while not node_instance.is_move_to_goal_complete():
            if node_instance.axes_value_list[node_instance.RIGHT_TRIGGER] <= RIGHT_TRIGGER_THRESHOLD:
                node_instance.cancel_move_to_goal()
            node_instance.ros_spin_once()

        print("목표로 이동")
    else:
        print("rejected nav2 go to goal command...")




def main(args=None):

    node_instance.start()

    while True:
        if node_instance.axes_value_list[node_instance.LEFT_TRIGGER] <= LEFT_TRIGGER_THRESHOLD:
        #    print("Left Trigger!")
            if node_instance.button_status_list[node_instance.BUTTON_A]: # 좌표가 저장된 버튼을 누름
                print("go to a")
                nav2_goal_command(POSITION_A, ORIENTATION_A)             # 리스트에 담긴 좌표가 불러저옴
            elif node_instance.button_status_list[node_instance.BUTTON_B]:
                print("go to b")
                nav2_goal_command(POSITION_B, ORIENTATION_B)
            elif node_instance.button_status_list[node_instance.BUTTON_X]:
                print("go to x")
                nav2_goal_command(POSITION_X, ORIENTATION_X)
            elif node_instance.button_status_list[node_instance.BUTTON_Y]:
                print("go to y")
                nav2_goal_command(POSITION_Y, ORIENTATION_Y)

        node_instance.ros_spin_once()



if __name__ == '__main__':
    main()
