# ROS2_teleop_joy

## 구성환경 : ubuntu20.04, ROS_foxy, PS3_EX aquad air, turtlebot3

조이스틱을 사용해서 터틀심 or 터틀봇 또는 기타 로봇을 제어 합니다 

영상자료 : https://www.youtube.com/watch?v=6Z6ax9_ZUUw

```bash
수정 : 2021.12.09 네비게이션 클라이언트 추가
      2021.12.16 Follow_Waypoints_button 추가
       -> 문제점 최초 Waypoints로 이동은 하지만 그 이후 조작이 루프에 빠짐
      2021.12.20 최종본  
```

< 실행 절차 >

ros2를 설치 할때 joy pkg는 기본으로 설치가 됩니다, 구매한 조이스틱을 연결하고 노드를 실행해 봅니다.

```bash
ros2 run joy joy_node
```

```bash
turtle01@turtle01:~$ ros2 run joy joy_node 
[INFO]: No haptic (rumble) available, skipping initialization
[INFO]: Opened joystick: EX Squad Air.  deadzone: 0.050000
```
위와 비슷한 내용이 출력되면 잘 연결된겁니다 

그 후  ``` ros2 topic list ``` 를 통해서 /joy의 관련 토픽이 발행되고 있는지 확인 합니다 

```bash
turtle01@LeeJuHyun:~$ ros2 topic list 
/joy
/joy/set_feedback
/parameter_events
/rosout
```

그 다음  ``` ros2 topic echo /joy ``` 를 통해서 ``` /joy ``` 의 관련 토픽이 발행되는 각각의 버튼값을 확인합니다 

```bash
header:
  stamp:
    sec: 1639633130
    nanosec: 794432350
  frame_id: joy
axes:
- -0.0
- -0.0
- -0.0
- -0.0
- 1.0
- 1.0
- 0.0
- 0.0
buttons:
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
- 0
---
```

조이스틱의 각 버튼을 눌러 보면서 센서 신호가 들어오는 인덱스 값을 확인합니다.

다운받은 파일을 실행시켜 조이스틱을 움직여 봅니다.
```bash
ros2 launch nav2_bringup tb3_simulation_launch.py
```

런치파일을 사용해서 시뮬레이션 월드를 불러와서 터틀봇이 움직이는지 확인해 봅시다



```bash
 elif Follow_Waypoints_button_A == 1:      # 좌표가 저장된 버튼 (a)
            print("A지점으로 이동")
            self.to_pose_msg.pose.position.x    = -1.7981275173738125   <--- position.x 좌표
            self.to_pose_msg.pose.position.y    = -0.5336102444608362   <--- position.y 좌표
            self.to_pose_msg.pose.orientation.z = 0.025922690997958475  <--- orientation.z 좌표
            self.to_pose_msg.pose.orientation.w = 0.9996639505811062    <--- orientation.w 좌표

            self.goal_msg.pose = self.to_pose_msg
            self.action_client.send_goal_async(self.goal_msg, self.__navi_action_feedback_callback)
```

Rviz의 map 에서 현재 로봇의 좌표를 알고 싶을때는 ``` ros2 topic echo /amcl_pose ``` 를 사용하면 현재 로봇의 좌표를 알 수 있습니다 

```bash

turtle01@turtle01:~$ ros2 topic echo /amcl_pose 
header:
  stamp:
    sec: 7041
    nanosec: 400000000
  frame_id: map
pose:
  pose:
    position:
      x: -1.8932298019702885   <--- 현재 로봇의 x 좌표 
      y: -0.5099666005587765   <--- 현재 로봇의 y 좌표
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.18195151273753174   <--- 현재 로봇의 orientation z 좌표  
      w: 0.9833075037914253    <--- 현재 로봇의 orientation w 좌표
  covariance:
  - 0.18300436874565307
  - -0.013812092353768013
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - -0.013812092353768013
  - 0.21193439804069364
  - 0.0
  - 0.0
  - 0.0
    .
    .
    .
---

```


```bash
< 로봇이 갑자기 움직이는것을 막기 위하여 LB 버튼을 누르고 조이스틱을 사용해야 움직이도록 만들었습니다 >
``` 

<※ 주의 & 팁 ※>

ros를 기본적으로 설치 할때 바이너리 설치를 통해 받아진 패키지와 별도로 깃허브에서 받은 "같은 패키지"가 서로 있으면 충돌이 날 수 있다, 따라서 시스템에 설치 된 패키지를 삭제 할때는 

```bash
sudo apt remove ros-foxy-teleop-twist-joy
```

하면 된다..

