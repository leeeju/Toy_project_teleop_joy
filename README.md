# ROS2_teleop_joy vr.2

구성환경 : ubuntu20.04, ROS_foxy, PS3_EX aquad air

조이스틱을 사용해서 터틀심 or 터틀봇 또는 기타 로봇을 제어 합니다 

버전 2임으로 코드 정리가 아직 안됬습니다.

```bash
수정 : 2021.12.09 네비게이션 클라이언트 추가
      2021.12.16 Follow_Waypoints_button 추가
       -> 문제점 최초 Waypoints로 이동은 하지만 그 이후 조작이 루프에 빠짐
```

< 실행 절차 >

ros2를 설치 할때 joy pkg는 기본으로 설치가 됩니다, 구매한 조이스틱을 연결하고 노드를 실행해 봅니다.

```bash
ros2 run joy joy_node
```

```bash
turtle01@LeeJuHyun:~$ ros2 run joy joy_node 
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

그 다음  ``` ros2 topic echo /joy ``` 를 통해서 /joy의 관련 토픽이 발행되는 각각의 버튼값을 확인합니다 

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
< 로봇이 갑자기 움직이는것을 막기 위하여 LB 버튼을 누르고 조이스틱을 사용해야 움직이도록 만들었습니다 >
``` 


