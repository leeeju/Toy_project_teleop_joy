# ROS2_teleop_joy vr.1

구성환경 : ubuntu20.04, ROS_foxy, PS3_EX aquad air

조이스틱을 사용해서 터틀심 or 터틀봇 또는 기타 로봇을 제어 합니다 

버전 1임으로 코드 정리가 아직 안됬습니다.

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

다운받은 파일을 실행시켜 조이스틱을 움직여 봅니다.

```bash
< 로봇이 갑자기 움직이는것을 막기 위하여 LB 버튼을 누르고 조이스틱을 사용해야 움직이도록 만들었습니다 >
``` 


![SNOW_20211207_183032_665](https://user-images.githubusercontent.com/84003327/145126621-e9895196-adf6-4469-b07d-16c8659a85cd.jpg)

