# GF3 Robot System

GF3는 6자유도 로봇 매니퓰레이터와 메카넘 휠 이동 플랫폼을 결합한 모바일 매니퓰레이터 시스템입니다. ROS2를 기반으로 구축되었으며, Moteus 액츄에이터를 사용한 로봇팔과 GGM 모터를 사용한 메카넘 이동 베이스로 구성되어 있습니다.

2023 올해의작가상 수상작입니다

## 시스템 개요

### 하드웨어 구성
- **로봇팔**: 6자유도 매니퓰레이터 (Moteus 액츄에이터 기반)
- **이동 베이스**: 메카넘 휠 플랫폼 (MyActuator RMD 모터 4개)
- **센서**: RPLiDAR (SLAM 및 내비게이션용)
- **통신**: CAN 버스 통신

### 소프트웨어 아키텍처
- **ROS2 Humble** 기반
- **ros2_control** 프레임워크를 통한 하드웨어 추상화
- **MoveIt2** 모션 플래닝
- **Nav2** 자율 내비게이션
- **SLAM Toolbox** 맵핑

## 패키지 구조

### 1. gf3_description
로봇의 기계적 구조와 시각화를 담당하는 패키지입니다.

```
gf3_description/
├── urdf/
│   ├── gf3.urdf          # 로봇 기본 URDF
│   ├── gf3.xacro         # 메인 XACRO 파일
│   └── ros2_control.xacro # ros2_control 설정
├── meshes/               # 3D 메시 파일들
└── launch/
    ├── display.launch    # RViz 시각화
    └── gazebo.launch     # Gazebo 시뮬레이션
```

**주요 특징**:
- 5개 조인트로 구성된 로봇팔 (shoulder_1-2, shoulder_2-3, shoulder_3-upper, upper-elbow, elbow-lower)
- 각 조인트별 물리적 제한값 설정
- 3D 메시를 통한 정확한 시각화

### 2. gf3_hardware
하드웨어와의 인터페이스를 담당하는 핵심 패키지입니다.

```
gf3_hardware/
├── include/gf3_hardware/
│   ├── gf3_hardware_interface.hpp  # 메인 하드웨어 인터페이스
│   ├── mecanum_control.hpp         # 메카넘 제어
│   ├── MoteusAPI.h                 # Moteus 통신 API
│   └── myactuator.hpp              # MyActuator 제어
└── src/
    ├── gf3_hardware_interface.cpp  # ros2_control 구현
    ├── mecanum_control.cpp         # 메카넘 오도메트리 및 제어
    └── MoteusAPI.cpp               # Moteus 프로토콜 구현
```

**핵심 기능**:

#### A. Gf3HardwareInterface
- ros2_control SystemInterface 구현
- Moteus 액츄에이터와 topic 기반 통신
- Position 명령/상태 인터페이스 제공
- `/command` (MoteusCommandArray) 및 `/state` (MoteusStateArray) 토픽 사용

#### B. MecanumControl
- 4륜 메카넘 드라이브 제어
- CAN 통신을 통한 모터 제어
- 역기구학을 통한 cmd_vel → 개별 바퀴 속도 변환
- 오도메트리 계산 및 TF 브로드캐스팅

```cpp
// 메카넘 운동학 (cmd_vel → 바퀴 속도)
wheel_front_left = (1/WHEEL_RADIUS) * (linear.x - linear.y - (W+L)*angular.z)
wheel_front_right = (1/WHEEL_RADIUS) * (linear.x + linear.y + (W+L)*angular.z)
wheel_rear_left = (1/WHEEL_RADIUS) * (linear.x + linear.y - (W+L)*angular.z)
wheel_rear_right = (1/WHEEL_RADIUS) * (linear.x - linear.y + (W+L)*angular.z)
```

### 3. gf3_bringup
시스템 실행 및 설정을 담당하는 패키지입니다.

```
gf3_bringup/
├── launch/
│   ├── gf3.launch.py              # 메인 로봇 런치
│   ├── mecanum_launch.py          # 메카넘 제어
│   ├── mecanum_nav_launch.py      # 통합 내비게이션
│   ├── navigation_launch.py       # Nav2 런치
│   ├── slam_launch.py             # SLAM 런치
│   └── rplidar_launch.py          # LiDAR 런치
└── config/
    ├── gf3_controllers.yaml       # 컨트롤러 설정
    └── nav2_params.yaml           # 내비게이션 파라미터
```

**주요 런치 파일**:

#### gf3.launch.py
- ros2_control_node 실행
- robot_state_publisher 시작
- RViz 시각화
- 컨트롤러 spawning (joint_state_broadcaster, forward_position_controller)

#### mecanum_nav_launch.py
통합 내비게이션 시스템으로 다음을 포함:
- RPLiDAR 구동
- 메카넘 제어
- SLAM (동시 위치 추정 및 매핑)
- Nav2 자율 내비게이션
- 레이저 필터링

### 4. gf3_moveit_config
MoveIt2 모션 플래닝 설정 패키지입니다.

```
gf3_moveit_config/
├── config/
│   ├── robot.srdf                 # 시맨틱 로봇 설명
│   ├── kinematics.yaml           # 운동학 솔버 설정
│   ├── joint_limits.yaml         # 조인트 제한값
│   └── moveit_controllers.yaml   # MoveIt 컨트롤러 설정
└── launch/
    ├── demo.launch.py             # MoveIt 데모
    └── move_group.launch.py       # Move Group 서버
```

**기능**:
- 6자유도 팔의 모션 플래닝
- 충돌 감지 및 회피
- 역기구학 해석
- Trajectory 생성 및 실행

## 설치 및 빌드

### 사전 요구사항
```bash
# ROS2 Humble 설치 필요
sudo apt install ros-humble-desktop-full

# 추가 종속성
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-moveit ros-humble-navigation2
sudo apt install ros-humble-slam-toolbox ros-humble-rplidar-ros
```

### 빌드
```bash
cd ~/ros2_ws/src
git clone <repository-url> gf3
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 사용법

### 1. 기본 로봇 구동
```bash
# 로봇 하드웨어 인터페이스 및 컨트롤러 시작
ros2 launch gf3_bringup gf3.launch.py

# 별도 터미널에서 MoveIt 시작
ros2 launch gf3_moveit_config demo.launch.py
```

### 2. 내비게이션 시스템
```bash
# SLAM 및 내비게이션 통합 시작
ros2 launch gf3_bringup mecanum_nav_launch.py

# 초기 포즈 설정 (RViz에서)
# 목표점 설정을 통한 자율 주행
```

### 3. 개별 시스템 테스트

#### 메카넘 제어 테스트
```bash
ros2 run gf3_hardware mecanum_control

# 별도 터미널에서 속도 명령
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"
```

#### 로봇팔 위치 제어
```bash
# Forward position controller 사용
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.3, -0.2, 0.1, 0.0]"
```

### 4. 시리얼 통신 테스트
```bash
# 하드웨어 직접 제어용 시리얼 인터페이스
python3 gf3_serial.py
```

## 토픽 및 서비스

### 주요 토픽
- `/cmd_vel` (geometry_msgs/Twist): 메카넘 속도 명령
- `/odom` (nav_msgs/Odometry): 오도메트리 정보
- `/command` (moteus_msgs/MoteusCommandArray): 로봇팔 명령
- `/state` (moteus_msgs/MoteusStateArray): 로봇팔 상태
- `/scan` (sensor_msgs/LaserScan): LiDAR 데이터
- `/map` (nav_msgs/OccupancyGrid): SLAM 맵

### 컨트롤러
- `joint_state_broadcaster`: 조인트 상태 브로드캐스터
- `forward_position_controller`: 위치 명령 컨트롤러
- `position_trajectory_controller`: 궤적 추종 컨트롤러

## 기술적 특징

### 1. 하이브리드 통신 구조
- **로봇팔**: Topic 기반 Moteus 통신
- **이동 베이스**: CAN 버스 직접 통신
- 각 시스템의 특성에 맞는 최적화된 통신 방식

### 2. 정확한 오도메트리
- 메카넘 휠 특성을 고려한 운동학 모델
- 캘리브레이션 팩터 적용 (선형: 1.12, 각속도: 2.1)
- TF tree를 통한 좌표계 관리

### 3. 로버스트한 제어 시스템
- ros2_control 프레임워크 활용
- 실시간 성능 최적화 (100Hz 업데이트)
- 하드웨어 추상화를 통한 확장성

### 4. 통합 내비게이션
- SLAM과 내비게이션의 동시 실행
- 동적 장애물 회피