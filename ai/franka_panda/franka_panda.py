import pybullet as p
import pybullet_data
import time
import math

# PyBullet 시뮬레이션 환경 설정
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 중력 설정
p.setGravity(0, 0, -9.81)

# 모델 로드
plane_id = p.loadURDF("plane.urdf")
franka_panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
cube_id = p.loadURDF("cube_small.urdf", [0.8, 0, 1], globalScaling=1.0)  # 큐브의 크기를 1로 설정

# 그리퍼 상태 (열림/닫힘)
gripper_open = True  # 처음에는 그리퍼가 열려 있는 상태

# 로봇 암의 끝 위치를 저장하는 변수
end_effector_position = [0, 0, 0]  # 엔드 이펙터 초기 위치

# # 큐브의 위치를 저장하는 변수
# cube_position = [0.8, 0, 1]  # 초기 위치

# # 큐브의 방향 (회전은 없음, [x, y, z, w]로 표현된 쿼터니언)
# cube_orientation = [0, 0, 0, 1]

# 그리퍼 열림/닫힘 동작을 위한 함수
def control_gripper(open_gripper):
    gripper_position = 0.04 if open_gripper else 0.0  # 열릴 때 0.04, 닫힐 때 0.0
    p.setJointMotorControl2(franka_panda, 9, p.POSITION_CONTROL, targetPosition=gripper_position, force=100)
    p.setJointMotorControl2(franka_panda, 10, p.POSITION_CONTROL, targetPosition=gripper_position, force=100)

# 유클리드 거리를 계산하는 함수
def calculate_distance(pos1, pos2):
    val = math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2 + (pos1[2] - pos2[2]) ** 2)
    print(val)
    return val

# 시뮬레이션 루프
while True:
    # # 큐브의 위치를 업데이트 (변수를 사용하여 위치를 변경)
    # p.resetBasePositionAndOrientation(cube_id, cube_position, cube_orientation)
    # # 큐브 위치를 수동으로 변경 (예: X축으로 이동)
    # cube_position[0] += 0.001  # X축으로 조금씩 이동

    # 큐브의 위치를 가져오기
    cube_position, cube_orientation = p.getBasePositionAndOrientation(cube_id)

    # 로봇 팔의 목표 위치를 큐브의 위치로 설정
    target_position = list(cube_position)

    # 역기구학 계산 (target_position에 맞는 관절 각도 계산)
    joint_angles = p.calculateInverseKinematics(franka_panda, 11, target_position)

    # 계산된 각도를 로봇에 적용
    for i in range(len(joint_angles)):
        p.setJointMotorControl2(franka_panda, i, p.POSITION_CONTROL, joint_angles[i])

    # 로봇 팔의 끝 위치(엔드 이펙터) 가져오기
    link_state = p.getLinkState(franka_panda, 11)  # 11번 링크는 엔드 이펙터
    end_effector_position = list(link_state[4])  # 엔드 이펙터의 현재 위치 저장

    # 두 위치(큐브와 목표 위치) 간의 유클리드 거리 계산
    distance = calculate_distance(cube_position, end_effector_position)

    # 임계값 0.001m 이하로 수렴했는지 확인하고 그리퍼를 제어
    if distance < 0.02:
        gripper_open = False  # 그리퍼 닫음
    else:
        gripper_open = True  # 큐브가 범위를 벗어나면 그리퍼 열림

    # 그리퍼 제어 적용
    control_gripper(gripper_open)

    # PyBullet 시뮬레이션을 한 프레임 실행
    p.stepSimulation()

    # 프레임 속도 조절 (1/240초 대기)
    time.sleep(1./240)
