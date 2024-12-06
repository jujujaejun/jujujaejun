import gym
import pybullet as p
import pybullet_data
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.envs import DummyVecEnv

class FrankaEnv(gym.Env):
    def __init__(self):
        super(FrankaEnv, self).__init__()
        
        # 로봇 시뮬레이션 초기화
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # 평면 로드
        self.plane = p.loadURDF("plane.urdf")
        
        # 프랑카 로봇 로드
        self.robot_arm = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0], useFixedBase=True)

        # 행동 공간과 관측 공간 설정
        self.action_space = gym.spaces.Box(low=-1, high=1, shape=(7,), dtype=np.float32)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(24,), dtype=np.float32)

    def reset(self):
        # 로봇 초기화 및 시뮬레이션 환경 재설정
        p.resetSimulation()
        p.loadURDF("plane.urdf")
        self.robot_arm = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0], useFixedBase=True)
        p.setGravity(0, 0, -9.81)

        # 초기 상태 반환 (관절 위치 등)
        obs = self._get_observation()
        return obs

    def _get_observation(self):
        # 로봇의 현재 상태 (관절 각도, 속도 등) 관측
        joint_states = p.getJointStates(self.robot_arm, range(7))
        joint_positions = np.array([state[0] for state in joint_states])
        joint_velocities = np.array([state[1] for state in joint_states])
        obs = np.concatenate([joint_positions, joint_velocities])
        return obs

    def step(self, action):
        # 주어진 행동(action)을 로봇에 적용
        for i in range(7):
            p.setJointMotorControl2(self.robot_arm, i, p.POSITION_CONTROL, targetPosition=action[i])

        # 시뮬레이션 한 스텝 진행
        p.stepSimulation()

        # 다음 상태 관측
        obs = self._get_observation()

        # 목표 위치에 가까워지는 것이 보상
        target_position = np.array([0.5, 0, 0.5])  # 목표 위치 (앞쪽으로 이동)
        end_effector_state = p.getLinkState(self.robot_arm, 11)[0]  # 엔드 이펙터 위치
        distance_to_target = np.linalg.norm(np.array(end_effector_state) - target_position)
        reward = -distance_to_target  # 거리가 줄어들수록 보상이 커짐

        done = False
        if distance_to_target < 0.05:  # 목표 위치에 도달한 경우 종료
            done = True

        return obs, reward, done, {}

    def render(self, mode="human"):
        pass

    def close(self):
        p.disconnect()

# 환경 초기화 및 학습
env = DummyVecEnv([lambda: FrankaEnv()])

# PPO 모델 초기화 및 학습
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=10000)

# 학습된 모델 저장
model.save("franka_forward_model")
