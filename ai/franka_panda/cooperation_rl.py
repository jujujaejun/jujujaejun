import pybullet as p
import pybullet_data
import time
import numpy as np
from stable_baselines3 import PPO

class FrankaEnv:
    def __init__(self):
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        # 로봇 및 환경 설정
        self.plane_id = p.loadURDF("plane.urdf")
        self.robot1_id = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0])
        self.robot2_id = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 1, 0])
        self.object_id = p.loadURDF("cube_small.urdf", basePosition=[0.5, 0.5, 0.5])

        self.target_position = [0.7, 0.7, 0.5]  # 목표물 위치 설정
        self.done = False

    def reset(self):
        p.resetBasePositionAndOrientation(self.robot1_id, [0, 0, 0], [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(self.robot2_id, [0, 1, 0], [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(self.object_id, [0.5, 0.5, 0.5], [0, 0, 0, 1])
        self.done = False
        return self.get_observation()

    def get_observation(self):
        # 로봇의 상태 및 물체의 위치를 관측
        robot1_state = p.getLinkState(self.robot1_id, 11)  # 말단 링크의 상태
        robot2_state = p.getLinkState(self.robot2_id, 11)
        object_pos, _ = p.getBasePositionAndOrientation(self.object_id)

        return np.array([robot1_state[0], robot2_state[0], object_pos])

    def step(self, action1, action2):
        # 로봇 1과 로봇 2에 대해 주어진 행동을 실행
        p.setJointMotorControlArray(self.robot1_id, range(7), p.POSITION_CONTROL, action1)
        p.setJointMotorControlArray(self.robot2_id, range(7), p.POSITION_CONTROL, action2)
        p.stepSimulation()

        # 새로운 상태 반환
        obs = self.get_observation()
        reward = -np.linalg.norm(np.array(self.target_position) - np.array(obs[2]))  # 목표와의 거리
        done = np.linalg.norm(np.array(self.target_position) - np.array(obs[2])) < 0.05
        return obs, reward, done, {}

    def render(self):
        p.stepSimulation()

env = FrankaEnv()

# 강화학습 모델 (PPO 알고리즘)
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=10000)

# 학습된 정책 사용하여 시뮬레이션 실행
obs = env.reset()
while not env.done:
    action1, _ = model.predict(obs)
    action2, _ = model.predict(obs)
    obs, reward, done, _ = env.step(action1, action2)
    env.render()
    time.sleep(1.0 / 240.0)
