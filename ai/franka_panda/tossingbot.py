import gym
import pybullet as p
import pybullet_data
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.envs import DummyVecEnv

# Define the custom environment for the robot arm to pick and throw objects
class TossingEnv(gym.Env):
    def __init__(self):
        super(TossingEnv, self).__init__()
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot = None
        self.object = None
        self.target = None
        self.reset()

    def reset(self):
        p.resetSimulation(self.physics_client)
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("plane.urdf")
        # Load a simple robot arm and an object to throw
        self.robot_id = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0], useFixedBase=True)
        self.object_id = p.loadURDF("cube_small.urdf", basePosition=[0.5, 0, 0.1])
        self.target_id = p.loadURDF("tray/traybox.urdf", basePosition=[1.0, 0, 0.5])
        self.done = False
        return self._get_observation()

    def _get_observation(self):
        # Return robot and object positions
        robot_pos, _ = p.getBasePositionAndOrientation(self.robot_id)
        object_pos, _ = p.getBasePositionAndOrientation(self.object_id)
        return np.array(robot_pos + object_pos)

    def step(self, action):
        # Apply action to the robot arm
        p.applyExternalForce(self.object_id, -1, action, [0, 0, 0], p.WORLD_FRAME)
        p.stepSimulation()

        # Get new object and robot positions
        object_pos, _ = p.getBasePositionAndOrientation(self.object_id)
        target_pos, _ = p.getBasePositionAndOrientation(self.target_id)

        # Calculate distance to target
        distance = np.linalg.norm(np.array(object_pos) - np.array(target_pos))

        # Reward based on how close the object is to the target
        reward = -distance

        # If the object is close enough to the target, the task is done
        if distance < 0.1:
            self.done = True
            reward += 100

        return self._get_observation(), reward, self.done, {}

    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect()

# Instantiate the environment
env = DummyVecEnv([lambda: TossingEnv()])

# Define the PPO model (Proximal Policy Optimization)
model = PPO("MlpPolicy", env, verbose=1)

# Train the model
model.learn(total_timesteps=10000)

# Save the trained model
model.save("tossingbot_model")

# Test the trained model
obs = env.reset()
for i in range(1000):
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()
