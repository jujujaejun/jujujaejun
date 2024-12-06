import gym
import pybullet as p
import pybullet_data
from stable_baselines3 import PPO
from stable_baselines3.common.envs import DummyVecEnv

# Create a custom environment for the robot to play table tennis
class TableTennisEnv(gym.Env):
    def __init__(self):
        super(TableTennisEnv, self).__init__()
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot = None
        self.target = None
        self.reset()

    def reset(self):
        p.resetSimulation(self.physics_client)
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF("franka_panda/panda.urdf", basePosition=[0, 0, 0.5], useFixedBase=True)
        self.target_id = p.loadURDF("sphere_small.urdf", basePosition=[1.0, 0.0, 0.5])
        self.done = False
        return self._get_observation()

    def _get_observation(self):
        # Return robot state or sensor information
        return p.getLinkState(self.robot_id, 0)[0]

    def step(self, action):
        p.applyExternalForce(self.robot_id, -1, action, [0, 0, 0], p.WORLD_FRAME)
        p.stepSimulation()
        robot_pos, robot_orient = p.getBasePositionAndOrientation(self.robot_id)
        target_pos, _ = p.getBasePositionAndOrientation(self.target_id)

        # Calculate distance to the target (like a table tennis ball)
        distance = ((robot_pos[0] - target_pos[0]) ** 2 + (robot_pos[1] - target_pos[1]) ** 2) ** 0.5

        # Define a reward function that incentivizes minimizing the distance
        reward = -distance

        # Check if the robot hit the target (table tennis ball)
        if distance < 0.1:
            self.done = True
            reward += 100  # Bonus reward for hitting the target

        return self._get_observation(), reward, self.done, {}

    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect()

# Instantiate the environment
env = DummyVecEnv([lambda: TableTennisEnv()])

# Define the PPO model (Proximal Policy Optimization)
model = PPO("MlpPolicy", env, verbose=1)

# Train the model
model.learn(total_timesteps=10000)

# Save the trained model
model.save("ppo_table_tennis")

# Load the model (for inference or further training)
model = PPO.load("ppo_table_tennis")

# Test the trained model
obs = env.reset()
for i in range(1000):
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    env.render()
