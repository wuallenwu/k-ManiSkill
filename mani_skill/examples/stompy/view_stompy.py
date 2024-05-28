import mani_skill.envs
import stompy
import gymnasium as gym
from dm_control import viewer

env = gym.make("EmptyEnv-v1", robot_uids="stompy")
def random_policy(_):
   return env.action_space.sample()
viewer.launch(env.unwrapped.mj_env, policy=random_policy)
