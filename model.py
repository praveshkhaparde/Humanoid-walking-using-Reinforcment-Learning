import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.monitor import Monitor
from humanoid_env_rl import HumanoidEnv
import sys
import torch

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("device name: ", torch.cuda.get_device_name())

# Create the humanoid environment
env = HumanoidEnv()

# Check the environment for compatibility with OpenAI Gym standards
check_env(env, warn=True)

# Wrap the environment with a Monitor to log episode rewards and lengths
env = Monitor(env)

# Define the RL model
model = PPO(
    policy="MlpPolicy",  # Multilayer perceptron policy
    env=env,              # Pass the custom environment
    verbose=1,            # Print training information
    tensorboard_log="./humanoid_rl_tensorboard/",  # Log directory for TensorBoard
    learning_rate=3e-3,   # Learning rate for optimization
    gamma=0.99,           # Discount factor
    n_steps=2048,         # Number of steps to run per rollout
    batch_size=64,        # Minibatch size for training
    n_epochs=10,          # Number of optimization epochs per update
    device = device
)

# # Check if the argument is passed, otherwise train a new model
if len(sys.argv) > 1 and sys.argv[1] == "load":
    model = PPO.load("humanoid_ppo_model", env=env)
else:
    TIMESTEPS = 1000  # Set the number of timesteps for training
    model.learn(total_timesteps=TIMESTEPS)
    model.save("humanoid_ppo_model")

model = PPO.load("humanoid_ppo_model", env=env)


# Test the trained model
obs = env.reset()[0]
done = False
while not done:
    action, _states = model.predict(obs, deterministic=True)  # Get action from the model
    print(env.step(action))
    obs, reward, done, _, info = env.step(action)  # Perform the action in the environment
    env.render()  # Render the environment (visualization)

# Close the environment
env.close()
