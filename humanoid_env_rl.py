import gymnasium as gym
from gymnasium import spaces
import numpy as np
import pygame
from simulation import Simulation

class HumanoidEnv(gym.Env):
    def __init__(self):
        super(HumanoidEnv, self).__init__()

        # Initialize the simulation
        self.simulation = Simulation()
        self.simulation_clock = pygame.time.Clock()

        # Define the action space: motor speeds for 4 joints
        self.action_space = spaces.Box(
            low=np.array([-10, -10, -10, -10]),  # Min motor speeds
            high=np.array([10, 10, 10, 10]),    # Max motor speeds
            dtype=np.float32
        )

        # Define the observation space based on the humanoid's log_state
        self.observation_space = spaces.Box(
            low=-np.inf,  # Allow for unlimited observation values (customize if needed)
            high=np.inf,
            shape=(len(self.simulation.humanoid.log_state()),),
            dtype=np.float32
        )
        
        # Timer for standing phase
        self.standing_time = 2.0  # Time to stand still in seconds
        self.start_time = None

    def reset(self, seed=None, options=None):
        """
        Reset the environment to its initial state and return the initial observation.
        """
        del self.simulation  # Clear the previous simulation
        self.simulation = Simulation()  # Restart the simulation
        self.start_time = pygame.time.get_ticks()  # Start the timer for standing phase
        return self._get_observation(), {}

    def step(self, action):
        """
        Perform one step in the environment with the given action.

        Args:
            action (np.array): Motor speeds for the joints.

        Returns:
            observation (np.array): The new observation.
            reward (float): The computed reward.
            done (bool): Whether the episode is finished.
            info (dict): Additional debug information.
        """
        current_time = pygame.time.get_ticks()
        
        # If the humanoid is still in the standing phase (check time elapsed)
        if self.start_time is not None and (current_time - self.start_time) < self.standing_time * 1000:
            # Set motor speeds for standing still (zero motor speeds or small values)
            standing_speeds = np.array([0.0, 0.0, 0.0, 0.0])
            self.simulation.humanoid.update_motors(standing_speeds)
        else:
            # Once standing time is over, transition to walking
            self.simulation.humanoid.update_motors(action)

        # Step the simulation forward
        self.simulation.world.Step(1.0 / 60.0, 6, 2)

        # Get the new observation
        observation = self._get_observation()

        # Compute the reward
        reward = self._compute_reward()

        # Check if the episode is done
        done = self._is_done()

        # Additional info (can include debug data if needed)
        info = {}

        return observation, float(reward), done, False, info

    def render(self, mode='human'):
        """
        Render the environment using the simulation's rendering system.
        """
        self.simulation.screen.fill(self.simulation.bg_color)
        self.simulation.render_ground()
        self.simulation.render_flag()
        self.simulation.humanoid.render(self.simulation.screen, self.simulation.ppm)
        pygame.display.flip()
        self.simulation_clock.tick(60)

    def close(self):
        """
        Close the environment.
        """
        pygame.quit()

    def _get_observation(self):
        """
        Get the current observation from the humanoid's state.

        Returns:
            np.array: The state as a flattened array.
        """
        state = self.simulation.humanoid.log_state()
        return np.array(list(state.values()), dtype=np.float32)

    def _compute_reward(self):
        """
        Compute the reward based on the current state.

        Returns:
            float: The reward value.
        """
        state = self.simulation.humanoid.log_state()

        # Progress reward: Encourage forward movement along the x-axis
        torso_x = state['torso_x']
        torso_vx = state['torso_vx']  # Forward velocity
        progress_reward = torso_vx  # Reward for moving forward

        # Stability reward: Encourage the torso to remain upright
        torso_y = state['torso_y']
        torso_upright = max(0, 1 - abs(torso_y - 1.0))  # Reward decreases as the torso deviates from y = 1.0

        # Penalty for falling: Large penalty if the torso y falls below a threshold
        fall_penalty = -10 if torso_y < 0.5 else 0

        # Combine the rewards and penalties
        reward = progress_reward + torso_upright + fall_penalty
        return float(reward)
            

    def _is_done(self):
        """
        Determine if the episode is done.

        Returns:
            bool: True if the episode is finished, otherwise False.
        """
        state = self.simulation.humanoid.log_state()
        x = state.get('torso_x', 0)

        # End the episode if the humanoid falls
        if x > 16:
            return True

        return False