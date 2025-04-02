#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import habitat_sim
from habitat_sim.agent.controls.controls import ActuationSpec
from habitat_sim.agent.agent import Agent

class ProgrammaticController:
    """Controller that provides programmatic movement for an agent.
    Allows execution of action sequences, movement by specific amounts,
    and navigation to target positions.
    """
    
    def __init__(self, agent: Agent):
        """Initialize the controller with an agent.
        
        Args:
            agent: The agent to control
        """
        self.agent = agent
        self.collision_occurred = False
        self.last_collision_action = None
        
    def move(self, action_name: str) -> bool:
        """Execute a single action by name.
        
        Args:
            action_name: Name of the action to execute
            
        Returns:
            Whether a collision occurred during the action
        """
        did_collide = self.agent.act(action_name)
        if did_collide:
            self.collision_occurred = True
            self.last_collision_action = action_name
        return did_collide
        
    def execute_sequence(self, action_sequence: list) -> bool:
        """Execute a sequence of actions.
        
        Args:
            action_sequence: List of action names to execute in order
            
        Returns:
            Whether any collision occurred during the sequence
        """
        any_collision = False
        for action in action_sequence:
            did_collide = self.move(action)
            any_collision = any_collision or did_collide
        return any_collision
    
    def move_forward_by(self, distance: float) -> bool:
        """Move forward by a specific distance in meters.
        
        Args:
            distance: Distance to move in meters
            
        Returns:
            Whether any collision occurred
        """
        # Get the move_forward action spec
        action_spec = self.agent.agent_config.action_space["move_forward"].actuation
        
        # Calculate how many times to execute the action
        num_actions = int(distance / action_spec.amount)
        remainder = distance % action_spec.amount
        
        # Execute the action multiple times
        collision = False
        for _ in range(num_actions):
            did_collide = self.move("move_forward")
            collision = collision or did_collide
            
        # Handle remainder
        if remainder > 0:
            # Store original amount
            original_amount = action_spec.amount
            # Modify action specification temporarily
            action_spec.amount = remainder
            did_collide = self.move("move_forward")
            collision = collision or did_collide
            # Restore original amount
            action_spec.amount = original_amount
            
        return collision
    
    def rotate_by(self, angle: float) -> bool:
        """Rotate by a specific angle in degrees.
        
        Args:
            angle: Angle to rotate in degrees, positive is left, negative is right
            
        Returns:
            Whether any collision occurred
        """
        collision = False
        
        if angle > 0:
            # Turn left
            action_spec = self.agent.agent_config.action_space["turn_left"].actuation
            num_actions = int(angle / action_spec.amount)
            remainder = angle % action_spec.amount
            
            for _ in range(num_actions):
                did_collide = self.move("turn_left")
                collision = collision or did_collide
                
            if remainder > 0:
                original_amount = action_spec.amount
                action_spec.amount = remainder
                did_collide = self.move("turn_left")
                collision = collision or did_collide
                action_spec.amount = original_amount
                
        elif angle < 0:
            # Turn right
            action_spec = self.agent.agent_config.action_space["turn_right"].actuation
            num_actions = int(abs(angle) / action_spec.amount)
            remainder = abs(angle) % action_spec.amount
            
            for _ in range(num_actions):
                did_collide = self.move("turn_right")
                collision = collision or did_collide
                
            if remainder > 0:
                original_amount = action_spec.amount
                action_spec.amount = remainder
                did_collide = self.move("turn_right")
                collision = collision or did_collide
                action_spec.amount = original_amount
                
        return collision
    
    def navigate_to(self, target_position: np.ndarray, 
                   position_threshold: float = 0.2, 
                   max_iterations: int = 100) -> bool:
        """Navigate to a target position using a simple approach.
        
        Args:
            target_position: Target position as numpy array [x, y, z]
            position_threshold: How close agent needs to be to target
            max_iterations: Maximum number of steps to try
            
        Returns:
            Whether any collision occurred during navigation
        """
        collision = False
        iterations = 0
        
        while iterations < max_iterations:
            # Get current state
            current_state = self.agent.get_state()
            current_position = current_state.position
            
            # Check if we're close enough to target
            distance = np.linalg.norm(current_position - target_position)
            if distance <= position_threshold:
                break
                
            # Calculate direction to target
            direction = target_position - current_position
            direction[1] = 0  # Ignore vertical component
            direction = direction / np.linalg.norm(direction)
            
            # Get agent's forward direction
            forward = np.array([0, 0, -1])  # Negative Z is forward
            forward = habitat_sim.utils.quat_rotate_vector(
                current_state.rotation, forward
            )
            forward[1] = 0  # Ignore vertical component
            if np.linalg.norm(forward) > 0:
                forward = forward / np.linalg.norm(forward)
            
            # Calculate angle between forward and target direction
            dot_product = np.dot(forward, direction)
            dot_product = np.clip(dot_product, -1.0, 1.0)
            angle = np.arccos(dot_product) * 180 / np.pi
            
            # Determine turn direction
            cross_product = np.cross(forward, direction)
            if cross_product[1] < 0:
                angle = -angle
                
            # Turn towards target
            if abs(angle) > 5:
                did_collide = self.rotate_by(angle)
                collision = collision or did_collide
            else:
                # Move forward
                did_collide = self.move("move_forward")
                collision = collision or did_collide
                
            iterations += 1
            
        return collision