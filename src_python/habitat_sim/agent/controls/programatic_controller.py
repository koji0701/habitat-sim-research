#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import os
import numpy as np
import habitat_sim
from habitat_sim.agent.controls.programmatic_controller import ProgrammaticController

def main():
    # Set up a basic simulator configuration
    sim_settings = {
        "scene": "data/scene_datasets/habitat-test-scenes/skokloster-castle.glb",
        "default_agent": 0,
        "sensor_height": 1.5,
        "width": 512,
        "height": 512,
        "sensor_pitch": 0,
    }
    
    # Create a simulator configuration
    cfg = make_simple_cfg(sim_settings)
    
    # Initialize the simulator
    sim = habitat_sim.Simulator(cfg)
    
    # Get the default agent
    agent = sim.agents[0]
    
    # Create our programmatic controller for the agent
    controller = ProgrammaticController(agent)
    
    # Execute a predefined sequence of actions
    print("Executing a simple action sequence")
    sequence = ["move_forward", "turn_left", "move_forward", "turn_right", "move_forward"]
    controller.execute_sequence(sequence)
    
    # Move forward by a specific amount
    print("Moving forward by 2 meters")
    controller.move_forward_by(2.0)
    
    # Rotate by a specific angle
    print("Rotating 90 degrees left")
    controller.rotate_by(90)
    
    # Navigate to a target position
    print("Navigating to target position")
    current_position = agent.get_state().position
    target_position = current_position + np.array([3.0, 0.0, 0.0])
    controller.navigate_to(target_position)
    
    # Check if collision occurred during the sequence
    if controller.collision_occurred:
        print(f"A collision occurred during the navigation when executing: {controller.last_collision_action}")
    
    # Close the simulator
    sim.close()
    
    print("Programmatic control example completed successfully!")

def make_simple_cfg(settings):
    """Create a simple simulator configuration based on settings."""
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = settings["scene"]
    
    # Set up the agent configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    
    # Create a RGB sensor
    sensors = {
        "rgb": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": [settings["height"], settings["width"]],
            "position": [0.0, settings["sensor_height"], 0.0],
        }
    }
    
    # Create sensor specifications
    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        sensor_spec = habitat_sim.SensorSpec()
        sensor_spec.uuid = sensor_uuid
        sensor_spec.sensor_type = sensor_params["sensor_type"]
        sensor_spec.resolution = sensor_params["resolution"]
        sensor_spec.position = sensor_params["position"]
        sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
        sensor_specs.append(sensor_spec)
    
    agent_cfg.sensor_specifications = sensor_specs
    return habitat_sim.Configuration(sim_cfg, [agent_cfg])

if __name__ == "__main__":
    main()