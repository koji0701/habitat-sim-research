import os
import cv2
import numpy as np
import habitat_sim
from programatic_controller import ProgrammaticController
import time
from habitat_sim.utils.viz_utils import observation_to_image  # Import the function

def main():
    # Set the path to your data directory (adjust as needed)
    data_path = os.path.join(os.path.expanduser("~/developer/"), "habitat_data")
    
    # Create a directory to store collision images if it doesn't exist
    collision_images_dir = "collision_images"
    os.makedirs(collision_images_dir, exist_ok=True)
    
    if not os.path.exists(data_path):
        print(f"Data directory not found at {data_path}")
        print("You may need to download test scenes using:")
        print("python -m habitat_sim.utils.datasets_download --data-path ~/habitat-sim-data --datasets habitat_test_scenes")
        data_path = "."  # Use current directory as fallback
    
    # Create simulator configuration
    sim_config = habitat_sim.SimulatorConfiguration()
    
    # Try to find a valid scene file
    scene_paths = [
        os.path.join(data_path, "scene_datasets/habitat-test-scenes/skokloster-castle.glb"),
        os.path.join(data_path, "scene_datasets/habitat-test-scenes/van-gogh-room.glb"),
    ]
    
    for scene_path in scene_paths:
        if os.path.exists(scene_path):
            sim_config.scene_id = scene_path
            print(f"Using scene: {scene_path}")
            break
    else:
        print("No valid scene found. Using an empty scene.")
        sim_config.scene_id = "NONE"
    
    # Create agent configuration
    agent_config = habitat_sim.AgentConfiguration()
    
    # Add RGB sensor to the agent
    rgb_sensor_spec = habitat_sim.CameraSensorSpec()
    rgb_sensor_spec.uuid = "rgb"
    rgb_sensor_spec.sensor_type = habitat_sim.SensorType.COLOR
    rgb_sensor_spec.resolution = [480, 640]
    rgb_sensor_spec.position = [0.0, 1.5, 0.0]
    rgb_sensor_spec.sensor_subtype = habitat_sim.SensorSubType.PINHOLE
    agent_config.sensor_specifications = [rgb_sensor_spec]
    
    # Create simulator with the configuration
    cfg = habitat_sim.Configuration(sim_config, [agent_config])
    simulator = habitat_sim.Simulator(cfg)
    
    # Get the agent
    agent = simulator.agents[0]
    
    # Create the programmatic controller
    controller = ProgrammaticController(agent)
    
    # Display initial observation
    observations = simulator.get_sensor_observations()
    cv2.imshow("RGB Observation", observations["rgb"])
    cv2.waitKey(2000)  # Wait 2 seconds
    
    # Execute a sequence of movements with visualization
    # IMPORTANT: THIS CAN BE CHANGED TO FIT A PATH OF MOVEMENT
    sequence = [
        "move_forward", "move_forward", "turn_left", 
        "move_forward", "turn_left", "move_forward",
        "move_forward", "move_forward", "turn_left", 
        "move_forward", "turn_left", "move_forward",
        "move_forward", "move_forward", "turn_left", 
        "move_forward", "turn_left", "move_forward",
        "move_forward", "move_forward", "turn_left", 
        "move_forward", "turn_left", "move_forward",
    ]
    
    print("\nExecuting a sequence of actions...")
    TOTAL_DEMO_MOVEMENT_TIME = 30
    startTime = time.time()

    lastCollisionCount = controller.collision_count

    while time.time() - startTime < TOTAL_DEMO_MOVEMENT_TIME:
        for action in sequence:
            #print(f"Executing: {action}")
            controller.move(action)
            observations = simulator.get_sensor_observations()
            cv2.imshow("RGB Observation", observations["rgb"])

            cv2.waitKey(50)  # Wait 0.05 second between actions

            rgb_image = observation_to_image(observations["rgb"], "color")
            rgb_collision_filename = os.path.join(this_collision_dir, f"collision_rgb_{controller.collision_count}.png")
            rgb_image.save(rgb_collision_filename)
            print(f"RGB Collision image saved to: {rgb_collision_filename}")

            if controller.collision_count > lastCollisionCount:
                print("Collision Occurred! Collision count: ", controller.collision_count)
                
                this_collision_dir = os.path.join(collision_images_dir, f"collision_{controller.collision_count}")
                lastCollisionCount = controller.collision_count

    print("\nMoving forward by 2 meters...")
    controller.move_forward_by(2.0)
    observations = simulator.get_sensor_observations()
    cv2.imshow("RGB Observation", observations["rgb"])
    cv2.waitKey(2000)
    
    print("\nRotating 90 degrees left...")
    controller.rotate_by(90)
    observations = simulator.get_sensor_observations()
    cv2.imshow("RGB Observation", observations["rgb"])
    cv2.waitKey(2000)
    
    print("\nNavigating to a target position...")
    # Get current position and set a target 10 meters ahead and 5 meters to the right
    current_position = agent.get_state().position
    target_position = current_position + np.array([10.0, 0.0, 5.0])
    
    # Navigate to the target position with visualization
    controller.navigate_to(target_position)
    observations = simulator.get_sensor_observations()
    print("Navigated to target position.")
    cv2.imshow("RGB Observation", observations["rgb"])
    cv2.waitKey(2000)
    
    # Check if any collisions occurred
    if controller.collision_occurred:
        print(f"A collision occurred during navigation when executing: {controller.last_collision_action}")
        
        # Save the final collision image if one occurred during navigation
        if controller.collision_count > lastCollisionCount:
            rgb_image = observation_to_image(observations["rgb"], "color")
            collision_filename = os.path.join(collision_images_dir, f"collision_navigation_{controller.collision_count}.png")
            rgb_image.save(collision_filename)
            print(f"Navigation collision image saved to: {collision_filename}")
    
    print("\nDemonstration complete!")
    print(f"Total collisions: {controller.collision_count}")
    print(f"Collision images saved in: {collision_images_dir}")
    cv2.destroyAllWindows()
    simulator.close()

if __name__ == "__main__":
    main()