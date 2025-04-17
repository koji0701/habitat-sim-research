#!/usr/bin/env python3

# Copyright (c) Meta Platforms, Inc. and its affiliates.
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.


import argparse
import os
import csv
import datetime

import demo_runner as dr

parser = argparse.ArgumentParser()
parser.add_argument("--scene", type=str, default=dr.default_sim_settings["scene"])
parser.add_argument("--width", type=int, default=640)
parser.add_argument("--height", type=int, default=480)
parser.add_argument("--max_frames", type=int, default=1000)
parser.add_argument("--save_png", action="store_true")
parser.add_argument("--sensor_height", type=float, default=1.5)
parser.add_argument("--disable_color_sensor", action="store_true")
parser.add_argument("--semantic_sensor", action="store_true")
parser.add_argument("--depth_sensor", action="store_true")
parser.add_argument("--print_semantic_scene", action="store_true")
parser.add_argument("--print_semantic_mask_stats", action="store_true")
parser.add_argument("--compute_shortest_path", action="store_true")
parser.add_argument("--compute_action_shortest_path", action="store_true")
parser.add_argument("--recompute_navmesh", action="store_true")
parser.add_argument("--seed", type=int, default=1)
parser.add_argument("--silent", action="store_true")
parser.add_argument("--test_fps_regression", type=int, default=0)
parser.add_argument("--enable_physics", action="store_true")
parser.add_argument(
    "--physics_config_file",
    type=str,
    default=dr.default_sim_settings["physics_config_file"],
)
parser.add_argument("--disable_frustum_culling", action="store_true")
args = parser.parse_args()


def make_settings():
    settings = dr.default_sim_settings.copy()
    settings["max_frames"] = args.max_frames
    settings["width"] = args.width
    settings["height"] = args.height
    settings["scene"] = args.scene
    settings["save_png"] = args.save_png
    settings["sensor_height"] = args.sensor_height
    settings["color_sensor"] = not args.disable_color_sensor
    settings["semantic_sensor"] = args.semantic_sensor
    settings["depth_sensor"] = args.depth_sensor
    settings["print_semantic_scene"] = args.print_semantic_scene
    settings["print_semantic_mask_stats"] = args.print_semantic_mask_stats
    settings["compute_shortest_path"] = args.compute_shortest_path
    settings["compute_action_shortest_path"] = args.compute_action_shortest_path
    settings["seed"] = args.seed
    settings["silent"] = args.silent
    settings["enable_physics"] = args.enable_physics
    settings["physics_config_file"] = args.physics_config_file
    settings["frustum_culling"] = not args.disable_frustum_culling
    settings["recompute_navmesh"] = args.recompute_navmesh

    return settings
def save_performance_data(output_dir, perfs, settings):
    """Save performance data to a CSV file."""
    # Create output directory if it doesn't exist yet
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    csv_path = os.path.join(output_dir, "performance_data.csv")
    
    # Calculate summary statistics
    avg_fps = 0
    avg_frame_time = 0
    avg_step_time = 0
    total_steps = 0
    
    if len(perfs) > 0:
        for perf in perfs:
            avg_fps += perf["fps"]
            avg_frame_time += perf["frame_time"]
            for step_time in perf["time_per_step"]:
                avg_step_time += step_time
                total_steps += 1
        
        avg_fps /= len(perfs)
        avg_frame_time /= len(perfs)
        avg_step_time /= total_steps if total_steps > 0 else 1
    
    # Write to CSV
    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Write simulation settings
        writer.writerow(["Simulation Settings"])
        for key, value in settings.items():
            if key != "output_dir":  # Skip output_dir to avoid circular reference
                writer.writerow([key, str(value)])
        
        writer.writerow([])  # Empty row
        
        # Write performance summary
        writer.writerow(["Performance Summary"])
        writer.writerow(["Resolution", f"{settings['width']} x {settings['height']}"])
        writer.writerow(["Average FPS", f"{avg_fps:.2f}"])
        writer.writerow(["Average Frame Time (ms)", f"{avg_frame_time * 1000.0:.3f}"])
        writer.writerow(["Average Step Time (ms)", f"{avg_step_time * 1000.0:.3f}"])
        
        writer.writerow([])  # Empty row
        
        # Write detailed performance data for each run
        writer.writerow(["Run Details"])
        writer.writerow(["Run", "Total Time (s)", "Frame Time (ms)", "FPS", "Avg Sim Step Time (ms)"])
        
        for i, perf in enumerate(perfs):
            writer.writerow([
                i + 1,
                f"{perf['total_time']:.2f}",
                f"{perf['frame_time'] * 1000.0:.3f}",
                f"{perf['fps']:.1f}",
                f"{perf.get('avg_sim_step_time', 0) * 1000.0:.3f}"
            ])
        
        writer.writerow([])  # Empty row
        
        # Write detailed frame data with position, rotation, action and collision information
        writer.writerow(["Frame Details"])
        writer.writerow(["Run", "Frame", "Action", "Position", "Rotation", "Step Time (ms)", "Collision", "Saved Files"])
        
        for i, perf in enumerate(perfs):
            if "frame_data" in perf:
                for frame_info in perf["frame_data"]:
                    saved_files_str = ", ".join(frame_info["saved_files"]) if frame_info["saved_files"] else "None"
                    writer.writerow([
                        i + 1,
                        frame_info["frame"],
                        frame_info["action"],
                        str(frame_info["position"]),
                        frame_info["rotation"],
                        f"{frame_info['step_time'] * 1000.0:.3f}",
                        "Yes" if frame_info["collision"] else "No",  # Add collision information
                        saved_files_str
                    ])
        
        writer.writerow([])  # Empty row
    
    # Also save a separate CSV with just the frame data for easier processing
    frame_csv_path = os.path.join(output_dir, "frame_data.csv")
    with open(frame_csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Run", "Frame", "Action", "Position X", "Position Y", "Position Z", "Rotation", "Step Time (ms)", "Collision", "Saved Files"])
        
        for i, perf in enumerate(perfs):
            if "frame_data" in perf:
                for frame_info in perf["frame_data"]:
                    saved_files_str = ", ".join(frame_info["saved_files"]) if frame_info["saved_files"] else "None"
                    position = frame_info["position"]
                    writer.writerow([
                        i + 1,
                        frame_info["frame"],
                        frame_info["action"],
                        position[0],
                        position[1],
                        position[2],
                        frame_info["rotation"],
                        f"{frame_info['step_time'] * 1000.0:.3f}",
                        "Yes" if frame_info["collision"] else "No",  # Add collision information
                        saved_files_str
                    ])
    
    return csv_path

settings = make_settings()

perfs = []
for _i in range(1):
    demo_runner = dr.DemoRunner(settings, dr.DemoRunnerType.EXAMPLE)
    perf = demo_runner.example()
    perfs.append(perf)

    print(" ========================= Performance ======================== ")
    print(
        " %d x %d, total time %0.2f s,"
        % (settings["width"], settings["height"], perf["total_time"]),
        "frame time %0.3f ms (%0.1f FPS)" % (perf["frame_time"] * 1000.0, perf["fps"]),
    )
    print(" ============================================================== ")

    # assert perf["fps"] > args.test_fps_regression, (
    #    "FPS is below regression threshold: %0.1f < %0.1f"
    #    % (perf["fps"], args.test_fps_regression)
    # )
# Replace the final part of example.py with this code
if len(perfs) > 0:
    # If output_dir is not set (save_png was False), create one
    if "output_dir" not in settings or not settings["output_dir"]:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        output_dir = f"habitat_output_{timestamp}"
        settings["output_dir"] = output_dir
    else:
        output_dir = settings["output_dir"]
    
    csv_path = save_performance_data(output_dir, perfs, settings)
    
    print(" ========================= Performance ======================== ")
    print(
        " %d x %d, total time %0.2f s,"
        % (settings["width"], settings["height"], perfs[0]["total_time"]),
        "frame time %0.3f ms (%0.1f FPS)" % (perfs[0]["frame_time"] * 1000.0, perfs[0]["fps"]),
    )
    print(" Data saved to:", csv_path)
    print(" ============================================================== ")
    
    if len(perfs) > 1:
        avg_fps = 0
        avg_frame_time = 0
        avg_step_time = 0
        for perf in perfs:
            avg_fps += perf["fps"]
            avg_frame_time += perf["frame_time"]
            for step_time in perf["time_per_step"]:
                avg_step_time += step_time
        avg_fps /= len(perfs)
        avg_frame_time /= len(perfs)
        avg_step_time /= len(perfs) * len(perfs[0]["time_per_step"])
        print("Average FPS:", avg_fps)
        print("Average frame time:", avg_frame_time)
        print("Average step time:", avg_step_time)