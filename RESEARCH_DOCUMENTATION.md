# Habitat-Sim Extensions Documentation for Research

## Table of Contents
- [ProgrammaticController](#programmaticcontroller)
- [Programmatic Movement Demo](#programmatic-movement-demo)

## ProgrammaticController

**Location:** `habitat-sim/src_python/habitat_sim/agent/controls/programatic_controller.py`

This module introduces the `ProgrammaticController` class which provides automated agent movement capabilities. The controller can be reused in any project requiring automated navigation within Habitat environments.

## Programmatic Movement Demo

**Location:** `habitat-sim/src_python/habitat_sim/agent/controls/programatic_movement_demo.py`

A demonstration script showcasing the `ProgrammaticController` class functionality with a pre-defined path (including both movements and rotations).

### Running the Demo

To execute the demonstration:

```bash
cd habitat-sim/src_python/habitat_sim/agent/controls
python programatic_movement_demo.py
