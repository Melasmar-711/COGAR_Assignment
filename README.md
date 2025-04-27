# Cooking Manager System

## Overview

This project implements a **cooking assistant system** composed of several ROS-based modules.  
The system allows a robot to track a recipe, monitor human verbal commands, plan actions dynamically, and control execution through different stages of a cooking task.  
It ensures seamless interaction between planning, perception, and control to accomplish cooking procedures safely and efficiently.

---

## Mandatory Components

### 📋 Recipe Tracking and Execution History
- Maintains an internal, updated representation of the recipe.
- Tracks the sequence of performed actions.
- Ensures correct step-by-step execution of the cooking process.
- Provides data to action planning modules to determine the next best action.

### 🤖 Action Planning Based on Cooking State and Task History
- Decides the next action based on current progress and history.
- Evaluates system state to ensure seamless task progression.
- Dynamically adjusts the planned sequence in response to unexpected conditions (e.g., errors, failures).

### 🗣️ Human Command Monitoring and Conflict Resolution
- Recognizes and interprets verbal human commands related to tasks (like adding ingredients or passing objects).
- Validates the command:
  - If valid, updates the task plan.
  - If invalid, rejects the command and sticks to the original plan.
  
---

## System Architecture

The system is built as **three ROS packages**:

### 1. Cooking Manager
Handles recipe management, action planning, and human interaction.

| Node                  | Description |
|-----------------------|-------------|
| `action_planner`       | Plans and schedules robot actions based on recipe and system state. |
| `human_command_manager`| Listens to verbal commands and modifies plans based on validity. |
| `recipe_tracker`       | Tracks recipe progress and history of completed steps. |

**Services:**
- `SendActionSeq.srv` - Service to send planned action sequences.
- `UpdateRecipe.srv` - Service to update the recipe steps.

**Messages:**
- `RecipeStep.msg` - Represents a step in the recipe.

---

### 2. Controller
Handles low-level control and system status management.

| Node                | Description |
|---------------------|-------------|
| `action_parser`      | Parses high-level actions into executable motions. |
| `PID_Controller`     | Controls the robot actuators using PID algorithms. |
| `trajectory_manager` | Manages robot trajectories to accomplish tasks safely. |

**Messages:**
- `SystemState.msg` - Broadcasts the current system status (e.g., IDLE, EXECUTING, FAILURE).

---

### 3. Perception
Handles the robot’s sensory inputs and environment mapping.

| Node               | Description |
|--------------------|-------------|
| `computer_vision`   | Detects objects in the scene using visual sensors. |
| `sensor_fusion`     | Combines multiple sensor data for better object detection and localization. |
| `slam`              | Provides environment mapping and localization. |

**Messages:**
- `DetectedObjects.msg` - Publishes detected objects information.

---

## Communication Between Nodes

- `recipe_tracker` publishes `RecipeStep` messages indicating the next recipe step.
- `action_planner` listens to `RecipeStep` and plans appropriate actions.
- `human_command_manager` subscribes to verbal command topics and checks for conflicts.
- `controller` nodes (`action_parser`, `PID_Controller`, `trajectory_manager`) execute the planned actions.
- `system_state` topic (published by `controller`) is used by `recipe_tracker` and `action_planner` to adapt their behavior based on system execution progress.

**Service Calls:**
- `update_recipe` service allows updating the internal recipe from external components.
- `send_action_seq` service is used to initiate a new action plan.

---

## Testing Strategy

### ✅ Unit Testing
Each core component is individually tested using ROS Test framework (rostest) and Python's unittest.
We implemented **real unit tests** for each mandatory component:
- **Recipe Tracker** (`test_recipe_tracker.py`) : Tests parsing of JSON recipes and Simulates system states (IDLE, EXECUTING, FAILURE) and verifies step transitions.
- **Action Planner** (`test_action_planner.py`) : Validates correct planning based on system status and recipe steps.
- **Human Command Manager** (`test_human_command_manager.py`) : Tests reciving and sending human commands and send it to action planner based on the urgent policies and system state.

to try test 
#### Test the Recipe Tracker
rostest cooking_manager test_recipe_tracker.test

#### Test the Action Planner
rostest cooking_manager test_action_planner.test

#### Test the Human Command Manager
rostest cooking_manager test_human_command_manager.test




Each test checks:
- Correct behavior under different system states.
- Response to dynamic updates and unexpected conditions.
- Handling of valid and invalid human commands.

### 🔄 Integration Testing
We also performed **integration testing** across the full system.  
The following subsystems were tested together:
- `recipe_tracker`
- `action_planner`
- `human_command_manager`
- `Perceptions`
- `controller` 

We verified the complete flow from receiving a recipe to executing physical motions based on human interaction and task history untill reaching the target postoin to execute the step.

---

## Diagrams

### Behavioral Diagrams
- **Action Planning State Machine:** Describes the decision-making logic inside the `action_planner`.
- **Human Command Monitor Sequence Diagram:** Visualizes how human commands are handled, validated, or rejected.
- **Recipe Tracker Diagram:** Shows the internal flow of recipe step tracking.

### Component Diagram
- **Overall System Component Diagram:** Illustrates relationships between all (subsustems and components) nodes and message/service types.

---

## Repository Structure
'''
Cooking Manager System
├── cooking_manager
│   ├── nodes
│   │   ├── action_planner
│   │   ├── human_command_manager
│   │   └── recipe_tracker
│   ├── tests
│   │   ├── test_action_planner
│   │   ├── test_human_command_manager
│   │   └── test_recipe_tracker
│   ├── srv
│   │   ├── SendActionSeq.srv
│   │   └── UpdateRecipe.srv
│   ├── msg
│   │   └── RecipeStep.msg
├── controller
│   ├── nodes
│   │   ├── action_parser
│   │   ├── PID_Controller
│   │   └── trajectory_manager
│   ├── msg
│   │   └── SystemState.msg
├── perception
│   ├── nodes
│   │   ├── computer_vision
│   │   ├── sensor_fusion
│   │   └── slam
│   ├── msg
│   │   └── DetectedObjects.msg
'''


