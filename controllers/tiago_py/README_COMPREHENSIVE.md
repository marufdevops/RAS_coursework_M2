# TiaGo Advanced Multi-Goal Navigation System

## Project Overview

This project implements a sophisticated multi-goal navigation system for the TiaGo robot, integrating PDDL (Planning Domain Definition Language) path planning with real-time robot control, obstacle avoidance, and intelligent goal queue management. The system provides optimal navigation paths while maintaining robust real-time performance and precise goal completion.

## Key Features

### Multi-Goal Navigation
- **Sequential Processing**: Automatic progression through goal queue
- **Dynamic Addition**: Goals can be added during active navigation  
- **Progress Tracking**: Real-time completion status and remaining goals
- **Seamless Transitions**: Smooth navigation between consecutive targets

### Optimal Path Planning
- **PDDL-Based**: Uses PDDL for optimal solutions
- **Strategic Waypoints**: 11-node network provides space coverage
- **Cost Optimization**: Euclidean distance minimization for shortest routes

### Robust Control System
- **Precise Positioning**: 5cm tolerance for waypoint navigation
- **Proportional Feedback**: Smooth acceleration and deceleration
- **Obstacle Avoidance**: Real-time detection with safe pause behavior
- **Error Recovery**: Graceful handling of navigation failures

### Real-Time Monitoring
- **Status Reporting**: Periodic updates every 10 seconds
- **Goal Detection**: Dual validation using distance and polygon methods
- **Progress Indicators**: Queue status and completion percentages
- **Debug Information**: Comprehensive logging for system analysis

## 🏗️ System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Main Control  │    │   Navigation     │    │   PDDL Planning │
│   (tiago_py.py) │◄──►│   Controller     │◄──►│   System        │
│                 │    │                  │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Keyboard      │    │   Node Network   │    │   Goal Checker  │
│   Reader        │    │   (11 nodes)     │    │   (Polygons)    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### Core Components

1. **Main Controller** (`tiago_py.py`) - System orchestration and user interface
2. **Navigation Controller** (`navigation_controller.py`) - Robot control and state management  
3. **PDDL Planning System** (`pddl_system.py`) - Optimal path planning with A* search
4. **Node Network** (`node_network.py`) - Strategic waypoint system and graph algorithms
5. **Support Modules** - Goal detection, keyboard input, and utility functions

### Supported Targets
- **balls**: Soccer ball collection area (bottom-left region)
- **green**: Green target zone (bottom-right region)
- **ducks**: Duck collection area (top-left region)  
- **red**: Red target zone (top-right region)

### Expected Output
```
BALLS GOAL REACHED! Total distance covered: 5.23 meters
Goal 1 of 3 completed!
Starting navigation to next goal: green
Remaining queue: ['red', 'ducks']
```

### Hardware Requirements
- TiaGo robot with differential drive system
- LiDAR sensor for obstacle detection (0.3m threshold)
- GPS sensor for position feedback (±0.3m accuracy)
- Compass sensor for orientation control (±6° tolerance)

### Software Dependencies
- Webots robotics simulator
- Python 3.x with standard libraries
- Optional: pyperplan for optimal PDDL solving
- Shapely library for geometric calculations

### Network Topology
- **Nodes**: 11 strategically placed waypoints
- **Connections**: 16 bidirectional edges