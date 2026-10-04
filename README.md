# Office-Assistant Robot

A team robotics project at Linköping University using ROS2 and Nav2.

**Project:** An autonomous office-assistant robot for environment exploration, natural-language command understanding, and task execution through navigation.

## My Contribution

I developed a human-aware local controller that uses LiDAR scans and human position estimates to generate robot velocity commands.

The controller supports:
- Following a navigation path.
- Avoiding nearby humans and obstacles.
- Logging navigation data for evaluation.

## Code Structure

- `air_navigation_examples/` — human-aware controller, visualization, and evaluation scripts.
- `air_navigation/` — Python–C++ integration for Nav2.
- `air_bringup/` — launch files, navigation configuration, and maps.
- `exploration/` — exploration module.
- `project_interface/` — custom ROS service definition.

My main implementation is in `air_navigation_examples/air_navigation_examples/__init__.py`.

**Technologies:** Python, ROS2, Nav2, LiDAR, NumPy.
