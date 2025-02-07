# Sally

## Overview

Sally is an 8.3 kg magnetic-wheeled climbing robot for inspecting steel structures using a portable X-ray fluorescence sensor. This package contains a URDF model of Sally.

## Usage

The `sally_description` package contains a `display.launch.py` launch file that will display the robot geometry in `rviz` with an interface for changing joint angles.

```
ros2 launch sally_description display.launch.py
```