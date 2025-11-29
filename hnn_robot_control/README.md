# Hnn Robot Control

Hardware interfaces and ros2_control configuration for Hnn robots.

## Package Structure

This package contains:
- **Hardware interfaces**: C++ implementation of hardware interfaces for different Hnn robot types
- **Controller configurations**: YAML files for ros2_control controllers
- **Plugin descriptions**: XML files for hardware interface plugins

## Hardware Interfaces

Currently supported robot types:
- `HnnDiffRobotHardware`: Hardware interface for Hnn Diff robot

## Usage

This package is typically used through launch files in `hnn_robot_bringup` package.

## License

MIT License

