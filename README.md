# Cooperative Robotics Project

This repository showcases a series of projects focused on task prioritization and control strategies for robotic systems, including underwater vehicles and bimanual manipulations with Franka Panda robots.

## 1. Task Priority Control for Underwater Vehicle Manipulator System (UVMS)

Developed a control framework for a UVMS comprising a 6 DOF vehicle and a 7 DOF manipulator. The system was designed to execute a sequence of tasks such as:

- Waypoint navigation
- Precise landing
- Target manipulation

The implementation was carried out using MATLAB and Unity-based simulations, demonstrating effective task prioritization to achieve complex underwater missions. The framework accounted for constraints on vehicle attitude, alignment, and manipulation stability.

## 2. Bimanual Manipulation with Franka Panda Robots

Implemented a task priority inverse kinematic algorithm for controlling two Franka Panda robots as a single coordinated system. This project involved:

- Defining task hierarchies for synchronized bimanual manipulation
- Incorporating safety tasks for joint limits and minimum altitude constraints
- Developing a rigid grasping constraint to ensure coordinated movement and stability

The approach was validated through comparative analysis of desired and actual Cartesian velocities.

## 3. Cooperative Manipulation with Independent Task Prioritization

Explored cooperative manipulation by treating two Franka Panda robots as independent systems, each with its own task priority algorithm. Key aspects included:

- Independent computation of reference trajectories
- Coordination through a weighted averaging approach to achieve common objectives
- Maintaining rigid grasp constraints
- Evaluating the impact of task prioritization on cooperative velocities under varying conditions, including multiple safety tasks


## Demonstration Videos

To see the system in action, check out the demonstration videos linked below:

- [Watch the simulation video](simulations.mkv)

