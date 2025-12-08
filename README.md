# V.I.S.O.R. (Visual Intelligent System for Object detection and Retrieval)

This repository contains the complete implementation of the robotic system, including perception, navigation, manipulation, and control modules, along with documentation, datasets, and development scripts.

## Project overview

The aim of the V.I.S.O.R. project is to develop a robot capable of autonomously locating, identifying, and retrieving coloured blocks from an indoor environment. Once retrieved, each object is transported back to a matching coloured bin and deposited inside. The system integrates multiple hardware platforms and sensing peripherals to achieve full autonomy.

## Hardware platform

- **Leo Rover 1.8** – mobile base used for locomotion and onboard computation
- **Elephant Robotics MyCobot 280** – robotic arm for object grasping and manipulation
- **Intel RealSense D435** – RGB-D camera used for colour detection and depth perception
- **RPLIDAR A2M12** – 2D lidar unit used for mapping and SLAM-based navigation
