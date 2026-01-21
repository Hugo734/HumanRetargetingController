This is the branch for ROS2; use the [ros1](https://github.com/isri-aist/HumanRetargetingController/tree/ros1) branch for ROS1.

# [HumanRetargetingController](https://github.com/isri-aist/HumanRetargetingController)
Controller for retargeting the motion from human to humanoid robot

[![CI](https://github.com/isri-aist/HumanRetargetingController/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/HumanRetargetingController/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/HumanRetargetingController/)
[![LICENSE](https://img.shields.io/github/license/isri-aist/HumanRetargetingController)](https://github.com/isri-aist/HumanRetargetingController/blob/master/LICENSE)

https://github.com/user-attachments/assets/827753ae-862e-464d-ab4b-4ea936c238c0

## Dependencies 
Follow this documentation to install the middleware mc_rtc
* [mc_rtc](https://github.com/mc-rtc/mc-rtc-superbuild/)
* ROS2

## Prerequisites to Install 
### Clone the repository
```bash
sudo clone https://github.com/Hugo734/HumanRetargetingController
```
### Install mc_mujoco to see the simulation of the controller
* [mc_mujoco](https://github.com/rohanpsingh/mc_mujoco)
Follow this documentation
```bash
mc_mujoco
```
After finish the installing check with this command if it work

### Install the SDK of Manus Gloves, follow this repository to do it
[mc_manus_plugin](https://github.com/Hugo734/mc_manus_plugin)

### To have the RHPS1 robot do this steps
With the latest version of the RHPS1 RHPS1_leap_leap_MuJoco, you will need to configurate your configuration like this:
```bash
nano ~/.config/mc_rtc/mc_rtc.yaml
```
In the part of ```MainRobot``` put this to enable the robot:
```bash
MainRobot: RHPS1_leap_leap_Mujoco
```

## Set up the workstation to use the mc_controller
To use the trackers you will need to configurate the application of SteamVr: 
[Download Steamvr here:](https://store.steampowered.com/app/250820/SteamVR/)

Follow this documentation to enable the use of SteamVr without the headseat follow this repository: 
[ViveSimpleInterface](https://github.com/isri-aist/ViveSimpleInterface?tab=readme-ov-file#hmd-settings)


## Start all the commands to use the simulation
### First 
Start the simulation with mc_mujoco:
```
mc_mujoco
```
Vive Tracker mapping and serial numbers
When running the Vive Tracker publisher:

```bash
python3 PublishPoseViveTracker.py
```
The node prints a mapping between SteamVR device serial numbers and the corresponding human body parts used by the retargeting controller:
``` bash
Map from device SN to body part:
{
    "LHR-1303A34B": "right_wrist",
    "LHR-66EDBD85": "left_wrist",
    "LHR-8FDCD86A": "waist",
    "LHR-A26B44E2": "left_elbow",
    "LHR-C0CF8E77": "right_elbow",
    "LHR-69DC3340": "right_wrist",
    "LHR-96603665": "left_wrist"
}
```

### Enabling Manus Gloves Support
To enable the use of Manus Gloves, you must start the Manus data publisher node.
The full setup and implementation details can be found in the following repository:
* [mc_manus_plugin](https://github.com/Hugo734/mc_manus_plugin) 

Below is a brief and practical summary of the required steps.

### Prerequisites

Before launching the Manus publisher, make sure that:

* The Manus license USB key is connected.

* The Manus wireless dongle(s) for each glove are connected.

* The gloves are powered on and paired.
### Start the Manus ROS2 Publisher
Run the following command:

```bash
ros2 run manus_ros2 manus_data_publisher
```
This node publishes hand and finger data from the Manus Gloves to ROS2 topics, which are later consumed by the retargeting controller.

### Recommended Manus SDK Initialization (Important)

For a more reliable connection, it is **strongly recommended** to initialize the Manus SDK **once** before running the ROS2 publisher.

#### Initialize the Manus SDK

Navigate to the Manus SDK minimal client:

```bash
cd ~/manus_sdk_integration/SDKMinimalClient_Linux
```
Run the client:
```bash
./SDKMinimalClient.out
```
When prompted, select option 1 (Integration). Once the gloves are detected and streaming correctly:
 
1. Exit the SDK client.
2. Star the ROS2 Manus publisher:

``` bash
ros2 run manus_ros2 manus_data_publisher
```

This step ensures that the Manus runtime, license, and glove connections are properly initialized before ROS2 starts consuming the data.

### Verify Manus Data Streaming
To verify that Manus data is being published correctly, run:
```bash
python3 ~/manus_ws/src/ROS2/manus_ros2/client_scripts/manus_data_viz.py
```
This script provides a simple visualization and confirmation that the glove data is being received correctly in ROS2.

### Notes
* The Manus Gloves setup is fully independent from SteamVR and Vive Trackers.
* If the Manus publisher fails to start, recheck:
    * USB license key
    * Wireless dongles
    * Gloves battey and pairing
* Restarting the Manus SDK cliente usually resolves most connection issues. 

## Simulation Demo

The following video shows the **full retargeting pipeline working in simulation** using:

- **mc_mujoco** for physics-based simulation
- **RHPS1** humanoid robot model
- **Vive Trackers** for upper-body and waist motion capture
- **Manus Gloves** for detailed hand and finger articulation
- **HumanRetargetingController** for real-time motion retargeting

The robot mirrors the human motion in real time, including:
- Arm and elbow movement
- Wrist orientation
- Waist/root motion
- Finger and hand articulation from Manus Gloves

## Simulation Demo

Click the image below to watch the simulation demo:

[▶ Watch the simulation demo](docs/media/Demo.webm)

---

### Demo Setup Summary

The demo was recorded using the following configuration:

- **Simulator:** mc_mujoco
- **Robot:** RHPS1 (`RHPS1_leap_leap_Mujoco`)
- **Control framework:** mc_rtc
- **Trackers:** HTC Vive Trackers (SteamVR, no HMD)
- **Gloves:** Manus Gloves (ROS2 publisher)
- **Operating System:** Ubuntu Linux
- **ROS version:** ROS2 Humble

---

### Notes

- The simulation runs fully in real time.
- No physical robot hardware is required.
- The same pipeline can be reused for real-robot deployment with minimal changes.

---

This demo represents the **final integration result** of the project.

