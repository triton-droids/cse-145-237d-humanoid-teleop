# Triton Droids \- Humanoid Teleop

## 1\. Project Overview and MVP Status

### 1.1 Project Goal and MVP

Our project aims to develop a wearable lower-body motion tracking system for humanoid robot motion imitation. The system combines embedded wearable IMU nodes with a machine learning pipeline that reconstructs human lower-body pose and uses that information for humanoid motion tracking in simulation.

### 1.2 MVP Progress

Our embedded systems MVP focused on creating a reliable motion capturing pipeline that can collect movement data from 7 different IMUs with the help of ESP32s and stream it to the Jetson Nano on top of the humanoid in real time. So far, we have successfully validated the core communication and sensing pipeline. 

* Successfully tested 7 wearable devices (IMUs and ESP32s) for transmitting sensor data **wirelessly** and returning accurate results.  
* Successfully tested transmission from these 7 devices to the Jetson Nano mounted on the humanoid **wirelessly.**  
*  Confirmed that the human data generated from these devices can be used for robot retargeting. 


**At this stage the MVP demonstrates that the embedded hardware can reliably collect and transmit motion data necessary for humanoid control.**

| MVP Component | Description | Status | Evidence |
| :---- | :---- | :---- | :---- |
| Embedded 6DOF sensor  | Build the wearable sensing unit using a microcontroller and IMU. This provides the raw motion data needed for the rest of the system. | COMPLETE |  |
| IMU / 6DOF pose estimation | Convert raw IMU readings into usable pose estimates, such as orientation or approximate 6DOF body-segment motion. | COMPLETE |  |
| WiFi data transmission | Stream sensor data wirelessly from the ESP-based wearable devices to the Jetson for real-time processing and logging. | COMPLETE |  |
| Human pose estimation | Use the wearable sensor data to estimate the user’s lower-body pose in a format that can be used by the robot pipeline. | IN PROGRESS |  |
| Retargeting to humanoid model | Map estimated human motion onto the humanoid robot’s URDF so the robot has feasible joint targets to follow. | COMPLETE |  |
| Computer Vision Baseline | A baseline using a Intel Realsense D435i for comparing our IMU human pose estimation | COMPLETE |  |
| Motion tracking RL environment | Modify the RL environment so the humanoid policy receives reference motion targets and is rewarded for tracking them. | IN PROGRESS |  |
| Verifying trained policy works in simulation | Test the trained policy in simulation to confirm the humanoid can follow the target motion stably and reasonably accurately, comparing the computer vision and IMU based human pose estimation. | IN PROGRESS |  |

## 2\. Completed Milestones

### 2.1 Embedded Software Milestones Completed

**2.1.1 MILESTONE I**: Embedded signals processing and validation \- COMPLETED

**Goal**: Verify that all components of the wearable embedded system can power on, communicate and transmit usable motion data. 

**Completed Evidence**:

* Tested 7 IMUs, ESP32s and batteries individually.   
* Verified communication between IMUs and ESP32s using the UART protocol.   
* Tested and validated the Jetson Nano as the onboard computer device for future integrations.   
* Confirmed successful signal transmission.  
* Verified reception of 3D vector data from all devices.

**Impact:** This milestone was needed to establish that all embedded devices were working as intended and we could build on top of them for motion capture and human control. 

**2.1.2 MILESTONE II**: Wireless Communication Testing \- COMPLETED

**Goal**: Ensure that the embedded devices could consistently communicate and transmit data using wireless protocols. 

**Completed Evidence**:

* Tested WIFI (UDP Protocol) communication between devices and host machine.  
* Confirmed stable packet transmission during movement tests.  
* Confirmed that the data was transmitting with low latency (\<50 ms).

**Impact:** Reliable and fast communication is critical for real time humanoid motion retargeting and motion. 

### 2.2 Machine Learning Milestones Completed

**2.2.1 MILESTONE III**: Coordinates Retargeting \- COMPLETED

**Goal**: 

**Completed Evidence**:

* 

**Impact:** 

**2.2.2 MILESTONE IV**: Locomotion Pipeline \- COMPLETED

**Goal**: 

**Completed Evidence**:

* 

**Impact:** 

## 3\. Individual Team Member Contributions

Darin:

* Updating the URDF model of the robot to reflect real world changes to the humanoid.  
* Rewriting base locomotion training script in IsaacSim (python) to match motion tracking.  
  This involved modifying the reward function, modifying how actions are applied to the robot, modifying robot initialization in simulation, and modifying domain randomization.  
* Adjusting parameters for domain randomization.  
* Giving team members direction for all implementation details.

Parth: 

* Assisted in testing and validating all embedded components, including the 7 IMUs, ESP32 microcontroller, batteries and Jetson Nano.  
* Developed human depth estimation code used for extracting spatial motion information for humanoid tracking.   
* Contributed to the overall development of the embedded devices including debugging and assisting with system integration. 

Fong-Yu:

* High level project spec clarifications and overall workflow  
* Policy and reward functions research and design  
* Handle Sim2Sim from IsaacLab to Mujoco  
* Take care of data pipeline for model training purposes  
* Team presentations and assignments tracker

## 4\. Future Milestones

**4.1 MILESTONE V**: Motion Tracking Pipeline \- IN PROGRESS

**Goal**: 

**Completed Evidence**:

* 

**Impact:** 

**4.2 MILESTONE VI**: Training Data Pipeline \- IN PROGRESS

**Goal**: 

**Completed Evidence**:

* 

**Impact:** 

## 5\. Remaining Work and Timeline