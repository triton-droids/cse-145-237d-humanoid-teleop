# Triton Droids \- Humanoid Teleop

## 0\. Addressing Feedback

Fortunately, we received full credit for our project specification, and no additional feedback was provided. With that said, we plan to proceed with the current project direction while refining the scope for the remaining two weeks. Rather than attempting to complete the full real-time teleoperation pipeline, we will focus on delivering a strong simulation MVP: loading retargeted lower-body motion data into IsaacLab, training a policy to track that reference motion, and evaluating the resulting behavior through videos and tracking metrics.

## 1\. Project Overview and MVP Status

### 1.1 Project Goal and MVP

Our project aims to develop a wearable lower-body motion tracking system for humanoid robot motion imitation. The system combines embedded wearable IMU nodes with a machine learning pipeline that reconstructs human lower-body pose and uses that information for humanoid motion tracking in simulation.

### 1.2 MVP Progress

Our embedded systems MVP focused on creating a reliable motion capturing pipeline that can collect movement data from 5 IMUs with the help of ESP32s and stream it to the Jetson Nano on top of the humanoid in real time. So far, we have successfully validated the core communication and sensing pipeline.

* Successfully tested 5 wearable devices (IMUs and ESP32s) for transmitting sensor data **wirelessly** and returning accurate results.
* Successfully tested transmission from these 7 devices to the Jetson Nano mounted on the humanoid **wirelessly.**
* Confirmed that the human data generated from these devices can be used for robot retargeting.

The machine learning side MVP focuses on validating that the computer vision baseline and IMU created human pose estimation works for driving a trained policy for motion tracking on a humanoid robot in simulation.

**At this stage the MVP demonstrates that the embedded hardware can reliably collect and transmit motion data necessary for humanoid control.**

| MVP Component | Description | Status |
| :---- | :---- | :---- |
| Embedded 6DOF sensor | Build the wearable sensing unit using a microcontroller and IMU. This provides the raw motion data needed for the rest of the system. | COMPLETE |
| IMU / 6DOF pose estimation | Convert raw IMU readings into usable pose estimates, such as orientation or approximate 6DOF body-segment motion. | COMPLETE |
| WiFi data transmission | Stream sensor data wirelessly from the ESP-based wearable devices to the Jetson for real-time processing and logging. | COMPLETE |
| Human pose estimation | Use the wearable sensor data to estimate the user's lower-body pose in a format that can be used by the robot pipeline. | IN PROGRESS |
| Retargeting to humanoid model | Map estimated human motion onto the humanoid robot's URDF so the robot has feasible joint targets to follow. | COMPLETE |
| Computer Vision Baseline | A baseline using a Intel Realsense D435i for comparing our IMU human pose estimation | COMPLETE |
| Motion tracking RL environment | Modify the RL environment so the humanoid policy receives reference motion targets and is rewarded for tracking them. | IN PROGRESS |
| Verifying trained policy works in simulation | Test the trained policy in simulation to confirm the humanoid can follow the target motion stably and reasonably accurately, comparing the computer vision and IMU based human pose estimation. | IN PROGRESS |

## 2\. Completed Milestones

### 2.1 Embedded Software Milestones Completed

**2.1.1 MILESTONE I**: Embedded signals processing and validation \- COMPLETED

**Goal**: Verify that all components of the wearable embedded system can power on, communicate and transmit usable motion data.

**Completed Evidence**:

* Tested 5 IMUs, ESP32s and batteries individually.
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

**Goal**: Convert human lower-body pose data from IMU or computer vision inputs into robot-compatible lower-body motion targets. The expected output of this stage is a 10-dimensional robot joint position vector matching the humanoid URDF joint order.

**Completed Evidence**:

* [retargeting.mov](https://drive.google.com/file/u/1/d/1p0ynm6Arqcus_ENbCrR2Yxl2MD3ah-LD/view?resourcekey&usp=slides_web) — video showing the retargeting algorithm acting on our humanoid robot.

**Impact:** This milestone established the critical bridge between human motion data and robot-compatible joint targets, enabling all downstream RL training.

**2.2.2 MILESTONE IV**: Locomotion Pipeline \- COMPLETED

**Goal**: Build a working IsaacLab reinforcement learning pipeline for the humanoid robot so that future motion tracking policies can reuse the same simulation, robot model, action interface, and training infrastructure.

**Completed Evidence**:

* Implemented and registered the IsaacLab locomotion environment: `Isaac-Humanoid-Locomotion-Flat-Direct-v0`
* Implemented a standing/balance environment: `Isaac-Humanoid-Standing-Flat-Direct-v0`
* Built a 10-DOF lower-body control interface using target joint positions and PD control
* Added PPO training support through RL-Games, with additional support for RSL-RL, SKRL, and SB3 configurations
* Added domain randomization including friction, mass, gravity, joint parameters, push disturbances, observation noise, action noise, and latency
* Added delayed PD actuator modeling based on motor-pair latency assumptions
* Successfully produced training checkpoints and logs
* [better\_locomotion\_v1.mov](https://drive.google.com/file/d/1A3ou17QwLKI-J7jG-QsdAsm-UAziRkec/view?usp=sharing)

**Impact:** This milestone gives the team a working RL simulation baseline. The motion tracking policy does not need to be built from scratch; it can extend the existing locomotion environment by replacing velocity-command tracking with reference-motion tracking.

## 3\. Individual Team Member Contributions

**Darin Djapri**:

* Updating the URDF model of the robot to reflect real world changes to the humanoid.
* Rewriting base locomotion training script in IsaacSim (python) to match motion tracking. This involved modifying the reward function, modifying how actions are applied to the robot, modifying robot initialization in simulation, and modifying domain randomization.
* Adjusting parameters for domain randomization.
* Giving team members direction for all implementation details.

**Fong-Yu (Yang) Lin**:

* High level project spec clarifications and overall workflow
* Policy and reward functions research and design
* Handle Sim2Sim from IsaacLab to Mujoco
* Take care of data pipeline for model training purposes
* Team presentations and assignments tracker

**Cindy Chen**:

* Adapted the Holosoma retargeting pipeline to our lower-body humanoid by integrating our robot's URDF and MuJoCo models, registering robot-specific joint limits, and tuning the floating-base configuration for stable optimization.
* Resolved geometric mismatches between human and robot morphology by introducing a virtual pelvis marker at the true hip pivot location.
* Validated the pipeline on live motion data captured from an Intel RealSense D435 depth camera.

**Parth Trivedi**:

* Assisted in testing and validating all embedded components, including the 5 IMUs, ESP32 microcontroller, batteries and Jetson Nano.
* Developed human depth estimation code used for extracting spatial motion information for humanoid tracking.
* Contributed to the overall development of the embedded devices including debugging and assisting with system integration.

**Tauhid Malik**:

* Developed the five-IMU lower-body aggregation pipeline to convert pelvis, thigh, and shank orientation data into joint positions and relative joint rotations.
* Added tests for lower-body aggregation, missing sensor handling, standing pose reconstruction, and joint rotation accuracy.
* Built a fake IMU simulation pipeline to test teleoperation workflows before relying on physical hardware.
* Created demo scripts for lower-body aggregation and live skeleton visualization.
* Helped test and verify the embedded hardware stack, including the five IMUs, ESP32 boards, batteries, and Jetson Nano.
* Supported embedded system bring-up by debugging hardware/software issues and assisting with integration between the sensors, microcontrollers, and Jetson.
* Configured the Jetson Nano hotspot/network setup so all five ESP32 devices could stream IMU data to the Jetson over UDP.
* Assisted with end-to-end validation of the wearable sensing pipeline, from ESP32 IMU readings to Jetson-side data reception.

**Neal Jian**:

* Assisted in testing and validating the IMUs and ESP32 microcontrollers to ensure stable hardware functionality.
* Developed the firmware scripts used for sensor communication and data handling.
* Implemented the UART communication pipeline between the IMUs and ESP32s for reliable sensor data transmission.
* Built the wireless transmission pipeline for streaming motion data from the embedded devices.
* Tested the latency for wireless communication between the embedded devices and the Jetson Nano.
* Developed the reverse kinematics pipeline used to translate quaternions from IMUs to 3D vectors.
* Designed and prototyped rechargeable battery power for ESP32 through 18650 LiPo batteries.

## 4\. Remaining Milestones

**4.1 MILESTONE V**: Human Pose Estimation Pipeline \- IN PROGRESS

**Goal**: Use wearable embedded devices to estimate the user's lower-body pose in a format compatible with the humanoid robot pipeline.

**Planned Work / Evidence to Produce**:

* Finalize lower-body pose estimation from wearable sensor inputs.
* Improve accuracy and stability of pose prediction.
* Connect estimated pose outputs directly into the robot retargeting pipeline.

**Impact:** Completing this milestone will allow the system to convert real-world human motion into robot-compatible movement data, enabling real time humanoid control and teleoperation.

**4.2 MILESTONE VI**: Motion Tracking RL Environment \- IN PROGRESS

**Goal**: Extend the current IsaacLab locomotion pipeline into a lower-body motion tracking environment. The policy should receive retargeted reference motion frames and learn to follow them using residual joint position control.

**Expected Evidence to Produce:**

* IsaacLab can load retargeted robot reference motion
* The policy receives reference motion in its observations
* Policy actions are residual corrections around the reference joint pose
* Motion tracking reward is implemented
* At least one trained checkpoint can track a simple walking or turning clip in simulation
* Demo video and tracking metrics are produced

**Planned Work**:

* Add a new IsaacLab task for lower-body motion tracking.
* Load retargeted robot motion data into the training environment.
* Modify observations to include past, current, and future reference frames.
* Modify action application so policy actions are residuals around the current reference joint frame: `q_des = q_ref[t] + residual_action`
* Replace the current velocity-command reward with motion tracking rewards: joint position tracking, joint velocity tracking, root height/orientation tracking, foot position/contact tracking, and small safety penalties.
* Run initial PPO training and compare tracking accuracy across walking and turning clips.

**Impact:** This will be the direct bridge between human motion capture and robot motion imitation. It turns the project from basic locomotion training into actual teleoperation-style motion tracking.

**4.3 MILESTONE VII**: Training Data Pipeline \- IN PROGRESS

**Goal**: Build a reliable data pipeline that converts human motion data into robot-space reference trajectories usable by IsaacLab rewards and observations.

**Expected Evidence to Produce:**

* A documented robot-reference motion format
* At least one usable retargeted motion clip
* GPU-accessible reference tensors during IsaacLab training
* Basic curriculum using easy clips first

**Planned Work**:

* Define the retargeted dataset schema.
* Convert IMU or computer vision pose estimates into retargeted robot joint references.
* Cache active motion clips on GPU during training to avoid slow CPU-to-GPU transfer every timestep.
* Add a data curriculum starting from easy motions such as standing, weight shifting, walking, and turning, then gradually introduce more complex motions.
* Add adaptive segment sampling so difficult motion segments are sampled more often after basic training is stable.

**Impact:** This milestone ensures that policy training has clean, robot-compatible motion references. It also reduces the risk that training fails because of noisy, infeasible, or poorly retargeted motion data.

## 5\. Timeline

| Week | Task | Owner | Status | Output |
| :---- | :---- | :---- | :---- | :---- |
| **1** | Project kickoff, team roles, architecture design | All | Complete | Project spec, pipeline design, component selection |
| **2** | Literature review (retargeting, locomotion, RL papers), initial software setup | All | Complete | Research notes, codebase scaffolded |
| **3** | Hardware research and ordering, initial ESP32/BNO085 bring-up | Embedded | Complete | Parts ordered, initial sensor communication verified |
| **4** | Hardware finalization (5x IMU + ESP32 array); retargeting pipeline started | Parth & Neal / Cindy | Complete | All 5 IMUs validated; retargeting contract defined |
| **5** | CV baseline (MediaPipe + RealSense); embedded signals processing validation (Milestone I) | Cindy / Embedded | Complete | 3D keypoints from RGB-D camera; quaternion streaming confirmed |
| **6** | Wearable assembly and fitment (Milestone I cont.); wireless communication testing (Milestone II) | Neal / Parth & Tauhid | Complete | Leg braces assembled; <50ms UDP latency to Jetson confirmed |
| **7** | Embedded networking to Jetson; IsaacLab locomotion baseline (Milestone III + IV) | Parth / Darin & Yang | Complete | All 5 ESP32s streaming to Jetson; locomotion policy trained; retargeting to ch\_robot verified |
| **8** | Five-IMU aggregation pipeline; motion tracking RL environment (Milestone VI) | Tauhid / Darin & Yang | In Progress | IMU → joint angle pipeline; modified reward + residual action interface |
| **9** | PPO training experiments; training data pipeline; pose estimation finalization (Milestones V, VII) | ML / Embedded | In Progress | Trained checkpoint tracking walking clip; retargeted dataset with GPU-cached tensors |
| **10** | Evaluation, sim2real preparation, final integration and presentation | All | Planned | Demo video, tracking metrics, final report, project slides |
