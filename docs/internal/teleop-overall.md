# People List

# Humanoid Teleop

## Members:

- Darin (ML)*✓*

- Parth (embedded) *✓*

- Neal (Embedded) *✓*

- Cindy (embedded/ML) *✓*

- Yang (ML) *✓*

- Malik (Embedded)

# "Try not to break the egg"

## Goals

Low-Cost Force-Sensing Robotic Hand attached to the ARCTOS 6-DOF Robotic Arm.  
Teach the arm to pick up an egg and place it into a container using ML.  
3-DOF wrist with visual servoing

**Show that Tactile-sensing performs better than vision-only**  
This could be for

- Teleoperation  
- Reinforcement Learning Sim2Real  
- Imitation Learning using pi0.6 model

### Force Sensor Options

Force Sensing Resistor (FSR) [https://www.pololu.com/product/1696](https://www.pololu.com/product/1696)

### Hand Options

AmazingHand: We already have this 8DOF 4Finger non-compliant linkage-based.

LEAP Hand: Direct drive gives us easy accurate finger joint angles, but like $2000

Yeah Robotic Hand: Uses ST3215 motors which we have a bunch from the SO101s. Tendon based

LEAP Hand V2: [https://v2.leaphand.com/](https://v2.leaphand.com/) $300-$500 8-DOF 4 finger. Tendon based. Missing CAD

Aero Hand $314 tendon based 6-DOF actuated 5 finger  
[https://shop.tetheria.ai/\#f500bddc99d7a066d47dd4c7025e8bae](https://shop.tetheria.ai/#f500bddc99d7a066d47dd4c7025e8bae)  
[https://shop.tetheria.ai/products/pcbs?variant=51308527321400](https://shop.tetheria.ai/products/pcbs?variant=51308527321400)

### Vision Options

RGBD Intel Realsense  
Need an adapter to mount on the arm/hand

### Teleoperation Options

I have a Oculus/Meta Quest 2  
I have a SteamVR Valve Index Basestation 2.0 \+ Controllers with hand presence

### Task Assignment

- [ ] Explore using Quest 2 \+ Hand Tracking. Get 3D position of your hand and send it to the computer.  
- [ ] Make a basic circuit using the force sensor and calibrate it.  
- [ ] Make ARCTOS software: Do 3D Point to Joint Angles

### To Buy

- [ ] Spend money and buy a hand  
- [ ] Spend money and buy more force sensors

### Wrist Options

The inspiration for this feature  
[https://www.linkedin.com/posts/aadeelakhtar\_ability-hand-on-a-3-dof-wrist-with-camera-activity-7444963169514561536-u7ah/](https://www.linkedin.com/posts/aadeelakhtar_ability-hand-on-a-3-dof-wrist-with-camera-activity-7444963169514561536-u7ah/)

Parloma 3-DOF 3D-Printed  
[https://journals.sagepub.com/doi/10.5772/64113?utm\_source=chatgpt.com](https://journals.sagepub.com/doi/10.5772/64113?utm_source=chatgpt.com)  
[https://www.thingiverse.com/thing:1597390](https://www.thingiverse.com/thing:1597390)  
This one looks nice

Pollen Robotics  
[https://medium.com/pollen-robotics/orbita-is-turning-heads-literally-d10d378550e2](https://medium.com/pollen-robotics/orbita-is-turning-heads-literally-d10d378550e2)  
[https://cad.onshape.com/documents/5ca7f684d33fbcace89ad4d3/w/29fb37b24eb49ed098217242/e/aa193690db5622f1e37493db](https://cad.onshape.com/documents/5ca7f684d33fbcace89ad4d3/w/29fb37b24eb49ed098217242/e/aa193690db5622f1e37493db)  
This one looks hard and overkill in terms of strength

# "Humanoid Teleoperation"

## GOALS

Project explanations in under seven words: Teleoperate Humanoid to imitate human pose

Embedded device OR camera OR existing motion data

  |  
 V

Human pose (each point is 3D) \-\> Retargeting (output is also 3D points)   \<- Cindy

  |  
 V

3D points \-\> Motion Tracker Policy in Sim (still don't know whats the best way to do this) \-\> Deploy same policy on real robot

Embedded:

- [ ] Low latency embedded device wearables that can send 6D (or 7D) pose information to the humanoid robot onboard computer  
      - [ ] What is required to generate a 7D pose?  
            For each device, an IMU can provide orientation, but what about position? Is it possible to do some sort of networking between the embedded devices and figure out relative positions?

            IMU \+ ESP32 is the first baseline. Networking can be done with bluetooth  
      - [ ] Potential integration with UWB

      - [ ] What networking scheme do we use to send data from the devices to the onboard computer?  
- [ ] Onboard computer (Jetson) setup

ML:

- [ ] Human pose construction using poses generated from the embedded devices  
- [ ] (1) Use a RGBD camera for human pose construction as a baseline to compare  
      examples: mediapipe \+ fastapi  
- [ ] Train a model to be able to track poses in MuJoCo or Isaacsim  
      - [ ] (2) (prerequisite) Retargeting motion data onto the humanoid robots joints  
            See [https://github.com/amazon-far/holosoma/tree/main/src/holosoma\_retargeting/holosoma\_retargeting](https://github.com/amazon-far/holosoma/tree/main/src/holosoma_retargeting/holosoma_retargeting) for reference  
      - [ ] just do training directly on existing motion data  
      - [ ] Read up the motion tracking papers first

            (3) Modify RL environment to include motion tracking reward, or develop environment from scratch in MuJoCo

Miscellaneous to do's:

- [ ] Sim2Real for basic locomotion test on humanoid robot (Darin is working on this)  
- [ ] Motor calibration

Note: While waiting for the embedded team to finish, we can attempt to train a controller to track pose estimates generated from keyboard controls

Extra note: All of the above will be converted to github issues for our project tracking along with timelines.

## Entire Project Top-Down

Embedded \-\> Pose Estimation \-\> Retargeting \-\> RL Training \-\> Sim2Sim \-\> Sim2Real

Embedded:

1. Order parts that track the human pose with 10 DoF?  
   1. 3 sensors on each leg \-\> 5 actuators on robot  
2. Get the data tracking working and sending back the human pose successfully to ML team  
   1. Q: What’s the unified pose format between embedded and RGBD camera? What’s the output from this to get as input to robot observation space?  
      1. 3 3D vectors with their own rotations (pitch, row, yaw)   
3. ESP32 \-\> Jetson-Orin?  
4. See above for more details

ML:

1. (Cindy) Get human pose (skeleton / 3D points) from (1) Embedded device, (2) RGBD camera, (3) existing motion data  
   1. Have it running on sim (IsaacLab) in real time  
      1. We’re not doing this but using a neural net  
   2. Q: What’s the 3D point format? world frame？robot frame? camera frame?  
      1. **Robot Frame**  
2. (Cindy) Retargeting to actual robot  
   1. Have URDF as spec describing actual robot  
   2. Retargeting the skeleton captured from RGBD camera to this URDF using Holosoma ([https://github.com/amazon-far/holosoma](https://github.com/amazon-far/holosoma))  
      1. Holosoma covers the entire pipeline from retargeting, sim2sim, and sim2real\!  
      2. Holosoma framework actually isn’t that good?  
   3. Q: What is the format of the 3D points outputs? (NumPy array? JSON?)  
      1.   
   4. Q: Which frame is the coordinate system? (World/camera/robot base)  
      1. Robot frame  
   5. Q: What is the output frequency in Hz?  
      1. We can set this to …  
   6. Q: Is it per joint or per keypoint?  
      1. Per keypoint — Keypoints are roughly at joints  
3. (Yang, Darin) train in sim (isaaclab)  
   1. Motion Tracker Policy in Sim  
   2. Now we have the correct 3D points data that works for our physical humanoid  robot, we can start training it  
   3. Before actual training, we have to figure out what policies and reward functions would work the best to train the model on  
      1. Research on motion tracking paper:  
         1. motion tracking RL paper: [https://arxiv.org/pdf/2512.01996](https://arxiv.org/pdf/2512.01996)  
            1. Different from on-Policy (PPO), off-policy (FastSAC/FastTD3) stores all the past experiences in the buffer rather than throwing them away. This makes the training faster bc no need to collect data all over again.  
         2. whole body motion tracking paper: [https://arxiv.org/pdf/2511.07820](https://arxiv.org/pdf/2511.07820)  
      2. Try training the model with existing motion data before the retargeting work is done  
      3. What are the best approaches?  
   4. Things to make sure and declare before starting training:  
      1. Reward function  
      2. Observation space  
         1. Robot’s joint positions  
         2. Robot’s joint velocities  
         3. Target 3D pose keypoints  
         4. Robot’s current base orientation  
      3. Action space (joint angles? Torques?)  
         1. Target joint angles（PD control）  
         2. Make sure the number of DoF \== URDF  
      4. Policy (MLP?Transformer?)  
      5. Termination condition  
4. (Yang, Darin) sim2sim (isaaclab to mujoco)  
   1. IsaacLab has low fidelity while Mujoco has high fidelity  
   2. Training on IsaacLab and evaluate on Mujoco can make sure Sim2Real smoother  
5. (Yang, Darin) sim2real  
   1. Deploy it to actual robot

## Project Hardware Requirements:

- 

## References:

[paper this idea is based off of](https://nvlabs.github.io/GEAR-SONIC/static/pdf/sonic_paper.pdf)

ml references:  
a good start to understanding sim2real locomotion: [https://manual.asimov.inc/v1/locomotion](https://manual.asimov.inc/v1/locomotion)  
motion tracking RL paper: [https://arxiv.org/pdf/2512.01996](https://arxiv.org/pdf/2512.01996)  
whole body motion tracking paper: [https://arxiv.org/pdf/2511.07820](https://arxiv.org/pdf/2511.07820)

embedded references:  
[https://github.com/slimevr](https://github.com/slimevr)  
[pico waist tracker](https://www.aliexpress.us/item/3256810590814069.html?src=google&src=google&albch=shopping&acnt=708-803-3821&isdl=y&slnk=&plac=&mtctp=&albbt=Google_7_shopping&aff_platform=google&aff_short_key=UneMJZVf&gclsrc=aw.ds&albagn=888888&ds_e_adid=&ds_e_matchtype=&ds_e_device=c&ds_e_network=x&ds_e_product_group_id=&ds_e_product_id=en3256810590814069&ds_e_product_merchant_id=5691154150&ds_e_product_country=US&ds_e_product_language=en&ds_e_product_channel=online&ds_e_product_store_id=&ds_url_v=2&albcp=20269108796&albag=&isSmbAutoCall=false&needSmbHouyi=false&gad_source=1&gad_campaignid=20273564092&gbraid=0AAAAAD6I-hFjh1oLKaJn4hccmyiWT5fQA&gclid=Cj0KCQjwp7jOBhDGARIsABe7C4eXTF_3tXzI9uvktOALVgn2EpmXuK7MgOxfwF_TTkHdTpK9z7EMUuMaApjzEALw_wcB&gatewayAdapt=glo2usa)  
[pic leg trackers](https://www.aliexpress.us/item/3256807466919502.html?src=google&src=google&albch=shopping&acnt=708-803-3821&isdl=y&slnk=&plac=&mtctp=&albbt=Google_7_shopping&aff_platform=google&aff_short_key=UneMJZVf&gclsrc=aw.ds&albagn=888888&ds_e_adid=&ds_e_matchtype=&ds_e_device=c&ds_e_network=x&ds_e_product_group_id=&ds_e_product_id=en3256807466919502&ds_e_product_merchant_id=5371548127&ds_e_product_country=US&ds_e_product_language=en&ds_e_product_channel=online&ds_e_product_store_id=&ds_url_v=2&albcp=20269108796&albag=&isSmbAutoCall=false&needSmbHouyi=false&gad_source=1&gad_campaignid=20273564092&gbraid=0AAAAAD6I-hFjh1oLKaJn4hccmyiWT5fQA&gclid=Cj0KCQjwp7jOBhDGARIsABe7C4c5SFfGXTHy_By7uSm29khWce5V1z7y5aRJzTPpvNzTtjPLVszz_MEaAi8vEALw_wcB&gatewayAdapt=glo2usa)  
[altermative for position estimtion](https://arxiv.org/pdf/2403.10194)

using phones to do human pose estimation: [https://arxiv.org/pdf/2504.12492v1](https://arxiv.org/pdf/2504.12492v1)

# Teleop Meeting Notes

# Humanoid Teleoperation Meeting Notes

## Week 8:

Parth:   
Initially Neal was making a 3d component but then bought 10 pcb breadboards  
Need to buy more buckles.

For the MVP we don't need jetson nano

## Week 5:

PARTS HAVE ARRIVED.

Darin: 

Cindy:  
Continue fine-tuning the retargeting behavior

Parth:   
Combined IMU \+ ESP32, doing a test script on reading quaternions from IMU's.   
Successfully got computer vision human pose estimation working with mediapipe  
Work with the jetson and wirelessly communicating

Yang:  
Done researching reward and policy and step into design and implementations

Tauhid:  
\-//- as parth, going over details on how to combine all the IMU readings  
How to sync 7 esp32 readings into the jetson

Neal:  
\-//- as parth, going over details on how to combine all the IMU readings  
will show in class  
Will work on multipoint estimation, testing calibration

## Week 2:

Cindy: Did review of retargeting \+ locomotion, prefers   
Parth: Optical is harder vs as opposed to using embedded devices for human pose estimation  
Yang: Read documentation for locomotion, having reached RL part, and has attempted mujoco  
Tauhid: Did early reading  
Neal: Read over papers that uses UWB sensors to get spatial distance

Want to get as low level as possible:  
When using IMU, what integrator are we using?  
RK Integration?   
If combining with UWB, what does that look like?

Use holosoma related retargeting code (or spin up our own) and make it work on our custom humanoid urdf.

# Humanoid Teleop \- Embedded

Meeting notes- 10 april 

Agenda:  
Goal: send signals about direction from human legs. Comparing from the baseline model using the camera. Reconstruct legs using the data passed from the sensors. 

* Decide on the parts \- number of each part  
  * Number of esp32c3:   
    * Consult Bryce  
    * Option A: One

      7 IMUs → 1 ESP32 → BLE

      

      ### Pros

* Simple logic  
* Easy synchronization

  ### Cons

  * Long wires across body  
    * Poor wearability  
      * I2C scaling issues

        

    * Option B: Three

      Left Leg ESP32  → BLE

      Right Leg ESP32 → BLE

      Waist ESP32     → BLE

      

      ### Pros

* Wearable and scalable  
* Short wiring  
* Clean modular design

  ### Cons

* Requires multi-device synchronization


  * Number of IMU \- [BNO085](https://www.adafruit.com/product/4754) ($25 each): 7   
    * On each leg 3 and 1 for waist(for reference) with wires  
  * Number of Batteries:   
    * Three Batteries: for each esp32c3 if going with option B  
    * One Battery: if doing option A  
* Backup plans for some methods  
  * If ESP 32 and BNO085 doesn’t work well with the SPI protocol, it would be problematic and we might need to use UART as backup interfaces  
    * For UART, it’s best to have one MCU for each IMU, but this means purchasing more esp32c3 and batteries, also we would transition to full wireless settings (this is both a backup and an upgrade actually)  
  * If we can’t handle the three esp32c3 synchronization, our backup would be to handle all 7 IMUs with a single MCU  
    * We need to use another MCU instead of c3 to handle that much I/O. (I personally have two usable ones with enough I/O and am willing to contribute)  
* How parts work together  
  * Central MCU \- the MCU on the waist. Responsible for synchronization between sub MCUs, integrating all IMU output and communicating with the computation center(Jetson Nano)  
    * Two sub MCUs \-  each for 3 IMU output reading via SPI and sending them to the central MCU wirelessly  
    *   
* Raw data: timestamp, ax, ay, az, gx, gy, gz, mx, my, mz  
  * `ax, ay, az` \= accelerometer  
  * `gx, gy, gz` \= gyroscope  
  * Fused data (quaternions): timestamp, qx, qy, qz, qw

Going with option B. 

The project aims to build a wearable lower-body teleoperation system that captures human leg motion using 7 BNO085 IMUs and maps it to a humanoid robot in real time. The proposed architecture uses 3 ESP32 boards (left leg, right leg, waist) connected to a Jetson for centralized processing, with SPI communication between IMUs and ESP32s, UART/serial from ESP32s to the Jetson, and a shared wired sync pulse to align sensor data reliably. The waist IMU serves as the global reference frame, while relative orientations between body segments are used to compute hip, knee, and ankle joint angles. If synchronization or hardware complexity becomes an issue, fallback options include software timestamp alignment, a simpler one-ESP32 benchtop prototype, changing interfaces, or switching IMU families. Overall, the recommended path is a modular 3-ESP32 wearable design with Jetson-based fusion and control.

# Individual Project/Course Goals

note that the tactile hands section is below.

# Triton Droids \- Humanoid Teleop

**1\. Which project are you committing to?**  
Triton Droids \- Humanoid Teleop

**2\. Why are you committing to this project?**  
I am committing to this project because I am passionate about …

**3\. What do you view as the ultimate goal(s) of the project?**  
The ultimate goal of this project is to develop embedded devices capable of tracking 3D position for humanoid teleoperation use. 

**4\. List some technical challenges that will need to be overcome to reach those goal(s).**  
We would need to overcome bridging the ML and embedded side of the project, doing continuous integration testing around week 6 up until finals week.

A lot of research will have to be done as well given that we are pursuing low cost options for the embedded devices, which may come at the cost of performance (for instance in measurement accuracy or latency)

**5\. What skills do you currently have that will enable you to contribute to the project?**  
Darin: My skills include simulation programming (MuJoCo, IsaacLab) as well as reinforcement learning, and on the embedded side I have past experience developing in ROS2, and working with sensors such as cameras and IMU's.

**6\. What skills are you going to have to develop that will enable you to contribute to the project?**  
Darin: I will likely need to develop more embedded software knowledge and skills, particularly in networking with as low latency as possible.

Parth: I will work on the embedded side of the project. My main task would be to develop the leg braces for the human to wear for teleoperation. I have embedded skills from working on tritondroids and will develop them while working on this project. I will also need to develop hardware comparison skills and fusion techniques to get the required outputs from the sensors. 

Cindy: I will need to develop some embedded sys knowledge and some hands-on work on retargeting the motion data and using the sim2real. 

Neal: I will need to learn some embedded network skills for communication with embedded systems and also new sensor fusion knowledge that is critical for our goals to construct spatial models with wearables.

**7\. Who are your project teammates and what skills do they have that you think will be valuable for this project? In the case where you are doing a solo project, please go into more depth as to your prior experience you bring to the table.**

Darin Djapri: My primary skills are in machine learning and model deployment on edge devices. I specialize in reinforcement learning, and have prior experience with humanoid locomotion and robotic manipulation, including embedded related frameworks such as ROS2 and TensorRT. My skills would be valuable for bridging the embedded portion and machine learning portions of the project.

Fong-Yu (Yang) Lin: My skills are in AI agents, Machine Learning, and a tiny bit of Reinforcement Learning. I have experience in operating a G1 humanoid robot. I will be mostly learning while implementing, and I will be valuable for locomotion and Sim2Real implementation.

Parth Trivedi: I have helped develop an arctos robot arm with 6 degrees of freedom, I also did embedded work on it. I also have some experience with reinforcement learning and AI agents using projects and coursework. 

Cindy Chen: My skills are software development, AI/ML, and some RL algorithms. I have experience with voice AI applications and ML model building. I will be working on the ML part of the project, and I’m also willing to learn some embedded systems part.

Tauhid Malik: My skills are in embedded systems especially the software side of things. I have experience with autonomous car and have made mini autonomous cars with Jack Silberman. I am also an embedded team member or Triton AI working on their Autonomous Go karts.

Neal Jian: My skills are embedded systems and software development. I did personal projects with MCU and sensors, mostly smart home devices. I also have some experience with pcb design and can help with the prototyping of the wearable sensor systems.

# 

# Triton Droids \- Tactile Hands

**1\. Which project are you committing to?**  
Triton Droids \- Tactile Hands

**2\. Why are you committing to this project?**  
I am committing to this project because I am passionate about …

**3\. What do you view as the ultimate goal(s) of the project?**

The ultimate goal of this project is to create a tactile-sensing robotic hand, and demonstrate that tactile sensing provides better results than a vision-only system. This can be demonstrated using a VR teleoperation system with either a fixed hand or a robotic arm. We can also show that force feedback data helps with simulation performance with pick-and-place tasks.

**4\. List some technical challenges that will need to be overcome to reach those goal(s).**

We will need to get a robotic hand that performs well in simulation. We will need to attach the force sensors to the hand. We will need to calibrate the sensors to be accurate. We will need to calibrate the robot arm to get 3D-point to joint angles. 

**5\. What skills do you currently have that will enable you to contribute to the project?**

**6\. What skills are you going to have to develop that will enable you to contribute to the project?**

**7\. Who are your project teammates and what skills do they have that you think will be valuable for this project? In the case where you are doing a solo project, please go into more depth as to your prior experience you bring to the table.**

# Project Specification

# Project Charter:

## High-level Project Description Range

Goal: To have the humanoid robot (legs) do the same thing as the person wearing embedded devices does (e.g. walking, dancing, etc).

High-Level Development Process Flow:  
Embedded \-\> Pose Estimation \-\> Retargeting \-\> RL Training \-\> Sim2Sim \-\> Sim2Real

Project Overview:   
We’re building a system that allows a humanoid robot to mirror a person's lower-body movements in real-time. Essentially, we want the robot to act as a "shadow" for your legs—when you walk, squat, or lift a leg, the robot does the same. We’re using wearable sensors and cameras to capture leg poses, translating those movements into data the robot understands, and then using AI (Reinforcement Learning) to teach the robot how to balance and move its limbs safely. The goal is to make robot locomotion more intuitive by using our own lower-body movements as the controller

## Execution Strategy

Project Approach:  
Our project follows a top-down pipeline from hardware sensing to real-world robot deployment, specifically focused on the legs:

* Sensing & Pose Construction: We are utilizing low-latency wearable IMU sensors (ESP32-based) to send 6D or 7D pose information to the robot. Our baseline involves five sensors per leg to correspond with the five actuators on each robot leg. We are also using an RGBD camera as a secondary baseline for pose construction.  
* Retargeting: Because human legs and robot legs have different joint structures, we use a retargeting process. This maps human 3D keypoints onto the robot’s specific URDF (the digital blueprint of the actual hardware) using frameworks like Holosoma.  
* Simulation & Training: Most of the learning happens in IsaacLab. We are training a Motion Tracker Policy where the robot is rewarded for matching target leg keypoints while maintaining balance. The observation space includes the robot's joint positions, velocities, and target pose keypoints.  
* Sim2Sim & Sim2Real: To ensure the AI works on physical hardware, we evaluate the IsaacLab-trained policy in MuJoCo, which offers higher physical fidelity. Once validated, we deploy the policy to the actual humanoid for real-world testing.

## Defining a Minimum Viable Product (MVP)

Our MVP is a system that can track the wearer's basic lower-body movements, such as walking and squatting, in real-time.

Long-term Goals: Achieving full lower-body teleoperation, including complex navigation and possibly add on other methods(extra sensor/ ML models) on the embedded systems for more precise motion tracking.

* Functionality:   
  * The humanoid robot can track the wearer’s basic lower body movements (e.g., walking, squatting) in real time.  
  * It uses the orientation data from the wearable IMU sensors and solves for the human pose.  
    * Fallback: RGBD camera as input and has been successfully deployed and runs stably on a real robot.  
* Success Metric: The system must be successfully deployed and run stably on a real robot.  
* Quarterly Objectives:   
  * Embedded side:   
    * We will first finalize the wearable embedded systems to be able to output the orientations of each part.   
    * Research on approaches for pose estimation out of only IMU data (using simple rigid biomechanics of a lower body skeleton through OpenSim.)  
    * Research on approaches for pose estimation with IMU and other assisting data collection(UWB sensors/Camera)  
    * Then we will deploy the pose estimation system on the central computing unit (Jetston Nano), so we could hand over the pose data to the ML side.  
  * ML side:  
    * We will first finalize the leg hardware return data and unify it with fallback (RGBD camera) data format.  
    * Retargeting with existing human pose data before the embedded hardware is set up.  
    * Research on approaches for policies and reward functions for the training process to go smoothly.  
    * Research and finalize all sorts of data formats such as actions space, observation space (3D points returnings).  
    * Move to RL training in IsaacLab.  
    * Sim2Sim from IsaacLab to Mujoco for generalization and to make sure Sim2Real transitions to be as smooth (or say less challenging) as possible.  
    * Perform Sim2Real locomotion tests.

## Risk Management

Risk 1: Embedded system delay  
Fallback: Use RGBD camera for pose estimation

Risk 2: Holosoma framework not reliable  
Fallback: Use Sonic’s algorithm for retargeting

Risk 3: Retargeting stuck  
Fallback：Use existing motion capture data first

Risk 4: Sim2Real domain gap too large  
Fallback: Evaluate with Sim2Sim (IsaacLab to MuJoCo) first and then deploy or move onto Siim2Real

# Group Management

## Team Roles

Darin: Team Lead, ML, RL, Policy & Reward Functions optimizations and training.  
Cindy: Human Poses extraction and 3D points retargeting  
Yang: ML, RL, Policy & Reward Functions optimizations and training.  
Parth/Neal/Tauhid: Focused on embedded systems, leg braces, sensor fusion, and networking .

## Decision-making Process

We brainstorm ideas together, evaluate if they are valid during the meetings and note them down in Meeting Minutes. So far, the team lead is suggesting most of the ideas and high level process, but we tend to split up more and have ideas collide with each other for the greatest outcome while maintaining its realisticness.

## Communication Structure

* Communicate through Discord group channel  
* Github Issues to track our project progress  
* Weekly meetings on Tuesday 10am

## Schedule Management Process

Each of the team members decide what they want to do and set up the schedule, deadline, milestones on their own and have them mentioned in the meetings for everyone to recognize the work. The team evaluates if the tasks are valid and doable and if the member needs more help, the team can figure out how to adjust the schedule.

# Project Development

## Team Technical Expertise

Development Roles: Darin, Yang, and Cindy handle the ML and Simulation side, while Parth, Neal, and Tauhid manage the embedded hardware and networking.

## Testing Infrastructure

We will conduct continuous integration testing from Week 6 through finals. Initial testing starts in IsaacLab (low fidelity), moves to MuJoCo (high fidelity), and concludes with physical motor calibration on the robot .

## Technological Requirements

Hardware：

* BNO085(IMU Sensor)  
* Esp32c3 devboard (MCU)   
* Jetson Nano (Computing Unit)

Software：

* ESP-NOW(Networking between the wearables)  
* OpenSim (pose estimation)  
* IsaacLab  
* MuJoCo  
* MediaPipe \+ FastAPI（RGBD pose estimation baseline）  
* Holosoma  
* Robot URDF

## Documentation Process

* We use Google Docs for documentations  
* Several tabs for different purposes such as Overview, Project Spec, Weekly Meeting Minutes, etc  
* We first set our goals for the project and have a high level description of our entire project workflow. Then we document everything along the research and development process.

# 

# Project Milestones and Schedule

## Defining Milestones

We use Github Issues to keep track of the milestones and progress.

**High-Level Deliverables:**  
Retargeting Pipeline (Week 4): Successful mapping of existing motion data to the humanoid URDF.

Hardware Baseline (Week 5): Finalized embedded hardware and functional 6DOF prototype.

Simulation Environment (Week 7): Modified RL environment with motion tracking rewards ready for training.

Final Deployment (Week 10): Stable Sim2Real locomotion running on the actual robot.

## Milestone Priorities

* **High Priority (Critical Path):** Retargeting pipeline, hardware sensor array completion, and the RL motion tracking policy.

* **Medium Priority:** Computer Vision (CV) baseline and Sim2Sim validation to bridge the gap between IsaacLab and MuJoCo.

* **Low Priority/Stretch Goals:** Improving 6D point estimation accuracy and integrating assistance methods for better spatial positioning.

## Connections Between Milestones and Team Members

* **The Hand-off**: The Embedded team (Parth, Neal, Tauhid) is responsible for building the leg braces and ensuring the sensor data is clean and low-latency .

* **The Translation**: Cindy takes that raw data and retargets it so it makes sense for the robot's specific leg structure (URDF).

* **The Execution**: Darin and Yang use that retargeted data as the "goal" for the RL agent, training the robot to actually move its motors to match the human user.

| Week | Individual Task | Description | Assignee | Deadline | Status |
| :---- | :---- | :---- | :---- | :---- | :---- |
| **4** | **Retargeting Pipeline** | Mapping existing motion data onto our humanoid's URDF model.  | Cindy  | Apr 28 | In Progress |
| **4** | **Hardware Finalization** | Ordering all BNO085 IMUs and ESP32 boards for the leg array.  | Parth & Neal  | Apr 28 | Complete |
| **5** | **CV Baseline** | Setting up MediaPipe and FastAPI to get 3D poses from a camera.  | Cindy  | May 5 | To Do |
| **6** | **Sensor Prototyping** | Assembling the 6DOF wearable leg braces and checking fitment.  | Neal  | May 12 | To Do |
| **7** | **Embedded Networking** | Streaming real-time sensor data from the ESP32s to the Jetson.  | Parth  | May 19 | To Do |
| **8** | **Policy Optimization** | Modifying the IsaacLab RL environment with motion tracking rewards.  | Darin & Yang  | May 26 | To Do |
| **10** | **Sim2Real Deployment** | Running the trained policy on the actual humanoid hardware.  | Darin & Yang  | Finals Week | To Do |

## Proof of Milestone Achievement

Retargeting Pipeline

* Evidence Required  
  * Video or screenshot of existing motion capture data successfully loaded and mapped onto the humanoid URDF  
  * Visual confirmation in IsaacLab showing the robot skeleton matching the input motion data keypoints  
  * Output format documented (coordinate frame, Hz, data format)  
* Fallback  
  * Manual joint angle mapping from a simplified skeleton also counts as milestone achieved

Hardware Finalization

* Evidence Required  
  * Parts Procurement: Photo of all parts received (7x BNO085, 3x ESP32-C3, batteries, wiring)  
  * IMU Output: Terminal screenshot showing stable quaternion data output (timestamp, qx, qy, qz, qw) with no dropouts  
  * Synchronization: Video of all 3 ESP32s online simultaneously with aligned timestamps  
  * Joint Angles: Video of person bending knee showing hip, knee, and ankle angles updating correctly  
  * Jetson Connection: Screenshot of Jetson successfully receiving sensor data stream  
* Fallback  
  * Single ESP32 benchtop prototype outputting all 7 IMU readings also counts as milestone achieved

CV Baseline

* Evidence Required  
  * Video of MediaPipe \+ FastAPI running in real time, detecting human leg keypoints from an RGBD camera  
  * Screenshot showing 3D keypoint coordinates being output correctly (x, y, z per joint)  
  * Confirmed output format is compatible with the retargeting pipeline input  
* Fallback  
  * 2D keypoint output without depth also counts if 3D reconstruction is not yet complete

Sensor Prototyping

* Evidence Required  
  * Photo of assembled wearable leg braces with IMUs mounted  
  * Video of person wearing the braces and walking, confirming sensors stay secured  
  * Terminal output showing all 6 leg IMUs (3 per leg) reading simultaneously with no dropouts  
* Fallback  
  * Benchtop (non-wearable) assembly with correct sensor readings also counts

Embedded Networking

* Evidence Required  
  * Video showing real-time sensor data streaming from all 3 ESP32s to the Jetson via ESP-NOW  
  * Screenshot on Jetson side confirming received quaternion data stream with latency \< 50ms  
  * Timestamp alignment confirmed across all 3 ESP32s  
* Fallback  
  * Wired USB connection from a single ESP32 to Jetson with correct data output also counts

Policy Optimization

* Evidence Required  
  * Screenshot of IsaacLab training run showing reward curve increasing over time  
  * Video of simulated robot successfully tracking target leg keypoints (e.g., walking, squatting)  
  * Documented final definitions of:  
    * Observation space (joint positions, velocities, target keypoints)  
    * Action space (target joint angles via PD control)  
    * Reward function weights  
* Fallback  
  * Policy trained on existing motion capture data (without retargeted data) also counts

Sim2Real Deployment

* Evidence Required  
  * Video of trained policy running in MuJoCo (Sim2Sim validation) with stable locomotion  
  * Video of the same policy deployed on the actual humanoid robot successfully tracking basic lower-body movements (walking or squatting)  
  * Robot remains stable for at least 10 seconds without falling  
* Difficulties to address  
  * Domain gap between simulation and real hardware (friction, motor delay, sensor noise)  
  * Motor calibration required before deployment  
  * Safety cutoff mechanism must be in place during all real robot tests  
* Fallback  
  * Stable standing balance on real robot without motion tracking also counts as partial milestone achieved

# Reward & Policy Research

# Proposal: Motion-Tracking Reward and Policy Design for Humanoid Whole-Body Teleoperation

## Problem:

Get 3D human pose from embedded / RGBD / existing motion data \-\> retarget to humanoid 

\-\> train motion tracking policy \-\> track pose in sim \-\> deploy to real robot

## Policy proposal

Start with an MLP PPO actor-critic policy.

* The policy observes robot proprioception (its own status) and future target motion features in the robot local frame.  
* The action is target joint position, tracked by low-level PD controllers.

## Reward proposal

The reward should prioritize motion tracking:

* Keypoint/body-link position tracking  
* Root orientation and height tracking  
* End-effector tracking  
* Optional velocity tracking

Then add stability and regularization:

* Upright/alive  
* Action rate  
* Energy  
* Joint limit  
* Foot slip  
* Undesired contact  
* Death penalty

## Full SONIC?

SONIC’s encoder-decoder-token architecture is powerful because it unifies many different kinds of motion commands (robot motion, human motion, and hybrid motion) into one universal token space. We don’t need that many for now, and it’ll give more implementation complexity.

In my opinion, direct target-motion conditioning is more useful.

If we really want to do an encoder-decoder for the future, we can add a simplified encoder later once we support multiple input modalities.

## Conclusion

The most practical first step is not to reproduce the full SONIC architecture, but to implement a reliable motion-tracking RL baseline. We should use our existing PPO/rl-games locomotion stack, add target motion observations in the robot local frame, and design a reward dominated by body/keypoint tracking while preserving stability, energy, contact, and joint-limit penalties. SONIC-style encoders and decoders are valuable for future multi-modal control, but they should be introduced only after the basic retargeted motion tracking policy works reliably in simulation and sim2sim evaluation.

## Questions

1. Do we already have a dataset of our humanoid that we can use? Or do we only have human-pose data now?  
   1. Only have online dataset now

## Resources

Should be able to answer:

1. What problem does this paper solve?  
2. What is the policy input?  
3. What is the action space?  
4. What reward terms do they use?  
5. What can we copy or simplify for our project?

- [ ] [DeepMimic: Example-Guided Deep RL of Physics-Based Character Skills](https://arxiv.org/pdf/1804.02717)  
* Combining motion imitation objective and task objective  
* Trying to get reward functions out from here:  
  * Pose tracking  
  * Velocity tracking  
  * End-effector tracking  
  * Root tracking  
  * Task reward  
- [ ] [H2O: Learning Human-to-Humanoid Real-Time Whole-Body Teleoperation](https://arxiv.org/pdf/2403.04436)  
* Use RGB camera to get human pose \-\> retarget to humanoid-compatible motion \-\> train real-time humanoid motion imitator \-\> zero-shot sim2real to robot  
* Very similar to what we’re doing  
- [ ] [OmniH2O](https://arxiv.org/pdf/2406.08858)  
* Explains how to design a common interface from human keypoints / VR / sparse input to robot policy  
- [ ] [HumanPlus: Humanoid Shadowing and Imitation from Humans](https://arxiv.org/pdf/2406.10454)  
* Use human motion dataset to train low-level RL policy in sim  
* Let humanoid to use RGB camera follow human body and hand motion in real-time  
* Collect those data for behavior cloning  
- [ ] [PHC: Perpetual Humanoid Control](https://arxiv.org/pdf/2305.06456)  
* Explains how to recover the humanoid controller under noisy input, video pose estimates, unexpected falls, and scale up to large motion clips  
* Recovery / noisy input / fail-state handling are important for us since we’re using embedded / camera pose  
- [ ] [Humanoid-Gym](https://arxiv.org/pdf/2404.05695)  
* A RL framework for humanoid locomotion  
* Zero-shot sim2real  
* Isaac Gym \-\> MuJoCo  
* Important and highly aligned with our project\!  
- [ ] [Retargeting Matters / GMR](https://arxiv.org/pdf/2510.02252)  
* Describing the performance of motion tracking policy depends highly on retargeting quality  
* Problems in retargeting such as foot sliding, self-penetration, physically infeasible motion will make the RL hard to learn  
* Reward might not be the only problem but we have to get clean retargeted reference

### Future direction

- [ ] [AMP: Adversarial Motion Priors Make Good Substitutes for Complex Reward Functions](https://arxiv.org/pdf/2203.15103)  
* To let the policy motions to look better  
- [ ] [BeyondMimic](https://arxiv.org/pdf/2508.08241)  
* Learn high-quality humanoid motion tracking from human motions, and further use guided diffusion to do downstream versatile control  
- [ ] [Any2Track](https://arxiv.org/pdf/2509.13833)  
* Talks about the importance of domain randomization and online adaptation

Yang and Darin discussion.

Darin's thoughts:

1) Remove most rewards and then keep mainly motion tracking (L2 norm between given mocap and current read joint positions)  
2) Data curriculum (start with easy poses like basic walking, turning, then gradually increase difficulty like dancing)  
3) Change the way actions are applied:  
   Apply actions from the policy relative to the given mocap frame  
   This means the joint positions you use as reference change every timestep  
4) Giving the policy future reference frames as well  
5) (for later) can train a teacher policy which includes the future frames and then distill the teacher behavior into a student policy which only acts on present frames  
   This would reduce latency in real deployment.  
6) **(What)** GMT’s adaptive sampling handles this by clipping long motions and balancing difficult segments so the policy sees harder motions more often.  
7) Add recovery behavior  
   add randomized unstable initializations and an annealed upward assistance force

**CONSIDERATIONS:**

1) **How to pass data into the rewards? Bcs data lives on CPU, and training happens on GPU.? Bcs all motion data can't fit into memory on GPU. Usually sending data from CPU to GPU is slow**

Everytime step we start with motion data:  
\[x, y, z, w, a, b, c\] 7x ← this is pose information for each IMU on the human

The above gets retargeted to the robot and then the robot will copy the movement but also will obtain new information:  
\[j1, j2, j3, …, j10\] ← this is a vector of joint positions for the robot at each time step

To Do's:

1) Get motion data into isaaclab training so reward functions can access  
2) Data curriculum  
3) Modify the reward functions  
4) Modify the actions application (use latest reference frame instead of default pose)  
5) Modifying observations to pass in past, present, and future reference frames  
6) Add recovery behavior  
7) RUN TRAINING  
8) Evaluate in MuJoCo (need to create inference scripts for mujoco which will involve building the same observations in mujoco, and also doing a data pipeline in mujoco where the data comes from real world IMU readings \-\> human pose \-\> retargeter)

# Jetson Hotspot

For the IMU/ESP32 setup, we got the Jetson hotspot working.

Hotspot info:

Wi-Fi name: JetsonIMU  
Password: imu12345  
Jetson IP: 10.42.0.1

Any ESP32 should connect to `JetsonIMU`, then send data to the Jetson at:

10.42.0.1

Use HTTP, not HTTPS. Example test URL:

http://10.42.0.1:8000

On the Jetson, to turn the hotspot on:

\~/hotspot\_on.sh

To switch back to UCSD-GUEST Wi-Fi/internet:

\~/ucsd\_wifi\_on.sh

Important: since the Jetson only has one Wi-Fi connection, it usually cannot be on UCSD-GUEST and host the ESP32 hotspot at the same time. When we need internet/GitHub, switch to UCSD-GUEST. When testing ESP32s, switch back to `JetsonIMU`.

For ESP32 code, use:

const char\* ssid \= "JetsonIMU";  
const char\* password \= "imu12345";  
const char\* jetson\_ip \= "10.42.0.1";

# Milestone Report

# Triton Droids \- Humanoid Teleop

## 0\. Addressing Feedback

## Fortunately, we received full credit for our project specification, and no additional feedback was provided. With that said, we plan to proceed with the current project direction while refining the scope for the remaining two weeks. Rather than attempting to complete the full real-time teleoperation pipeline, we will focus on delivering a strong simulation MVP: loading retargeted lower-body motion data into IsaacLab, training a policy to track that reference motion, and evaluating the resulting behavior through videos and tracking metrics.

## 1\. Project Overview and MVP Status

### 1.1 Project Goal and MVP

Our project aims to develop a wearable lower-body motion tracking system for humanoid robot motion imitation. The system combines embedded wearable IMU nodes with a machine learning pipeline that reconstructs human lower-body pose and uses that information for humanoid motion tracking in simulation.

### 1.2 MVP Progress

Our embedded systems MVP focused on creating a reliable motion capturing pipeline that can collect movement data from 7 different IMUs with the help of ESP32s and stream it to the Jetson Nano on top of the humanoid in real time. So far, we have successfully validated the core communication and sensing pipeline. 

* Successfully tested 7 wearable devices (IMUs and ESP32s) for transmitting sensor data **wirelessly** and returning accurate results.  
* Successfully tested transmission from these 7 devices to the Jetson Nano mounted on the humanoid **wirelessly.**  
*  Confirmed that the human data generated from these devices can be used for robot retargeting. 


The machine learning side MVP focuses on validating that the computer vision baseline and IMU created human pose estimation works for driving a trained policy for motion tracking on a humanoid robot in simulation.

**At this stage the MVP demonstrates that the embedded hardware can reliably collect and transmit motion data necessary for humanoid control.**

| MVP Component | Description | Status |
| :---- | :---- | :---- |
| Embedded 6DOF sensor  | Build the wearable sensing unit using a microcontroller and IMU. This provides the raw motion data needed for the rest of the system. | COMPLETE |
| IMU / 6DOF pose estimation | Convert raw IMU readings into usable pose estimates, such as orientation or approximate 6DOF body-segment motion. | COMPLETE |
| WiFi data transmission | Stream sensor data wirelessly from the ESP-based wearable devices to the Jetson for real-time processing and logging. | COMPLETE |
| Human pose estimation | Use the wearable sensor data to estimate the user’s lower-body pose in a format that can be used by the robot pipeline. | IN PROGRESS |
| Retargeting to humanoid model | Map estimated human motion onto the humanoid robot’s URDF so the robot has feasible joint targets to follow. | COMPLETE |
| Computer Vision Baseline | A baseline using a Intel Realsense D435i for comparing our IMU human pose estimation | COMPLETE |
| Motion tracking RL environment | Modify the RL environment so the humanoid policy receives reference motion targets and is rewarded for tracking them. | IN PROGRESS |
| Verifying trained policy works in simulation | Test the trained policy in simulation to confirm the humanoid can follow the target motion stably and reasonably accurately, comparing the computer vision and IMU based human pose estimation. | IN PROGRESS |

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

**Goal**: Convert human lower-body pose data from IMU or computer vision inputs into robot-compatible lower-body motion targets. The expected output of this stage is a 10-dimensional robot joint position vector matching the humanoid URDF joint order.

**Completed Evidence**:

* [retargeting.mov](https://drive.google.com/file/u/1/d/1p0ynm6Arqcus_ENbCrR2Yxl2MD3ah-LD/view?resourcekey&usp=slides_web)  
* The above link shows our retargeting algorithm acting on our humanoid robot.

**Impact:** 

**2.2.2 MILESTONE IV**: Locomotion Pipeline \- COMPLETED

**Goal**: Build a working IsaacLab reinforcement learning pipeline for the humanoid robot so that future motion tracking policies can reuse the same simulation, robot model, action interface, and training infrastructure.

**Completed Evidence**:

* Implemented and registered the IsaacLab locomotion environment:  
  * Isaac-Humanoid-Locomotion-Flat-Direct-v0  
* Implemented a standing/balance environment:  
  * Isaac-Humanoid-Standing-Flat-Direct-v0  
* Built a 10-DOF lower-body control interface using target joint positions and PD control  
* Added PPO training support through RL-Games, with additional support for RSL-RL, SKRL, and SB3 configurations  
* Added domain randomization including friction, mass, gravity, joint parameters, push disturbances, observation noise, action noise, and latency  
* Added delayed PD actuator modeling based on motor-pair latency assumptions  
* Successfully produced training checkpoints and logs  
* [better\_locomotion\_v1.mov](https://drive.google.com/file/d/1A3ou17QwLKI-J7jG-QsdAsm-UAziRkec/view?usp=sharing)

**Impact:** This milestone gives the team a working RL simulation baseline. The motion tracking policy does not need to be built from scratch; it can extend the existing locomotion environment by replacing velocity-command tracking with reference-motion tracking.

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

Cindy Chen

* Adapted the Holosoma retargeting pipeline to our lower-body humanoid by integrating our robot's URDF and MuJoCo models, registering robot-specific joint limits, and tuning the floating-base configuration for stable optimization.  
* Resolved geometric mismatches between human and robot morphology by introducing a virtual pelvis marker at the true hip pivot location  
* Validated the pipeline on live motion data captured from an Intel RealSense D435 depth camera

Tauhid Malik:

* Developed the seven-IMU lower-body aggregation pipeline to convert pelvis, thigh, shank, and foot orientation data into joint positions and relative joint rotations.  
* Added tests for lower-body aggregation, missing sensor handling, standing pose reconstruction, and joint rotation accuracy.  
* Built a fake IMU simulation pipeline to test teleoperation workflows before relying on physical hardware.  
* Created demo scripts for lower-body aggregation and live skeleton visualization.  
* Helped test and verify the embedded hardware stack, including the seven IMUs, ESP32 boards, batteries, and Jetson Nano.  
* Supported embedded system bring-up by debugging hardware/software issues and assisting with integration between the sensors, microcontrollers, and Jetson.  
* Configured the Jetson Nano hotspot/network setup so all seven ESP32 devices could stream IMU data to the Jetson over UDP.  
* Assisted with end-to-end validation of the wearable sensing pipeline, from ESP32 IMU readings to Jetson-side data reception.

Neal Jian:

* Assisted in testing and validating the IMUs and ESP32 microcontrollers to ensure stable hardware functionality.  
* Developed the firmware scripts used for sensor communication and data handling.  
* Implemented the UART communication pipeline between the IMUs and ESP32s for reliable sensor data transmission.  
* Built the wireless transmission pipeline for streaming motion data from the embedded devices.  
* Tested the latency for wireless communication between the embedded devices and the Jetson Nano.  
* Developed the reverse kinematics pipeline used to translate quartanions from IMUs to 3D vectors.  
* Designed and prototyped rechargeable battery power for esp32 through 18650 Lipo batteries.

## 4\. Remaining Milestones

**4.1 MILESTONE V**: Human Pose Estimation Pipeline \- IN PROGRESS

**Goal**: Use wearable embedded devices to estimate the user’s lower-body pose in a format compatible with the humanoid robot pipeline.

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
* Modify action application so policy actions are residuals around the current reference joint frame:  
  * q\_des \= q\_ref\[t\] \+ residual\_action  
* Replace the current velocity-command reward with motion tracking rewards:  
  * joint position tracking, joint velocity tracking, root height/orientation tracking, foot position/contact tracking, and small safety penalties.  
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

* Define the retargeted dataset schema  
* Convert IMU or computer vision pose estimates into retargeted robot joint references.  
* Cache active motion clips on GPU during training to avoid slow CPU-to-GPU transfer every timestep.  
* Add a data curriculum starting from easy motions such as standing, weight shifting, walking, and turning, then gradually introduce more complex motions.  
* Add adaptive segment sampling so difficult motion segments are sampled more often after basic training is stable.

**Impact:** This milestone ensures that policy training has clean, robot-compatible motion references. It also reduces the risk that training fails because of noisy, infeasible, or poorly retargeted motion data.

## 5\. Timeline

| Timeline | Task | Owner | Expected Output |
| :---- | :---- | :---- | :---- |
| Days 1-2 | Finalize wearable IMU structure | Embedded | Having the wearable ready and tested to be able to send back complete data |
| Days 2-4 | Finalize retargeted robot reference format and confirm data sent back | Embedded \+ ML | A fixed schema containing important data |
| Days 3-5 | Modify observations and action application | ML | Policy receives current/future reference frames |
| Days 5-7 | Implement motion tracking reward | ML | Reward includes joint tacking, root tracking, foot/contact tracking |
| Days 7-10 | Run PPO training experiments | ML | Trained checkpoint for walking stably |
| Days 10-12 | Tune rewards and add simple curriculum | ML | Stable tracking on easier clips before adding turning or more difficult clips |
| Days 12-13 | Generate evaluation results  | ML | Videos, reward curves, tracking error metrics, and fall-rate measurements |
| Days 14 | Final integration and presentation prep | Embedded \+ ML | Demo video, milestone report updates, final project slides |

# Retargeting to RL Pipeline

5/19/2026

Summary  
Added ch\_robot support to the Holosoma retargeting converter and verified the full path from retargeted motion to IsaacLab/RL tracking .npz.

Changes Made

  1\. Added ch\_robot joint order to the converter config:

  "ch\_robot": \[  
      "left\_hip1\_joint",  
      "left\_hip2\_joint",  
      "left\_thigh\_joint",  
      "left\_knee\_joint",  
      "left\_ankle\_joint",  
      "right\_hip1\_joint",  
      "right\_hip2\_joint",  
      "right\_thigh\_joint",  
      "right\_knee\_joint",  
      "right\_ankle\_joint",  
  \]

  This lets the converter correctly map retargeted qpos\[:, 7:\] values into the MuJoCo robot joint order.

  2\. Fixed the converter’s hardcoded G1 DOF assumption.

  Previously it sliced:

  motion\[:, 7:36\]

  That assumes 29 DOF, which only works for G1. ch\_robot has 10 DOF, so the converter now slices based on the selected robot’s joint count:

  motion\[:, 7 : 7 \+ robot\_dof\]

  3\. Fixed input fps parsing.

  The retargeted .npz stores fps as 30, but the converter was treating the value like a timestep in some cases. Updated the logic so integer FPS values like 30 are handled correctly.

  4\. Added a \--headless converter option.

  The converter previously always launched a MuJoCo viewer. For batch conversion and CI-style workflows, added:

  \--headless

  This still runs mj\_forward() and computes body states/velocities, but doesn’t open the viewer.

  5\. Fixed setup scripts for paths containing spaces.

  Our local repo path includes Triton Droids, so unquoted shell variables broke setup. Quoted path variables in:

  scripts/setup\_retargeting.sh  
  scripts/source\_retargeting\_setup.sh

  Why  
  The retargeter output is not the final IsaacLab/RL tracking format.  
  Retargeting produces:

  qpos  
  fps  
  human\_joints  
  cost

  For ch\_robot, qpos is:

  (T, 17\) \= 7 floating base \+ 10 robot joints

  IsaacLab should consume the converted tracking .npz, which  
  contains:

  joint\_pos  
  joint\_vel  
  body\_pos\_w  
  body\_quat\_w  
  body\_lin\_vel\_w  
  body\_ang\_vel\_w  
  joint\_names  
  body\_names  
  fps

  Needed the converter to understand ch\_robot joint order and DOF count.

  Validation  
  Tested with synthetic data first, then with real OMOMO data.

  The full sub10\_largebox\_049.pt sequence has 222 frames. Direct retargeting to ch\_robot became infeasible around frame 158, likely because ch\_robot has only 10 DOF and the constraints become too strict for that part of the motion.

  To validate the pipeline with real data, clipped the first 120 frames:

  sub10\_largebox\_049\_clip120.pt

  Retargeting output:

  demo\_results/ch\_robot/robot\_only/omomo/sub10\_largebox\_049\_clip120.npz

  Verified:

  qpos: (120, 17\)  
  fps: 30  
  human\_joints: (120, 52, 3\)

  Converter output:

  converted\_res/ch\_robot/robot\_only/omomo/sub10\_largebox\_049\_clip120\_mj\_fps50.npz

  Verified:

  joint\_pos: (199, 17\)  
  joint\_vel: (199, 16\)  
  body\_pos\_w: (199, 28, 3\)  
  body\_quat\_w: (199, 28, 4\)  
  body\_lin\_vel\_w: (199, 28, 3\)  
  body\_ang\_vel\_w: (199, 28, 3\)  
  joint\_names: (10,)  
  body\_names: (28,)  
  fps: \[50\]

  joint\_vel being 16 is expected: MuJoCo free-joint velocity is 6D, while free-joint position is 7D. So:

  joint\_pos \= 7 base qpos \+ 10 joints \= 17  
  joint\_vel \= 6 base qvel \+ 10 joints \= 16

Command lines:

From retarget the first 120 frames of OMOMO dataset to our 10DoF robot:  
python examples/robot\_retarget.py \\  
    \--data\_path demo\_data/OMOMO\_new \\  
    \--task-type robot\_only \\  
    \--task-name sub10\_largebox\_049\_clip120 \\  
    \--data\_format smplh \\  
    \--robot ch\_robot \\  
    \--save-dir demo\_results/ch\_robot/robot\_only/omomo \\  
    \--retargeter.fix-orientation

Convert this retargeted data to RL IsaacLab training data:  
  python data\_conversion/convert\_data\_format\_mj.py \\  
    \--input-file demo\_results/ch\_robot/robot\_only/omomo/sub10\_largebox\_049\_clip120.npz \\  
    \--robot ch\_robot \\  
    \--data-format smplh \\  
    \--object-name ground \\  
    \--output-fps 50 \\  
    \--output-name converted\_res/ch\_robot/robot\_only/omomo/sub10\_largebox\_049\_clip120\_mj\_fps50.npz \\  
    \--headless \\  
    \--once

# Tab 12

Demo Launcher:  
Start The Demo Launcher

  cd /Users/tauhid/cse-145-237d-humanoid-teleop  
  conda run \--no-capture-output \-n humanoid-sim python demos/demo\_launcher.py

  Flash Each ESP32

  Edit:

  hardware/esp32\_bno085\_udp/secrets.h

  Set unique board/segment IDs, then upload esp32\_bno085\_udp.ino in Arduino IDE.

  Segment map:

  0 pelvis       LED white  
  1 left\_thigh   LED green  
  2 left\_shank   LED yellow  
  3 left\_foot    LED blue  
  4 right\_thigh  LED orange  
  5 right\_shank  LED purple  
  6 right\_foot   LED red  
  255 unknown    LED gray

  Example 5-board setup:

  SENSOR\_ID 0, SEGMENT\_ID 0 \= pelvis  
  SENSOR\_ID 1, SEGMENT\_ID 1 \= left\_thigh  
  SENSOR\_ID 2, SEGMENT\_ID 2 \= left\_shank  
  SENSOR\_ID 3, SEGMENT\_ID 4 \= right\_thigh  
  SENSOR\_ID 4, SEGMENT\_ID 5 \= right\_shank

  Check Packets

  In launcher, run:

  UDP Quaternion Receiver

  Expected:

  \- Each connected segment row updates around 100 hz.  
  \- drops should stay near 0\.  
  \- With only 5 ESP32s, complete fresh frame: False is normal because full frame expects all 7\.

  Run Live Viewer

  Use the correct launcher button:

  Partial IMU Viewer \- Thighs

  Needs:

  pelvis \+ left\_thigh \+ right\_thigh

  Partial IMU Viewer \- Shanks

  Needs:

  pelvis \+ left\_thigh \+ left\_shank \+ right\_thigh \+ right\_shank

  Partial IMU Viewer \- Full

  Needs all 7:

  pelvis, left\_thigh, left\_shank, left\_foot,  
  right\_thigh, right\_shank, right\_foot

  Before live viewer starts, stand still in neutral pose. The launcher auto-calibrates after about 1 second.

  Useful Terminal Commands

  Check laptop hotspot IP:

  ipconfig getifaddr en0

  Run receiver without launcher:

  conda run \--no-capture-output \-n humanoid-sim python demos/demo\_udp\_quaternion\_receiver.py \--host 0.0.0.0 \--port 5005

  Run latency ping, replacing IP with ESP32 IP from Serial Monitor:

  conda run \--no-capture-output \-n humanoid-sim python demos/demo\_udp\_latency\_ping.py ESP32\_IP\_HERE \--port 5006 \--count 20

  LED Meanings

  booting        yellow/orange  
  Wi-Fi joining  slow pulsing blue  
  Wi-Fi joined   green briefly  
  streaming      segment color  
  error          red

  Important Notes

  \- Uploading uart\_sanity.ino only checks BNO085 UART. It does not assign segment IDs or stream UDP.  
  \- Uploading esp32\_bno085\_udp.ino is what uses secrets.h.  
  \- If a color is wrong, the board probably has the wrong SEGMENT\_ID or has not been re-uploaded after the latest LED changes.  
