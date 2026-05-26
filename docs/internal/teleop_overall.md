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