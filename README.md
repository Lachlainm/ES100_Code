Code Outline and Explanation: 

1. Ardupilot_LQR_Changed_Files:
   This folder contains only the files that were modified from the original Ardupilot source code, which were overview in the Build section of the thesis.
2. Demo_Video:
   This folder contains a video demonstration of the platform's capabilities.
3. ES100_Test_Data:
   This folder contains all the graphics created during the data analysis phase of the project, some of which are figures in the thesis. Also included here are the raw trajectory files for LQR testing. Lastly, this contains the hard coded version of the PPO policy and the raw .pth policy files.
3. LQR_Controller:
   This folder contains all the relavent Matlab files used to build the LQR controller, including the Simulink files.
5. ardupilot_PPO:
   This folder contains the entire ardupilot code base for those interested in looking through the general architecture. The specific modified files found in folder 1 will be changed to use the PPO controller in this folder.
6. gym_multirotor(Native PPO):
   This folder contains the full PPO simulation environment, including the files modified to encorporate the tilt-rotor drone, which are: tiltrotor_plus_hover.py, base_env.py, tiltrotor_plus_hover.xml, Visualize2.ipynb, visualize.ipynb, andtrain_sb3_ppo.py.

    
