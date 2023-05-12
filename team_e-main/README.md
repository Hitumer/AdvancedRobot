# ARCL Robothon SoSe 2023 Team E
Team E's repository for code, documentation and result submission.

Presentation with videos: https://docs.google.com/presentation/d/17xcduSn0HVbBVdS6hPw9Wy1gHTUkZGq8/edit?usp=sharing&ouid=112227696933899726489&rtpof=true&sd=true

## Requirements

+ CoppeliaSim Version 4.4.0(rev. 0)
+ CoppeliaSim Version 4.1.0(rev. 0) (task 2.2, 3.1, 3.2)
+ Pyrep
+ RLbenc for task 2.3

## How to run 


## Task 1
1. Run Coppeliasim Application
2. Go to file menu and open scene (Task_1/team_e_Task1.ttt)
3. Run Simulation (A small User Interface window pops up)
4. Enter Joints Position in csv format in the provided input box and press Enter
	for e.g. -2.3783212199964026, 1.3806609172751285, -2.281388442457785, -2.5530324557943875, -1.887167331183505, 1.1272934809736468, 1.3750410813247251
	Then either you could click on Reach target Joint to reach the target or click on Detect Collision
5. Enter the Target Cartesian Pose in the provided input box and press Enter
	for e.g. 0.411040052773964, -0.798643436077781, 0.439540082275394, 0.0, -0.633869092073425, 0.0961268791588709, 0.767431002838949, 0.0, -0.655167963447808, -0.594067190423387, -0.466732378279153, 0.0, -0.199684497549472, -0.521513280596378, 0.324080383823181, 1.0
	Then click on Reach Target Cartesian Pose to reach the pose
6. Press Start Position button to return to the start/default position


## Task 2
+ 2.1 
1. Go to Task_2 folder
2. run task2_1a for first scene
3. run task2_1b for second scene 


+ 2.2
1. Go to Team_e root folder
2. Run task2_2.py

+ 2.3
1. for resnet encoder lstm network run task2_3_model1.py
2. for our developed encoder lstm  network run task2_3_model2.py


### Regarding the question from the presentation: 
Q: For null space the end-effector stays at the same position. But why does its orientation still change? 

A: There was a mistake in our code. For the desired task variable sigma_desired_dot we didn't count the orientation of the end-effector. Now this problem has been fixed, but the orientation of end-effector still changes with no reason we could found. 
Thus, we have tried another solution: use P-controller to keep both the end-effector and the last (the 7th) joint of the robotic arm from leavng the desired position, which should also achieve the goal: keep the position and orientation of the end-effector at the same value. 
We have uploaded a new video for the second solution of task 2.2. It is called /Task_2/task2_2_new.mp4. 

## Task 3
1. Go to Team_e root folder
2. Run task3_1.py for task 3.1; run task3_2.py for task 3.2
