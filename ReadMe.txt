--------------
Project
--------------

1. No Obstacles

a. RL_q_test_update.m : MATLAB training file
b. PolUpdated.csv : Resultant 199 x 199 policy matrix after training
c. QUpdated.csv : Resultant 8 x 199 x 199 Q matrix after training

d. DiscTest.cpp : cpp testing file (Accesses PolUpdated.csv))
e. disc.exe : example execution for No Obstacles case. Must be run from MobileRobots\ARIA\bin



2. Fixed Obstacles

a. RL_q_obst.m : MATLAB training file with map stored in map.csv
b. pol_obst_5_6.csv : Resultant policy matrix for goal (2250,2750)
c. pol_obst_6_10.csv : Resultant policy matrix for goal (4750,2750)

d. ObstTest.cpp: cpp testing file (Accesses policy csv files))
e. obst_5_6.exe : example execution for Fixed Obstacles case (goal (2250,2750)). Must be run from MobileRobots\ARIA\bin
f. obst_6_10.exe : example execution for Fixed Obstacles case (goal (4750,2750)). Must be run from MobileRobots\ARIA\bin



3. Dynamic Obstacles 

a. RL_q_obst_dyn.m : MATLAB training file
b. pol_obst_0.csv : Resultant policy matrix in case no obstacles detected
c. pol_obst_1.csv : Resultant policy matrix in case obstacle detected in zone 1
d. pol_obst_2.csv : Resultant policy matrix in case obstacle detected in zone 2
e. pol_obst_3.csv : Resultant policy matrix in case obstacle detected in zone 3
f. pol_obst_4.csv : Resultant policy matrix in case obstacle detected in zone 4


g. obst_dyn.cpp: cpp testing file (Accesses policy csv files))
h. obst_dyn.exe : example execution for Dynamic Obstacles case. Must be run from MobileRobots\ARIA\bin
i. obst_dyn.map : example map (using Mapper3Basic) for Dynamic Obstacles testing in MobileSim



4. VIVA

a. HiRes_Pics : High resolution version of pictures used in presentation
b. LowRes_Pics : Low resolution version of pictures used in presentation

c. BTP_final_review.pdf : PDF version of presentation
d. BTP_final_review.ppt : PPT version of presentation

e. MobileRobot_Video.mp4 : Video of physical mobile robot P3-DX testing cases



5. ME11B149_BTP_Report.pdf : Final Project Report





--------------
MobileRobots
--------------

THESE FILES ARE ONLY FOR 32 BIT WINDOWS SYSTEMS
FOR OTHER SYSTEMS, INSTALL THE APPROPRIATE BINARY

1. ARIA : README file contains instructions for settings while using VisualStudio on Windows. As mentioned above, with these settings, all exe files must be run from MobileRobots\ARIA\bin

a. ARIA\bin\1_disc.exe : No Obstacles case testing file
b. ARIA\bin\2_obst_5_6.exe : Fixed Obstacles case testing file (check for details above in Projects -> Fixed Obstacles)
c. ARIA\bin\2_obst_6_10.exe : Fixed Obstacles case testing file (check for details above in Projects -> Fixed Obstacles)
d. ARIA\bin\3_obst_dyn.exe : Dynamic Obstacles case testing file (check for details above in Projects -> Dynamic Obstacles). May load obstacle map (like obst_dyn.map) for testing in simulation.



2. Mapper3Basic : For making obstacle maps. Run Mapper3Basic.exe from MobileRobots\Mapper3Basic\bin



3. MobileSim : For running simulations. Run MobileSim.exe from MobileRobots\MobileSim. This opens a GUI from which you can choose to load a map.




-----------------------------------------------------------------------------------------------------------------------------