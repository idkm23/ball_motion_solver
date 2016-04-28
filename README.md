# ball_motion_solver

C++ Component of the odg ball demo. The program recieves images from the ODG through ros  
and then determines where the ball is and sends that coordinate back to the glass, that way the glass
doesn't have to do as much processing.  
  
Setup:  
```
# navigate to your myo_raw package's workspace and source it (```source devel/setup.bash```)  
  
# navigate to a catkinworkspace/src, if you don't have one:  
# mkdir my_workspace/src  
# cd my_workspace/src  
# catkin_init_workspace  
  
git clone https://github.com/idkm23/ball_motion_solver.git  
cd ..  
catkin_make  
source devel/setup.bash  
roslaunch ball_motion_solver odg_tip_finder.launch 
```  
  
If you want to have the myo do anything in the system, you must also train the classifier in myo_raw.
(myo is not required though)  
```
roscd myo_raw  
./scripts/myo_classify.py
```  
Hit a number on your keyboard (not the numpad) and it will start to record points (train the system).
For this program, 1 should be squeezing, and 0 should be not squeezing.
