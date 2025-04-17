# ENPM663_Assignment2

**_Please create your own branch, only merge to main if you know everything you have is fully working_**

This repo only contains the package, the rwa2_spring2025.yaml will need to be manually added to the ariac_gazebo package within ARIAC

To run, launch terminals in following order
first run ariac in one terminal using:

ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa2_spring2025

Then open another terminal and run
ros2 run group1_ariac retrieve_orders.py

Finally in another terminal run
ros2 run group1_ariac check_competition_status.py
