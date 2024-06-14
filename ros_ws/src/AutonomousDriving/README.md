## Getting Started


1. Copy the src-folder to your repository and build it
2. Download the Unity Environment: https://syncandshare.lrz.de/getlink/fiEg9ocZ6Pc5iuEa4QqN1b/
3. Unzip the Unity file and copy the files to .../devel/lib/simulation/
4. Run a test:
  a.) roslaunch simulation simulation.launch
  b.) rosrun controller_node
  
The car will start driving and bump into the wall on the right. 
For testings you can as well manually control the car with w-a-s-d from Unity.



# Tips

Here are a couple of hints regarding the implementation. The hints are just suggestions; you are free so solve the task differently:
- The controller in controller_pkg courrently sends fixed values to the car. Start working here.
- Generating point cloud from depth image: use depth_image_proc in http://wiki.ros.org/depth_
image_proc.
- Generating occupancy Grid: use Octomap in http://wiki.ros.org/octomap.
- Please ping us in case you have any questions or if you get stuck in some subtasks.
- Use a global map as your voxel grid representation. Use a smart resolution for your voxel grid representation (e.g. 1m).

