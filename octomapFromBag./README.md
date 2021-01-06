## Visualising a point cloud and an octomap in RViz

### The bag file
As mentioned in the root readme, a [rosbag](http://wiki.ros.org/rosbag/Commandline) is a convenient way of storing and playing back data to the ROS framework.
You can manually find and download a rosbag from the internet but for the sake of this being a *tutorial*, I'll walk you through the steps that I followed to 
download a KITTI odometry sequence from [here](http://www.cvlibs.net/datasets/kitti/raw_data.php). Select the option to download the synced + rectified data. 
I downloaded the data sequence 2011_09_26_drive_0002 for the sole reason that it's small in size. 

We now have a [KITTI sequence](http://www.cvlibs.net/datasets/kitti/)! KITTI is a popular dataset used to test, train and evaluate one's computer-vision based 
research on. Sadly, ROS can't read a kitti sequence in its current form. We need to make the sequence a rosbag. To do so, follow the steps in the [kitti2bag](https://github.com/tomas789/kitti2bag)
repository, but do not `wget` the sequence again.

Now that we have a bagfile, we can see the message types and the topics that it publishes to with the command `rosbag info <name_of_the_bag>`. 

But wait, that doesn't work! We haven't *sourced* the setup.bash file from the catkin workspace that we wish to operate in yet! Remember to source the workspace
every single time that you open a new terminal. Now, try the above command once again and you'll see that it still doesn't work. Why? That's because there is no 
rosmaster yet. On a new terminal (by now you must remember the golden rule of sourcing the setup.bash file from the workspace, I will not mention that from here on) run `rosecore`. Now, on the terminal which
is in the directory of the bagfile, run the `rosbag info <name_of_the_bag>` command to see the details of the bagfile.

### Visualising the point cloud

Great! We now have a ROS bagfile! You can play this using the `rosbag play -r 0.2 <name_of_the_bag>` (the -r 0.2 tag increases the playback duration to 5 times the original value) but you won't be able to visualise anything just yet. For that we
will need to fireup [rviz](http://wiki.ros.org/rviz/UserGuide), which is the 3D visualisation tool for ROS, with the command `rosrun rviz rviz`. But what exactly do 
we want to visualise? What can we visualise? To answer that question we must (yup, you've {hopefully} guessed it) start another terminal and while the rosbag is playing,
we can run the command `rostopic list` to see a list of the current topics. The topic that we're interested in is `/kitti/velo/pointcloud`, to understand a little more 
about this topic, we can run `rostopic info /kitt/vel` (you can hit the tab key at this point, to undestand why, take a look at what the output of rostopic list is).

We see its message type `Type: sensor_msgs/PointCloud2` and the ID of the node publishing to it. This is what we wish to visualise in rviz.
Now, while the rosbag is still publishing, pull up the rviz window and navigate to the bottom left to the "Add" button. Click on that and then a 'create visualisation'
window will pop up, select the 'By topic' option and locate the pointcloud2 message by following the path `kitti/velo/pointcloud`. You might think that you're done
but you'll run into the error *For frame [velo_link]: Fixed Frame [map] does not exist*, what does this mean? Well it simply means that the coordinate frame where
the point cloud is being displayed isn't the same as the view from the coordinate frame currently being visualised and that there is no *static transform* encoding 
the relation between the two. To workaround this, we see the "Fixed Frame" parameter under the Global Options heading. We see that it is set to "map" which is the default 
for visualising any **SLAM-system**. On changing that parameter to 'velo_link', we see our point cloud being rendered!

###### What is a 'SLAM-system'
If you can image the pointclouds as what our eyes percieve, then the SLAM system is equivalent to us trying to paint our surroundings on a paper.
The act of us painting is the SLAM system, The paper and the brush are the octomap. Stereo reconstruction by itself is just that, reconstruction, not a SLAM system. 
The reconstruction (mapping) + odometry (localisation) gives us our SLAM (Simulataneous Localisation and Mapping) system. 

### Generating the octomap
In robotics, a very simple method to describe a scene would be to generate an "octomap". This has many varients but in its simplest form, what it does is 
that it breaks down the scene into 3D cubes known as "voxels" which have a binary value: 1 if there is an object in that voxel (i.e. it's occupied), 0 if there isn't an object in that voxel (i.e. unoccupied).
Advanced versions of an octomap prescribe a probability of a voxel being free or occupied and use multiple readings with a bayesian update rule (essentially an [Extended Kalman Filter](https://www.youtube.com/watch?v=mwn8xhgNpFY&list=PLn8PRpmsu08pzi6EMiYnR-076Mh-q3tWr)).
It isn't very computationally demanding to generate one of these and it doesn't take a lot of space to store but one major disadvantage of the same is that
we now lose information pertaining to the dynamics of the objects in the scene, that is, we do not know if the group of voxels classified as occupied determines a static obstacle
or a dynamic obstacle. This sort of information is essential in path-planning and thus the loss of it is a major drawback of the octomap mode of representing a scene, pointclouds, bounding boxes and meshframes 
are used in modern applications of robotics such as [Tesla's Autopilot](https://www.youtube.com/watch?v=zRnSmw1i_DQ) (I would insist that one checks out this link, it's super cool).

Nevertheless, generating an octomap is always a good starting point when trying to perform tasks on a scene. We clone the [octomap repo](https://github.com/OctoMap/octomap_mapping) into the 'src' folder of our catkin workspace.
Once this is done, we must build the workspace using `catkin build` and source the setup.bash file once again. we can use `roscd octomap_server` to directly change our present working directory to octomap_server. Go to the launch directory and copy-paste the
launch file from [here](https://github.com/sreeharshaparuchur1/ROS_starter_pack/blob/main/octomapFromBag./octomap_mapping_fromBag.launch). In another terminal, run the command `rosparam set use_sim_time True` (now the tf messages are published with the timestamp of the bagfile when run) followed by
`rosrun tf static_transform_publisher 0.810543903486 -0.307054359206 0.802724058277 0 0 0 1 map velo_link 100` (which now enables us to render visualisations in the map frame) and check the tf topic `rostopic info tf`. 
To generate an octomap, the octomap_server node needs a point cloud (of message type [pointcloud2](http://wiki.ros.org/pcl/Overview)) and a tf (transform of message type [tf2](http://wiki.ros.org/tf2/Tutorials/Introduction%20to%20tf2)) to generate an octomap.
You can find out what a message consists of (for example, the sensor_msgs/PointCloud2 message) by playing the rosbag file, executing `rosmsg show` and `rosmsg show sensor_msgs/PointCloud2`.

In another terminal, navigate to the directory of the rosbag, run `rosparam set use_sim_time True` followed by `rosbag play --clock -r 0.1 kitti_2011_09_26_drive_0002_synced.bag`. Finally (assuming that you have rviz open and Fixed Frame is "map"), run `roslaunch octomap_server octomap_mapping_fromBag.launch`.
You should now see an octomap on rviz (remember to zoom in/out, the visualisation in rviz works similar to that of open3d) with a voxel size of 1 meter. These paramters can be changed in the launch file, 
the link to the official octomap site which has all the possible tuneable parameters. Try adding a few of those into the launch file and playing around with the values.
Ask yourself some questions like what does the colour of the voxel represent, why does the octomap show every single voxel to be occupied (the road should be marked as free space) and how can I fix that?

Congratulations on making it this far! I solemnly hope that you now have some idea on how to navigate through the seas of ROS. Throught this tutorial, you have seen me linking to various pages instead of providing all the informaiton in this doc.
"Why so?", you might ask. Well that's because everything IS available on Google and when learning something new and daunting (to the level of ROS), you shouldn't be scared to google and ask for help.
You have to search on your own and be prepared to spend time on your project.







