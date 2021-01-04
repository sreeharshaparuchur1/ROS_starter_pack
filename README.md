# ROS_starter_pack
Hey there! Below, I have compiled a guide to help get the basics of ROS out of the way so that on understanding the below topics, one should find the path ahead to be not-so-smooth (ROS is vast and this is only meant to be a small introductory document to understand and on how to use ROS and some of its convenient debugging features, google is your best friend when it comes to learning how to use this system and the [roskwiki](http://wiki.ros.org/Documentation) page provides great documentation and starter code), but that's better than nothing. Feel free to contact me on [LinkedIn](https://www.linkedin.com/in/sreeharshaparuchuri/) if you have any questions or would like to contribute to this repo. Happy Coding!

The version of ROS that I currently use is Melodic as it is well supported on Ubuntu 18 which now has good compactability with laptops running an Nvidia graphics card (Nividia compactability really plagued the user experience on Ubuntu 16). My configuration is an i7 system with SSD and an Nvidia 1050Ti graphics card, these details shouldn't matter too much though but when it comes to compute power and RAM, the more the merrier :)  

Starting out, ROS can be quite daunting given the loquatious amount of documentation for it and the recondite jargon that comes with it. Nevertheless, below is my best attempt at helping one overcome this fear of using this software and I'd like to break the ice by saying that while the scope of topics involved in ROS is very vast, it is also quite well-orgainsed and things come together very neatly, like the puzzle pieces of a jigsaw.

## Basic Installations

What follows references something known as a "ROS package", simply put, this contains your code and data. We will come back to this later, but for now, this definition will suffice. `roscd` is a command line command that allows you to directly switch between ros packages without using the cumbersome `cd` bash command.

[Catkin](http://wiki.ros.org/catkin) is a workspace which houses three main directories: build, devel (development) and src (source). Whenever you fireup a terminal and access a catkiun workspace, if you do not wish to modify your bash file then ALWAYS remember to *source* your setup.bash file as `source ./devel/setup.bash`. While the official ROS docs use the command `catkin_make` to "build" the workspace after installing a package, i would recommend that one uses the [catkin CLI interface](https://github.com/catkin/catkin_tools) command `catkin build`. This builds your packages in "isolation", essentially, it's a much cleaner build. Remember to source the setup.bash file after building the workspace.

#### Ubuntu installation procedures and what they mean
A common way for Linux users to install a library / software / package is via the command:

` sudo apt-get install <package-name> `

sudo is super-user-do, gives you adminstrator permissions, apt-get is a server by ubuntu that hosts packages (from Canonical) to install, install will install the said packages. 
The important thing to note is that it doesn't matter what directory you're in. If it's a ros package you're installing, the Linux system knows to put the package somewhere in /opt/ros/melodic/. When using apt-get, all the dependencies of the package that is to be installed, will be installed too. Packages downloaded this way are accessible system-wide and you **do not** need to be in a specific directory to access the software. A common installer for python is 'pip' (or pip3, depending on python version) which installs python packages directly from PyPI.

For general ros packages such as [rviz](http://wiki.ros.org/rviz), [rqt_graph](http://wiki.ros.org/rqt_graph), [gazebo](http://wiki.ros.org/gazebo_ros_pkgs) etc., usually come pre-built with your ros installation. Nevertheless, if it's an important package that you will universally use with ros, you must sudo apt-get install that package. This way you can call the package from anywhere. (Not just from catkin_ws/src)

In the case of [octomap](https://github.com/OctoMap/octomap), it's a very general package because you can use it to build maps in any setting you want. Therefore, you sudo apt-get install it.

The packages that you're experimenting with, say something like LeGo-Loam, they need to be downloaded from GitHub and built using catkin inside your ros workspace. 

Please follow the ROS [installation guide](http://wiki.ros.org/melodic/Installation/Ubuntu) to setup ROS on your system.
You can check that it's properly installed by launching some of the inbuilt packages by using the `rosrun` [command](http://wiki.ros.org/rosbash#rosrun). You can try running the following commands on seperate terminals (don't forget to source the setup.bash file):

`rosrun rviz rviz`

`rosrun rqt_tf_tree rqt_tf_tree`

Do remember that the tab key is your friend! Just like the autocomplete feature on pressing the tab key while on terminal, the tab key can auto fill ros commands. Try it out with `rosrun rqt_gr .. (press the tab button twice)`.

#### ROS topics, messages and nodes (publisher and sub) 
###### When running something ALWAYS remember to run `roscore` on one of your terminals 
 remember to source the setup.bash file first though. ROS itself is a mighty communication intercase and the rosmaster (aka roscore) is the brain and muscle behind that. It basically starts up nodes essential to the functioning of the programme. If you use a `roslaunch` command then a roscore is automatically started up if not up already but I wouldn't advise one to bank on that in the initial stages.
 
 

#### Useful commandline commands


#### Debugging tools in ROS
