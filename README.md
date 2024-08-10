## Interplanetar Workshop 2024: Software
# Lecture-4: ROS Packages

In this lecture, we will take a look at ROS packages. We will be discussing on what ROS packages are, the structure of ROS packages, the build tools utilized to create a package, and how to use them. We will go through examples of creating a ROS package from scratch, as well as installing existing ROS packages.

## Table of Content
1. [Introduction](#intro)
2. [ROS Package Structure](#structure)
3. [Introduction to Build Tool: Catkin](#catkin-intro)
4. [Creating ROS Packages](#create)
    - [Creating From Scratch](#scratch)
    - [Installing Pre-existing Packages](#existing)
        - [From GitHub](#github)
        - [From Apt](#apt)
5. [Using ROS Packages](#usage)
6. [Creating Launch Files](#launch)
7. [Resources](#ending)

## Introduction <a class="anchor" id="intro"></a>

Software in ROS is organized in packages. A package might contain ROS nodes, a ROS-independent library, a dataset, configuration files, a third-party piece of software, or anything else that logically constitutes a useful module. The goal of these packages it to provide this useful functionality in an easy-to-consume manner so that software can be easily reused. In general, ROS packages follow a "Goldilocks" principle: enough functionality to be useful, but not too much that the package is heavyweight and difficult to use from other software.

For more information: https://wiki.ros.org/Packages

## ROS Package Structure <a class="anchor" id="structure"></a>

A ROS package folder has directories in it based on type and functionality:

- `CMakeLists.txt`: Contains build information for CMake.

- `package.xml`: Contains metadata for the package (e.g., package name, maintainer's information, dependencies).

- `src`: Contains the C++ files for nodes designed in C++.

- `include`: Contains header files required for the C++ nodes.

- `launch`: Contains the launch files for that package.

- `msg`: Contains custom ROS message types for that package (if created).

- `srv`: Contains files for ROS services in that package.

- `action`: Contains ROS action files.

- `config`: Contains necessary config(.yaml) files for the package (if needed).

- `urdf`: Contains robot description files that contain information for constructing a 3D visualization of a robot using STL files (if created).

- `meshes`: Contains the necessary STL files needed to construct a 3D visualization of a robot (if created).

- `scripts`: Contains executable python files for the nodes designed in Python.

- `README.md`: Contains description about the package.

A typical ROS package may have a structure like this:
```
my_package/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── my_package_node.cpp
├── include/
│   └── my_package/
│       └── my_package_node.h
├── launch/
│   └── my_package.launch
├── msg/
│   └── MyMessage.msg
├── srv/
│   └── MyService.srv
├── action/
│   └── MyAction.action
├── config/
│   └── my_package.yaml
├── urdf/
│   └── my_robot.urdf
├── meshes/
│   └── my_robot_part.stl
├── scripts/
│   └── my_script.py
└── README.md
```


## Introduction to Build Tool: Catkin <a class="anchor" id="catkin-intro"></a>

A *build system* is responsible for generating 'targets' from raw source code that can be used by an end user. These targets may be in the form of libraries, executable programs, generated scripts, exported interfaces (e.g. C++ header files) or anything else that is not static code. In ROS terminology, source code is organized into 'packages' where each package typically consists of one or more targets when built.

CMake is a popular build system that is widely used for compiling C++ code. However, ROS supports multiple language support, mainly C++ and Python. To compile executable code from multiple files written in multiple languages and seamlessly use them together, ROS uses a custom build tool called **Catkin**.

Catkin combines CMake macros and Python scripts to provide some functionality on top of CMake's normal workflow. Catkin was designed for better distribution of packages, better cross-compiling support, and better portability.



## Creating ROS Packages <a class="anchor" id="create"></a>
In order to create and use a ROS package, we need to have a ROS workspace set up with catkin and build the packages in it. For creating a workspace, we need to create a folder and run `catkin_make`.

```
$ cd ~/
$ mkdir tutorial_ws
$ cd tutorial_ws
$ mkdir src
$ catkin_make
```

> **Note:** You have to run `catkin_make` at the **workspace** folder every time you add or create a new package. To streamline this process and better manage the packages, we can use *catkin_tools* commands instead of running `catkin_make`.
> To install `catkin_tools`, follow the instructions [here](https://catkin-tools.readthedocs.io/en/latest/installing.html). Then, you can run the following commands:
>```
>$ cd ~/
>$ mkdir tutorial_ws
>$ cd tutorial_ws
>$ mkdir src
>$ catkin build
>```
> **Note:** You should use *either* `catkin_make` *or* `catkin build` for building packages, not *both*.

### Creating From Scratch <a class="anchor" id="scratch"></a>

To create a ROS package from scratch, run the `catkin_create_pkg` command on the terminal at the `tutorial_ws/src` directory. The syntax for the command is as follows:

```
catkin_create_pkg package_name dependency1 dependency2 dependency3
```

Here, the `package_name` is the name you want to give to your package, and `dependency1`,... are the packages that will be used in your package as dependencies.

Here's an example of creating a package:

```
# assuming the workspace is created in ~/tutorial_ws directory
$ cd ~/tutorial_ws/src
$ catkin_create_pkg tutorial_pkg roscpp rospy std_msgs geometry_msgs
```

Now you should see your package being built, and the package directory should look something like this:

```
tutorial_ws
|--- build
|--- devel
|--- src
    |--- tutorial_pkg
        |--- CMakeLists.txt
        |--- package.xml
        |--- src
```

If you decide to add/remove dependencies to your package, you will need to update your `package.xml` file. Add the dependencies as `<depend>` and `<exec_depend>` in the `package.xml` file. For Python nodes, you will need to add that dependency as `<exec_depend>`. For example:

```
<exec_depend> dependency4 </exec_depend>
```

For C++ nodes, you will need to add dependencies as `<build_depend>` and/or `<exec_depend>` based on what type of dependency that package is.

```
<build_depend> dependency4 </build_depend>

<exec_depend> dependency4 </exec_depend>
```

To learn more about different types of dependencies, check [this](https://wiki.ros.org/catkin/package.xml#Build.2C_Run.2C_and_Test_Dependencies) out.

### Installing Pre-existing Packages <a class="anchor" id="existing"></a>

There are many useful ROS packages out there that can be installed into a workspace. We will be covering how you can install an external ROS package using GitHub and Apt.

#### From GitHub <a class="anchor" id="github"></a>

All you have to do is clone the package repository into the `src` forder of your workspace, and run `catkin_make` as usual to build the package.

```
# installing package from GitHub
$ cd ~/tutorial_ws/src
$ git clone -b <branch> <address>
$ cd ..
$ catkin_make
```
> **Note:** Generally, the ROS packages are organized in Git branches according to the ROS versions. If you're installing from the official ROS Github repo (https://github.com/ros), make sure to check out the branch that contains the ROS version your device is running. If you're checking out other repositories, make sure to download the package folder and paste the folder in the `src` folder of your workspace and build the package.

#### From Apt <a class="anchor" id="apt"></a>

Open a terminal and run the apt command to download a package.

```
# installing package using apt-get
$ sudo apt-get install <package-name>
```

For the purpose of this lecture, we are going to install the **Turtlesim** package by running the command:

```
$ sudo apt-get install ros-$(rosversion -d)-turtlesim
```

## Using ROS Packages <a class="anchor" id="usage"></a>

In order to use any Python node in a ROS package, we first need to make the .py files executable. We can do this with command lines by using the `chmod +x` command.

```
# making python nodes executable
$ sudo chmod +x /dir/to/python_node.py
```

For C++ nodes, we do not need to do this step.

Once that's done, we need to source the `setup.bash` file of the workspace in order to use the nodes in the package. We need to repeat this process every time we switch to a new terminal.

```
$ cd ~/tutorial_ws
$ source devel/setup.bash
```

We also need to source the `setup.bash` file for ROS every time we switch to a new terminal.

```
$ source /opt/ros/noetic/setup.bash # if the ROS version is 'noetic'
```
> **Note:** Sourcing ROS every time for a new terminal is a cumbersome process. To avoid doing that, we can write this command in the `~/.bashrc` file, so that every time a new terminal is created, the sourcing is done automatically. 
>
>Open the `~/.bashrc` file with a text editor and write down the command in the file.
> ```
>$ source /opt/ros/noetic/setup.bash # if the ROS version is 'noetic'
>```
> Save the file, and now every time a terminal opens, the `setup.bash` file for ROS will already be sourced. 

Once all the sourcing is done, we can use `rosrun` command to run a node from the sourced packages.

```
$ rosrun tutorial_pkg python_node.py 
```

For C++ nodes, we do not need to mention the extension of file name when running a node.

```
$ rosrun tutorial_pkg cpp_node
```

We can also check if a certain package exists that can be used using the `rospack find` command.

```
$ rospack find <package-name>
```

If the package exists and is usable, you should see the directory of the package. If the package is not found, the package either does not exist, or it is not sourced.


## Creating Launch Files <a class="anchor" id="launch"></a>

Launch files are very handy for running multiple ROS nodes simultaneously. These are XML files that contain information on which nodes to run. The structure of a launch file is as the following:

```
<?xml version="1.0"?>
<launch>

    <!-- "arg": used for setting variables within launch files -->
    <arg name="my_arg" default="my_value"/>
    <arg name="my_other_arg" default="true"/>

    <!-- Node-1 (Python) -->
    <node pkg="ros_package" type="my_node.py" name="python_node">
    </node>
    
    <!-- Node-2 (C++) -->
    <node pkg="ros_package" type="my_other_node" name="cpp_node" output="screen">
        <!-- "remap" changes the name of a topic to something else -->
        <remap from="my_topic_name" to="other_topic_name"/>
    </node>

    <!-- "group": can be controlled with an argument -->
    <group if="$(arg my_other_arg)">
        
        <!-- "include": lets us run other launch files within a launch file -->
        <include file="$(find ros_package)/launch/some_other_launch_file.launch">

            <!-- args can be passed into nodes and launch files -->
            <arg name="my_arg" value="some_value"/>

        </include>

    </group>

</launch>
```

Some of the important tags are described below:

- `<launch></launch>`: Elements in a launch file are kept within this tag.
- `<node></node>`: Used for launching a node. Attributes include:
    - `pkg`: Name of the package containing the node.
    - `type`: Name of the executable file for the node (name of the .py or .cpp file). For Python node, the .py extension must be mentioned, but .cpp files the extension is not needed to include.
    - `name`: The name that will be given to the node.
    - `output`: Decides what you want to do with the output. Leaving this argument empty will not print any output generated by the node to the terminal. `output="screen"` lets the node to print the output to the terminal.
- `<arg/>`: Variables that can be used throughout the launch file. Arguments include:
    - `name`: Name of the variable.
    - `value`: Value of the variable.
    - `default`: Default value of the variable.
- `<include></include>`: Lets us run other launch files within the launch file. Arguments include:
    - `file`: Directory to the launch file that we want to run from the current launch file.
- `<remap/>`: Lets us change the name of a topic to something else. Arguments include:
    - `from`: Topic name that will be changed.
    - `to`: New name of the topic.
- `<group></group>`: Used for creating a group of nodes and launch files. We can decide if a specific group will be launched or not by using an `arg` or `param` of boolean type. Arguments include:
    - `if`: Value of argument that decides whether or not to launch the group. `if="$(arg my_arg)"` means the group will be launched if the value of `my_arg` is `true`.
    - `unless`: Value of argument that decides whether or not to launch the group. `unless="$(arg my_arg)"` means the group will be launched if the value of `my_arg` is `false`.

Save the launch file as `<filename>.launch`. Launch files must be put in the `launch` folder in your package.

> **Note:** the Python nodes must be made executable before using.

After doing that, you have to source your terminal, and use `roslaunch` command to run a launch file.

```
$ cd ~/tutorial_ws
$ source devel/setup.bash
$ roslaunch tutorial_pkg <filename>.launch
```
## Resources <a class="anchor" id="ending"></a>

GitHub repository for the package built in the tutorial: https://github.com/shawbicc/tutorial_pkg.git

GitHub repository for the resources needed for the assignment: 

https://github.com/shawbicc/ros-playground.git (branch: `ros-noetic-devel`)

Follow along the recorded lecture on ROS Packages for hands-on tutorials and examples on how to create ROS packages and launch files. Be sure to try the codes yourself. Some of the files used in this tutorial are already provided in a GitHub repository. It is highly encouraged to get used to the GitHub workflow and use that with your projects as well.
