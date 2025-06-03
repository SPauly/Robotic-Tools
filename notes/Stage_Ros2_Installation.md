# Installing Stage for ROS2

Installation of stage_ros2 on Ubuntu 24.04: (not tested in all configurations)

- Update the needed development packages and tools:
  
  - ```bash
      sudo apt-get install git cmake g++ libjpeg8-dev libpng-dev libglu1-mesa-dev libltdl-dev libfltk1.3-dev
    ```

  - install FLTK of version 1.3 instead of 1.1 here

- Install stage as dependency for the stage_ros2 bridge: (only works manually on Ubuntu 24.04)
  
  - ```bash
      cd YOUR_ROS2_WORKSPACE && mkdir src && cd src
      git clone --branch ros2 https://github.com/tuw-robotics/stage.git
      cd stage
      mkdir build && cd build
      cmake ..
      make # <- this will take a while or use -j<number_of_cores> to speed it up (not recommended on wsl2)
      sudo make install # <- will install the needed cmake files for us
    ```

- Make sure stage_ros2 finds the stage installation:

  - ```bash
      export CMAKE_PREFIX_PATH=/usr/local/lib/cmake:$CMAKE_PREFIX_PATH # <- this might be different on your system or not needed when installing in the same workspace
    ```

- Install stage_ros2: Either as a submodule or as a normal git clone:

  - ```bash
      cd YOUR_ROS2_WORKSPACE/src 
      git clone --branch humble https://github.com/tuw-robotics/Stage.git
      cd YOUR_ROS2_WORKSPACE
    ```

  - or as a submodule:

    - ```bash
      cd <workspace>/src
      git submodule add --branch humble https://github.com/tuw-robotics/stage_ros2.git stage_ros2
      ```

- Make sure the python ros development tools are installed:

  - ```bash
      cd YOUR_ROS2_WORKSPACE
      sudo apt install python3-rosdep
      sudo rosdep init
      rosdep update
      rosdep install --from-paths src --ignore-src -r -y # this installs all missing dependencies of your pkgs
    ```
  
- Build the workspace:

  - ```bash
       colcon build --symlink-install --cmake-args -DOpenGL_GL_PREFERENCE=LEGACY
       colcon build --symlink-install --packages-select stage_ros2
    ```

- Source Workspace and run the stage_ros2 example:

  - ```bash
      source install/setup.bash
      ros2 launch stage_ros2 stage.launch.py world:=cave enforce_prefixes:=false one_tf_tree:=true
    ```

Full example of the stage_ros2 example:

```bash
# Install dependencies
sudo apt-get install git cmake g++ libjpeg8-dev libpng-dev libglu1-mesa-dev libltdl-dev libfltk1.3-dev

# Build and install stage
cd YOUR_ROS2_WORKSPACE
mkdir src 
cd src
git clone --branch ros2 https://github.com/tuw-robotics/Stage.git
cd stage
cmake -S . -B build
cd build
make
sudo make install # install cmakefiles
export CMAKE_PREFIX_PATH=/usr/local/lib/cmake:$CMAKE_PREFIX_PATH # not needed when installing in the same workspace

# Install stage_ros2
cd YOUR_ROS2_WORKSPACE/src
git clone --branch humble https://github.com/tuw-robotics/stage_ros2.git
cd YOUR_ROS2_WORKSPACE

# Update the needed development packages and tools
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y  # this installs all missing dependencies of your pkgs

# Build the workspace
colcon build --symlink-install --cmake-args -DOpenGL_GL_PREFERENCE=LEGACY
colcon build --symlink-install --packages-select stage_ros2        
```

- How to use it in other packages:

  - For easy access add sourcing of the workspace setup to the bash

    - ```bash
        echo "source ~/YOUR_ROS2_WORKSPACE/install/setup.bash" >> ~/.bashrc
        ```
