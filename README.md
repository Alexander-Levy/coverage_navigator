# Coverage Navigator

Simple python interface written to interact with the `NavigateCompleteCoverage` action server and the rest of the navigation stack. This is being build and tested on ROS2 humble. Not ready. Still Testing. Missing stuff. Only the coverage server is working at the moment. 

## TODO
Pretty much everything, still very early. 
 - [ ] write readme
 - [ ] integrate rest of navigation stack (at least the basics)
 - [ ] clean up code
 - [ ] provide examples 

## Usage

Note: Still a prototype, only has part of funtionality at the moment.

You can give the robot a field to cover using the following command. It will ask you to input a field and then foward the command.
```bash
ros2 run coverage_navigator test_coverage
```

## Dependencies

### The navigation stack:
```bash
sudo apt install -y                         
    ros-humble-navigation2   \
```

### Fields2Cover and Opennav_Coverage 
First go to your work space or create if you have not already
```bash
cd ~
mkdir -p <path_to_your_workspace>/src
cd <path_to_your_workspace>/src
```
Then download the following files
```bash
git clone https://github.com/Fields2Cover/Fields2Cover.git -b v1.2.1
git clone https://github.com/open-navigation/opennav_coverage.git -b humble
```
Go to the main project folder of Fields2Cover and build the library:
```bash
cd ~/<path_to_your_workspace>/src/Fields2Cover
mkdir -p build; 
cd build; cmake -DCMAKE_BUILD_TYPE=Release ..; 
make -j$(nproc); 
sudo make install;
```
Finally, get back to the main project folder, build, source & test
```bash
cd ~/<path_to_your_workspace>
colcon build --symlink-install
source install/setup.bash
ros2 launch opennav_coverage_demo coverage_demo_launch.py
```

## Install 
To use this package please download all of the necesary dependencies first and then follow these steps
```bash
cd ~/<path_to_your_workspace>/src
git clone https://github.com/Alexander-Levy/coverage_navigator.git 
cd ..
colcon build --symlink-install
```

